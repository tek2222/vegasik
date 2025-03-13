#include "draw_stl.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Helper functions
static void* malloc_safe(size_t size) {
    void* ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    memset(ptr, 0, size);
    return ptr;
}

// STL file header structure
typedef struct {
    char header[80];
    uint32_t num_triangles;
} __attribute__((packed)) STLHeader;

// STL triangle structure (as stored in file)
typedef struct {
    float normal[3];
    float vertex1[3];
    float vertex2[3];
    float vertex3[3];
    uint16_t attribute;  // Attribute byte count
} __attribute__((packed)) STLTriangle;

// STLMesh functions
STLMesh* stl_mesh_create(void) {
    STLMesh* mesh = (STLMesh*)malloc_safe(sizeof(STLMesh));
    mesh->triangles = NULL;
    mesh->vertex_buffer = NULL;
    mesh->normal_buffer = NULL;
    mesh->num_vertices = 0;
    return mesh;
}

void stl_mesh_destroy(STLMesh* mesh) {
    if (!mesh) return;
    free(mesh->triangles);
    free(mesh->vertex_buffer);
    free(mesh->normal_buffer);
    free(mesh);
}

int stl_mesh_load(STLMesh* mesh, const char* filename) {
    if (!mesh || !filename) return 0;
    
    FILE* file = fopen(filename, "rb");
    if (!file) {
        fprintf(stderr, "Could not open STL file: %s\n", filename);
        return 0;
    }
    
    // Read header
    STLHeader header;
    if (fread(&header, sizeof(STLHeader), 1, file) != 1) {
        fprintf(stderr, "Failed to read STL header\n");
        fclose(file);
        return 0;
    }
    
    // Allocate triangles
    uint32_t num_triangles = header.num_triangles;
    printf("Loading STL file with %u triangles\n", num_triangles);
    
    mesh->triangles = (Triangle*)malloc_safe(num_triangles * sizeof(Triangle));
    mesh->num_vertices = num_triangles * 3;
    
    // Read triangles
    STLTriangle stl_tri;
    for (uint32_t i = 0; i < num_triangles; i++) {
        size_t bytes_read = fread(&stl_tri, sizeof(STLTriangle), 1, file);
        if (bytes_read != 1) {
            fprintf(stderr, "Failed to read triangle %u (read %zu bytes)\n", i, bytes_read * sizeof(STLTriangle));
            fprintf(stderr, "File position: %ld\n", ftell(file));
            fclose(file);
            return 0;
        }
        
        // Convert normal
        mesh->triangles[i].normal = vector3_create(
            stl_tri.normal[0],
            stl_tri.normal[1],
            stl_tri.normal[2]
        );
        
        // Convert vertices
        mesh->triangles[i].vertices[0] = vector3_create(
            stl_tri.vertex1[0],
            stl_tri.vertex1[1],
            stl_tri.vertex1[2]
        );
        
        mesh->triangles[i].vertices[1] = vector3_create(
            stl_tri.vertex2[0],
            stl_tri.vertex2[1],
            stl_tri.vertex2[2]
        );
        
        mesh->triangles[i].vertices[2] = vector3_create(
            stl_tri.vertex3[0],
            stl_tri.vertex3[1],
            stl_tri.vertex3[2]
        );
    }
    
    // Check if we've read the entire file
    long file_size = 0;
    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    fclose(file);
    
    long expected_size = sizeof(STLHeader) + num_triangles * sizeof(STLTriangle);
    if (file_size != expected_size) {
        fprintf(stderr, "Warning: File size mismatch. Expected %ld bytes, got %ld bytes\n", 
                expected_size, file_size);
    }
    
    printf("Successfully loaded %u triangles\n", num_triangles);
    
    // Compute vertex and normal buffers for OpenGL
    stl_mesh_compute_buffers(mesh);
    
    return 1;
}

void stl_mesh_compute_buffers(STLMesh* mesh) {
    if (!mesh || !mesh->triangles || mesh->num_vertices == 0) return;
    
    // Allocate buffers
    mesh->vertex_buffer = (GLfloat*)malloc_safe(mesh->num_vertices * 3 * sizeof(GLfloat));
    mesh->normal_buffer = (GLfloat*)malloc_safe(mesh->num_vertices * 3 * sizeof(GLfloat));
    
    // Fill buffers
    int vertex_idx = 0;
    int normal_idx = 0;
    
    for (int i = 0; i < mesh->num_vertices / 3; i++) {
        // Store normal (same for all 3 vertices of triangle)
        for (int v = 0; v < 3; v++) {
            mesh->normal_buffer[normal_idx++] = (GLfloat)mesh->triangles[i].normal.x;
            mesh->normal_buffer[normal_idx++] = (GLfloat)mesh->triangles[i].normal.y;
            mesh->normal_buffer[normal_idx++] = (GLfloat)mesh->triangles[i].normal.z;
        }
        
        // Store vertices
        for (int v = 0; v < 3; v++) {
            mesh->vertex_buffer[vertex_idx++] = (GLfloat)mesh->triangles[i].vertices[v].x;
            mesh->vertex_buffer[vertex_idx++] = (GLfloat)mesh->triangles[i].vertices[v].y;
            mesh->vertex_buffer[vertex_idx++] = (GLfloat)mesh->triangles[i].vertices[v].z;
        }
    }
    
    printf("Created OpenGL buffers with %d vertices\n", mesh->num_vertices);
}

void stl_mesh_draw(const STLMesh* mesh) {
    if (!mesh || !mesh->vertex_buffer || !mesh->normal_buffer || mesh->num_vertices == 0) return;
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    glVertexPointer(3, GL_FLOAT, 0, mesh->vertex_buffer);
    glNormalPointer(GL_FLOAT, 0, mesh->normal_buffer);
    
    glDrawArrays(GL_TRIANGLES, 0, mesh->num_vertices);
    
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
} 