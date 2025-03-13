#include "draw_dae.h"
#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <stdlib.h>
#include <string.h>

DAEMesh* dae_mesh_create(void) {
    DAEMesh* mesh = (DAEMesh*)calloc(1, sizeof(DAEMesh));
    if (mesh) {
        // Initialize default material properties
        mesh->ambient[0] = mesh->ambient[1] = mesh->ambient[2] = 0.7f;
        mesh->ambient[3] = 1.0f;
        
        mesh->diffuse[0] = mesh->diffuse[1] = mesh->diffuse[2] = 0.8f;
        mesh->diffuse[3] = 1.0f;
        
        mesh->specular[0] = mesh->specular[1] = mesh->specular[2] = 0.2f;
        mesh->specular[3] = 1.0f;
        
        mesh->shininess = 50.0f;
    }
    return mesh;
}

void dae_mesh_destroy(DAEMesh* mesh) {
    if (mesh) {
        free(mesh->vertices);
        free(mesh->normals);
        free(mesh->texcoords);
        free(mesh->indices);
        if (mesh->has_texture) {
            glDeleteTextures(1, &mesh->texture_id);
        }
        free(mesh);
    }
}

int dae_mesh_load(DAEMesh* mesh, const char* filename) {
    if (!mesh) {
        fprintf(stderr, "DAE Load Error: mesh pointer is NULL\n");
        return 0;
    }
    if (!filename) {
        fprintf(stderr, "DAE Load Error: filename is NULL\n");
        return 0;
    }

    printf("Attempting to load DAE file: %s\n", filename);

    // Try different combinations of import flags
    unsigned int flags = aiProcess_Triangulate | 
                        aiProcess_GenNormals | 
                        aiProcess_JoinIdenticalVertices |
                        aiProcess_ValidateDataStructure |  // Add validation
                        aiProcess_FixInfacingNormals |    // Fix normal directions
                        aiProcess_FindInvalidData |       // Find invalid data
                        aiProcess_GenUVCoords |          // Generate texture coordinates if missing
                        aiProcess_OptimizeMeshes;        // Optimize the mesh data

    const struct aiScene* scene = aiImportFile(filename, flags);

    if (!scene) {
        fprintf(stderr, "DAE Load Error: %s\n", aiGetErrorString());
        
        // Try again with minimal processing
        flags = aiProcess_Triangulate | aiProcess_GenNormals;
        printf("Retrying with minimal processing...\n");
        scene = aiImportFile(filename, flags);
        
        if (!scene) {
            fprintf(stderr, "DAE Load Error (minimal processing): %s\n", aiGetErrorString());
            
            // Try one last time with no processing
            printf("Retrying with no processing...\n");
            scene = aiImportFile(filename, 0);
            
            if (!scene) {
                fprintf(stderr, "DAE Load Error (no processing): %s\n", aiGetErrorString());
                return 0;
            }
        }
    }

    printf("Successfully loaded scene:\n");
    printf("- Number of meshes: %d\n", scene->mNumMeshes);
    printf("- Number of materials: %d\n", scene->mNumMaterials);
    printf("- Number of textures: %d\n", scene->mNumTextures);
    printf("- Number of animations: %d\n", scene->mNumAnimations);

    // For simplicity, we'll just load the first mesh
    if (scene->mNumMeshes < 1) {
        fprintf(stderr, "DAE Load Error: No meshes found in file\n");
        aiReleaseImport(scene);
        return 0;
    }

    struct aiMesh* ai_mesh = scene->mMeshes[0];
    printf("Processing mesh:\n");
    printf("- Vertices: %d\n", ai_mesh->mNumVertices);
    printf("- Faces: %d\n", ai_mesh->mNumFaces);
    printf("- Has normals: %s\n", ai_mesh->mNormals ? "yes" : "no");
    printf("- Number of UV channels: %d\n", AI_MAX_NUMBER_OF_TEXTURECOORDS);
    for (unsigned int i = 0; i < AI_MAX_NUMBER_OF_TEXTURECOORDS; i++) {
        if (ai_mesh->mTextureCoords[i]) {
            printf("  - UV channel %d: present\n", i);
        }
    }

    // Allocate memory for mesh data
    mesh->num_vertices = ai_mesh->mNumVertices;
    mesh->vertices = (float*)malloc(mesh->num_vertices * 3 * sizeof(float));
    if (!mesh->vertices) {
        fprintf(stderr, "DAE Load Error: Failed to allocate vertex memory\n");
        aiReleaseImport(scene);
        return 0;
    }

    mesh->normals = (float*)malloc(mesh->num_vertices * 3 * sizeof(float));
    if (!mesh->normals) {
        fprintf(stderr, "DAE Load Error: Failed to allocate normal memory\n");
        free(mesh->vertices);
        aiReleaseImport(scene);
        return 0;
    }
    
    if (ai_mesh->mTextureCoords[0]) {
        printf("Mesh has texture coordinates\n");
        mesh->texcoords = (float*)malloc(mesh->num_vertices * 2 * sizeof(float));
        if (!mesh->texcoords) {
            fprintf(stderr, "DAE Load Error: Failed to allocate texture coordinate memory\n");
            free(mesh->vertices);
            free(mesh->normals);
            aiReleaseImport(scene);
            return 0;
        }
    } else {
        printf("Mesh has no texture coordinates\n");
    }

    // Copy vertex data with scaling
    const float scale = 0.001f;
    printf("Applying scale factor: %f\n", scale);
    
    for (unsigned int i = 0; i < ai_mesh->mNumVertices; i++) {
        mesh->vertices[i*3] = ai_mesh->mVertices[i].x * scale;
        mesh->vertices[i*3+1] = ai_mesh->mVertices[i].y * scale;
        mesh->vertices[i*3+2] = ai_mesh->mVertices[i].z * scale;

        mesh->normals[i*3] = ai_mesh->mNormals[i].x;
        mesh->normals[i*3+1] = ai_mesh->mNormals[i].y;
        mesh->normals[i*3+2] = ai_mesh->mNormals[i].z;

        if (ai_mesh->mTextureCoords[0]) {
            mesh->texcoords[i*2] = ai_mesh->mTextureCoords[0][i].x;
            mesh->texcoords[i*2+1] = ai_mesh->mTextureCoords[0][i].y;
        }
    }

    // Copy face indices
    mesh->num_indices = ai_mesh->mNumFaces * 3;
    mesh->indices = (unsigned int*)malloc(mesh->num_indices * sizeof(unsigned int));
    if (!mesh->indices) {
        fprintf(stderr, "DAE Load Error: Failed to allocate index memory\n");
        free(mesh->vertices);
        free(mesh->normals);
        free(mesh->texcoords);
        aiReleaseImport(scene);
        return 0;
    }
    
    printf("Processing %d triangular faces\n", ai_mesh->mNumFaces);
    for (unsigned int i = 0; i < ai_mesh->mNumFaces; i++) {
        struct aiFace face = ai_mesh->mFaces[i];
        if (face.mNumIndices != 3) {
            fprintf(stderr, "Warning: Face %d has %d indices (expected 3)\n", 
                    i, face.mNumIndices);
            continue;
        }
        mesh->indices[i*3] = face.mIndices[0];
        mesh->indices[i*3+1] = face.mIndices[1];
        mesh->indices[i*3+2] = face.mIndices[2];
    }

    printf("Successfully loaded mesh data:\n");
    printf("- Vertices: %d\n", mesh->num_vertices);
    printf("- Indices: %d\n", mesh->num_indices);
    printf("- Has texcoords: %s\n", mesh->texcoords ? "yes" : "no");

    aiReleaseImport(scene);
    return 1;
}

void dae_mesh_draw(const DAEMesh* mesh) {
    if (!mesh) {
        fprintf(stderr, "Draw Error: mesh pointer is NULL\n");
        return;
    }

    if (!mesh->vertices || !mesh->normals || !mesh->indices) {
        fprintf(stderr, "Draw Error: mesh data is incomplete\n");
        fprintf(stderr, "  vertices: %p\n", (void*)mesh->vertices);
        fprintf(stderr, "  normals: %p\n", (void*)mesh->normals);
        fprintf(stderr, "  indices: %p\n", (void*)mesh->indices);
        return;
    }

    printf("Drawing mesh with %d vertices and %d indices\n", 
           mesh->num_vertices, mesh->num_indices);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    
    if (mesh->texcoords && mesh->has_texture) {
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glBindTexture(GL_TEXTURE_2D, mesh->texture_id);
        glTexCoordPointer(2, GL_FLOAT, 0, mesh->texcoords);
    }

    glVertexPointer(3, GL_FLOAT, 0, mesh->vertices);
    glNormalPointer(GL_FLOAT, 0, mesh->normals);
    
    GLenum error;
    if ((error = glGetError()) != GL_NO_ERROR) {
        fprintf(stderr, "OpenGL error before draw: %d\n", error);
    }
    
    glDrawElements(GL_TRIANGLES, mesh->num_indices, GL_UNSIGNED_INT, mesh->indices);
    
    if ((error = glGetError()) != GL_NO_ERROR) {
        fprintf(stderr, "OpenGL error after draw: %d\n", error);
    }

    if (mesh->texcoords && mesh->has_texture) {
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    }
    
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
} 