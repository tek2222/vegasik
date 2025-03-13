#ifndef DRAW_DAE_H
#define DRAW_DAE_H

#include <GL/gl.h>
#include <stdio.h>  // Add this for printf/fprintf functions

typedef struct {
    // Mesh data
    float* vertices;
    float* normals;
    float* texcoords;
    unsigned int* indices;
    int num_vertices;
    int num_indices;
    
    // Texture data
    GLuint texture_id;
    int has_texture;
    
    // Material properties
    float ambient[4];
    float diffuse[4];
    float specular[4];
    float shininess;
} DAEMesh;

// Function declarations
DAEMesh* dae_mesh_create(void);
void dae_mesh_destroy(DAEMesh* mesh);
int dae_mesh_load(DAEMesh* mesh, const char* filename);
void dae_mesh_draw(const DAEMesh* mesh);

#endif // DRAW_DAE_H 