#ifndef DRAW_STL_H
#define DRAW_STL_H

#include "math3d.h"
#include <GL/gl.h>

// Forward declarations
typedef struct STLMesh STLMesh;
typedef struct Triangle Triangle;

// Structures
struct Triangle {
    Vector3 normal;
    Vector3 vertices[3];
};

struct STLMesh {
    Triangle* triangles;
    GLfloat* vertex_buffer;
    GLfloat* normal_buffer;
    int num_vertices;
};

// Function declarations
STLMesh* stl_mesh_create(void);
void stl_mesh_destroy(STLMesh* mesh);
int stl_mesh_load(STLMesh* mesh, const char* filename);
void stl_mesh_compute_buffers(STLMesh* mesh);
void stl_mesh_draw(const STLMesh* mesh);

#endif // DRAW_STL_H 