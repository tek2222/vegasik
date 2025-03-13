#ifndef MATH3D_H
#define MATH3D_H

#include <math.h>

// Vector3 structure
typedef struct {
    double x;
    double y;
    double z;
} Vector3;

// Matrix4 structure (4x4 matrix for transformations)
typedef struct {
    double data[16];  // Column-major order like OpenGL
} Matrix4;

// Matrix3 structure (3x3 matrix for rotations)
typedef struct {
    double data[9];  // Column-major order
} Matrix3;

// Vector3 operations
Vector3 vector3_create(double x, double y, double z);
Vector3 vector3_add(Vector3 a, Vector3 b);
Vector3 vector3_sub(Vector3 a, Vector3 b);
Vector3 vector3_scale(Vector3 v, double s);
Vector3 vector3_cross(Vector3 a, Vector3 b);
double vector3_dot(Vector3 a, Vector3 b);
double vector3_norm(Vector3 v);
Vector3 vector3_normalize(Vector3 v);
float vector3_mag(const float v[3]);

// Matrix operations
Matrix4 matrix4_identity(void);
Matrix4 matrix4_multiply(Matrix4 a, Matrix4 b);
Matrix4 matrix4_from_matrix3_vector3(Matrix3 rot, Vector3 trans);
Vector3 matrix4_transform_vector3(Matrix4 m, Vector3 v);
Matrix4 matrix4_inverse(Matrix4 m);
Matrix4 matrix4_translation(double x, double y, double z);

// Rotation operations
Matrix3 matrix3_identity(void);
Matrix3 matrix3_rotation_x(double angle);
Matrix3 matrix3_rotation_y(double angle);
Matrix3 matrix3_rotation_z(double angle);
Matrix3 matrix3_from_rpy(Vector3 rpy);  // Roll-Pitch-Yaw to rotation matrix
Matrix3 matrix3_from_axis_angle(Vector3 axis, double angle);
Matrix3 matrix3_multiply(Matrix3 a, Matrix3 b);
Matrix3 matrix3_transpose(Matrix3 m);

// Utility functions
void matrix4_print(Matrix4 m);
void matrix3_print(Matrix3 m);
void vector3_print(Vector3 v);

// Add to header
Vector3 matrix3_transform_vector3(Matrix3 m, Vector3 v);

// Add these declarations to math3d.h
Matrix4 matrix4_rotation_x(double angle);
Matrix4 matrix4_rotation_y(double angle);
Matrix4 matrix4_rotation_z(double angle);
Matrix4 matrix4_from_axis_angle(Vector3 axis, double angle);

#endif // MATH3D_H 