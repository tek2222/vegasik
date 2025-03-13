#include "math3d.h"
#include <stdio.h>
#include <string.h>

// Vector3 operations
Vector3 vector3_create(double x, double y, double z) {
    Vector3 v = {x, y, z};
    return v;
}

Vector3 vector3_add(Vector3 a, Vector3 b) {
    return vector3_create(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3 vector3_sub(Vector3 a, Vector3 b) {
    return vector3_create(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3 vector3_scale(Vector3 v, double s) {
    return vector3_create(v.x * s, v.y * s, v.z * s);
}

Vector3 vector3_cross(Vector3 a, Vector3 b) {
    return vector3_create(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

double vector3_dot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double vector3_norm(Vector3 v) {
    return sqrt(vector3_dot(v, v));
}

Vector3 vector3_normalize(Vector3 v) {
    double norm = vector3_norm(v);
    if (norm < 1e-10) return vector3_create(0, 0, 0);
    return vector3_scale(v, 1.0 / norm);
}

float vector3_mag(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

// Matrix3 operations
Matrix3 matrix3_identity(void) {
    Matrix3 m;
    memset(m.data, 0, sizeof(m.data));
    m.data[0] = m.data[4] = m.data[8] = 1.0;
    return m;
}

Matrix3 matrix3_rotation_x(double angle) {
    Matrix3 m = matrix3_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[4] = c;
    m.data[5] = -s;
    m.data[7] = s;
    m.data[8] = c;
    return m;
}

Matrix3 matrix3_rotation_y(double angle) {
    Matrix3 m = matrix3_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[0] = c;
    m.data[2] = s;
    m.data[6] = -s;
    m.data[8] = c;
    return m;
}

Matrix3 matrix3_rotation_z(double angle) {
    Matrix3 m = matrix3_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[0] = c;
    m.data[1] = -s;
    m.data[3] = s;
    m.data[4] = c;
    return m;
}

Matrix3 matrix3_from_rpy(Vector3 rpy) {
    Matrix3 rx = matrix3_rotation_x(rpy.x);
    Matrix3 ry = matrix3_rotation_y(rpy.y);
    Matrix3 rz = matrix3_rotation_z(rpy.z);
    return matrix3_multiply(rz, matrix3_multiply(ry, rx));
}

Matrix3 matrix3_from_axis_angle(Vector3 axis, double angle) {
    Matrix3 m;
    Vector3 a = vector3_normalize(axis);
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    
    // Rotation matrix from axis-angle
    // See: https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    m.data[0] = t*a.x*a.x + c;      // xx
    m.data[1] = t*a.x*a.y - s*a.z;  // xy
    m.data[2] = t*a.x*a.z + s*a.y;  // xz
    
    m.data[3] = t*a.x*a.y + s*a.z;  // yx
    m.data[4] = t*a.y*a.y + c;      // yy
    m.data[5] = t*a.y*a.z - s*a.x;  // yz
    
    m.data[6] = t*a.x*a.z - s*a.y;  // zx
    m.data[7] = t*a.y*a.z + s*a.x;  // zy
    m.data[8] = t*a.z*a.z + c;      // zz
    
    return m;
}

Matrix3 matrix3_multiply(Matrix3 a, Matrix3 b) {
    Matrix3 m;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0;
            for (int k = 0; k < 3; k++) {
                sum += a.data[i + k * 3] * b.data[k + j * 3];
            }
            m.data[i + j * 3] = sum;
        }
    }
    return m;
}

// Matrix4 operations
Matrix4 matrix4_identity(void) {
    Matrix4 m;
    memset(m.data, 0, sizeof(m.data));
    m.data[0] = m.data[5] = m.data[10] = m.data[15] = 1.0;
    return m;
}

Matrix4 matrix4_multiply(Matrix4 a, Matrix4 b) {
    Matrix4 m;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double sum = 0;
            for (int k = 0; k < 4; k++) {
                sum += a.data[i + k * 4] * b.data[k + j * 4];
            }
            m.data[i + j * 4] = sum;
        }
    }
    return m;
}

Matrix4 matrix4_from_matrix3_vector3(Matrix3 rot, Vector3 trans) {
    Matrix4 m = matrix4_identity();
    
    // Copy rotation part (3x3)
    m.data[0] = rot.data[0];  // r11
    m.data[1] = rot.data[1];  // r12
    m.data[2] = rot.data[2];  // r13
    
    m.data[4] = rot.data[3];  // r21
    m.data[5] = rot.data[4];  // r22
    m.data[6] = rot.data[5];  // r23
    
    m.data[8] = rot.data[6];  // r31
    m.data[9] = rot.data[7];  // r32
    m.data[10] = rot.data[8]; // r33
    
    // Copy translation part (in last column)
    m.data[12] = trans.x;  // tx (was index 3)
    m.data[13] = trans.y;  // ty (was index 7)
    m.data[14] = trans.z;  // tz (was index 11)
    
    return m;
}

Vector3 matrix4_transform_vector3(Matrix4 m, Vector3 v) {
    double w = m.data[3] * v.x + m.data[7] * v.y + m.data[11] * v.z + m.data[15];
    return vector3_create(
        (m.data[0] * v.x + m.data[4] * v.y + m.data[8] * v.z + m.data[12]) / w,
        (m.data[1] * v.x + m.data[5] * v.y + m.data[9] * v.z + m.data[13]) / w,
        (m.data[2] * v.x + m.data[6] * v.y + m.data[10] * v.z + m.data[14]) / w
    );
}

Matrix4 matrix4_inverse(Matrix4 m) {
    Matrix4 inv;
    double det;
    int i;

    inv.data[0] = m.data[5]  * m.data[10] * m.data[15] - 
                  m.data[5]  * m.data[11] * m.data[14] - 
                  m.data[9]  * m.data[6]  * m.data[15] + 
                  m.data[9]  * m.data[7]  * m.data[14] +
                  m.data[13] * m.data[6]  * m.data[11] - 
                  m.data[13] * m.data[7]  * m.data[10];

    inv.data[4] = -m.data[4]  * m.data[10] * m.data[15] + 
                   m.data[4]  * m.data[11] * m.data[14] + 
                   m.data[8]  * m.data[6]  * m.data[15] - 
                   m.data[8]  * m.data[7]  * m.data[14] - 
                   m.data[12] * m.data[6]  * m.data[11] + 
                   m.data[12] * m.data[7]  * m.data[10];

    inv.data[8] = m.data[4]  * m.data[9] * m.data[15] - 
                  m.data[4]  * m.data[11] * m.data[13] - 
                  m.data[8]  * m.data[5] * m.data[15] + 
                  m.data[8]  * m.data[7] * m.data[13] + 
                  m.data[12] * m.data[5] * m.data[11] - 
                  m.data[12] * m.data[7] * m.data[9];

    inv.data[12] = -m.data[4]  * m.data[9] * m.data[14] + 
                    m.data[4]  * m.data[10] * m.data[13] +
                    m.data[8]  * m.data[5] * m.data[14] - 
                    m.data[8]  * m.data[6] * m.data[13] - 
                    m.data[12] * m.data[5] * m.data[10] + 
                    m.data[12] * m.data[6] * m.data[9];

    inv.data[1] = -m.data[1]  * m.data[10] * m.data[15] + 
                   m.data[1]  * m.data[11] * m.data[14] + 
                   m.data[9]  * m.data[2] * m.data[15] - 
                   m.data[9]  * m.data[3] * m.data[14] - 
                   m.data[13] * m.data[2] * m.data[11] + 
                   m.data[13] * m.data[3] * m.data[10];

    inv.data[5] = m.data[0]  * m.data[10] * m.data[15] - 
                  m.data[0]  * m.data[11] * m.data[14] - 
                  m.data[8]  * m.data[2] * m.data[15] + 
                  m.data[8]  * m.data[3] * m.data[14] + 
                  m.data[12] * m.data[2] * m.data[11] - 
                  m.data[12] * m.data[3] * m.data[10];

    inv.data[9] = -m.data[0]  * m.data[9] * m.data[15] + 
                   m.data[0]  * m.data[11] * m.data[13] + 
                   m.data[8]  * m.data[1] * m.data[15] - 
                   m.data[8]  * m.data[3] * m.data[13] - 
                   m.data[12] * m.data[1] * m.data[11] + 
                   m.data[12] * m.data[3] * m.data[9];

    inv.data[13] = m.data[0]  * m.data[9] * m.data[14] - 
                   m.data[0]  * m.data[10] * m.data[13] - 
                   m.data[8]  * m.data[1] * m.data[14] + 
                   m.data[8]  * m.data[2] * m.data[13] + 
                   m.data[12] * m.data[1] * m.data[10] - 
                   m.data[12] * m.data[2] * m.data[9];

    inv.data[2] = m.data[1]  * m.data[6] * m.data[15] - 
                  m.data[1]  * m.data[7] * m.data[14] - 
                  m.data[5]  * m.data[2] * m.data[15] + 
                  m.data[5]  * m.data[3] * m.data[14] + 
                  m.data[13] * m.data[2] * m.data[7] - 
                  m.data[13] * m.data[3] * m.data[6];

    inv.data[6] = -m.data[0]  * m.data[6] * m.data[15] + 
                   m.data[0]  * m.data[7] * m.data[14] + 
                   m.data[4]  * m.data[2] * m.data[15] - 
                   m.data[4]  * m.data[3] * m.data[14] - 
                   m.data[12] * m.data[2] * m.data[7] + 
                   m.data[12] * m.data[3] * m.data[6];

    inv.data[10] = m.data[0]  * m.data[5] * m.data[15] - 
                   m.data[0]  * m.data[7] * m.data[13] - 
                   m.data[4]  * m.data[1] * m.data[15] + 
                   m.data[4]  * m.data[3] * m.data[13] + 
                   m.data[12] * m.data[1] * m.data[7] - 
                   m.data[12] * m.data[3] * m.data[5];

    inv.data[14] = -m.data[0]  * m.data[5] * m.data[14] + 
                    m.data[0]  * m.data[6] * m.data[13] + 
                    m.data[4]  * m.data[1] * m.data[14] - 
                    m.data[4]  * m.data[2] * m.data[13] - 
                    m.data[12] * m.data[1] * m.data[6] + 
                    m.data[12] * m.data[2] * m.data[5];

    inv.data[3] = -m.data[1] * m.data[6] * m.data[11] + 
                   m.data[1] * m.data[7] * m.data[10] + 
                   m.data[5] * m.data[2] * m.data[11] - 
                   m.data[5] * m.data[3] * m.data[10] - 
                   m.data[9] * m.data[2] * m.data[7] + 
                   m.data[9] * m.data[3] * m.data[6];

    inv.data[7] = m.data[0] * m.data[6] * m.data[11] - 
                  m.data[0] * m.data[7] * m.data[10] - 
                  m.data[4] * m.data[2] * m.data[11] + 
                  m.data[4] * m.data[3] * m.data[10] + 
                  m.data[8] * m.data[2] * m.data[7] - 
                  m.data[8] * m.data[3] * m.data[6];

    inv.data[11] = -m.data[0] * m.data[5] * m.data[11] + 
                    m.data[0] * m.data[7] * m.data[9] + 
                    m.data[4] * m.data[1] * m.data[11] - 
                    m.data[4] * m.data[3] * m.data[9] - 
                    m.data[8] * m.data[1] * m.data[7] + 
                    m.data[8] * m.data[3] * m.data[5];

    inv.data[15] = m.data[0] * m.data[5] * m.data[10] - 
                   m.data[0] * m.data[6] * m.data[9] - 
                   m.data[4] * m.data[1] * m.data[10] + 
                   m.data[4] * m.data[2] * m.data[9] + 
                   m.data[8] * m.data[1] * m.data[6] - 
                   m.data[8] * m.data[2] * m.data[5];

    det = m.data[0] * inv.data[0] + m.data[1] * inv.data[4] + 
          m.data[2] * inv.data[8] + m.data[3] * inv.data[12];

    if (det == 0)
        return matrix4_identity();

    det = 1.0 / det;

    for (i = 0; i < 16; i++)
        inv.data[i] = inv.data[i] * det;

    return inv;
}

// Utility functions
void matrix4_print(Matrix4 m) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%8.3f ", m.data[i + j * 4]);
        }
        printf("\n");
    }
}

void matrix3_print(Matrix3 m) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            printf("%8.3f ", m.data[i + j * 3]);
        }
        printf("\n");
    }
}

void vector3_print(Vector3 v) {
    printf("(%8.3f, %8.3f, %8.3f)\n", v.x, v.y, v.z);
}

Matrix4 matrix4_translation(double x, double y, double z) {
    Matrix4 m = matrix4_identity();
    m.data[12] = x;   // Translation X (was index 3)
    m.data[13] = y;   // Translation Y (was index 7)
    m.data[14] = z;   // Translation Z (was index 11)
    return m;
}

Matrix3 matrix3_transpose(Matrix3 m) {
    Matrix3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i * 3 + j] = m.data[j * 3 + i];
        }
    }
    return result;
}

Vector3 matrix3_transform_vector3(Matrix3 m, Vector3 v) {
    return vector3_create(
        m.data[0] * v.x + m.data[3] * v.y + m.data[6] * v.z,
        m.data[1] * v.x + m.data[4] * v.y + m.data[7] * v.z,
        m.data[2] * v.x + m.data[5] * v.y + m.data[8] * v.z
    );
}

Matrix4 matrix4_rotation_x(double angle) {
    Matrix4 m = matrix4_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[5] = c;
    m.data[6] = -s;
    m.data[9] = s;
    m.data[10] = c;
    return m;
}

Matrix4 matrix4_rotation_y(double angle) {
    Matrix4 m = matrix4_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[0] = c;
    m.data[2] = s;
    m.data[8] = -s;
    m.data[10] = c;
    return m;
}

Matrix4 matrix4_rotation_z(double angle) {
    Matrix4 m = matrix4_identity();
    double c = cos(angle);
    double s = sin(angle);
    m.data[0] = c;
    m.data[1] = -s;
    m.data[4] = s;
    m.data[5] = c;
    return m;
}

Matrix4 matrix4_from_axis_angle(Vector3 axis, double angle) {
    Matrix4 m = matrix4_identity();
    Vector3 a = vector3_normalize(axis);
    double c = cos(angle);
    double s = sin(angle);
    double t = 1.0 - c;
    
    m.data[0] = t*a.x*a.x + c;      // xx
    m.data[1] = t*a.x*a.y - s*a.z;  // xy
    m.data[2] = t*a.x*a.z + s*a.y;  // xz
    
    m.data[4] = t*a.x*a.y + s*a.z;  // yx
    m.data[5] = t*a.y*a.y + c;      // yy
    m.data[6] = t*a.y*a.z - s*a.x;  // yz
    
    m.data[8] = t*a.x*a.z - s*a.y;  // zx
    m.data[9] = t*a.y*a.z + s*a.x;  // zy
    m.data[10] = t*a.z*a.z + c;     // zz
    
    return m;
} 