#include "canvas.h"
#include "math3d.h"  // Add this include for vector operations
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <math.h>
#include <time.h>

typedef struct {
    int mouse_button_pressed;
    double last_mouse_x;
    double last_mouse_y;
    // IMU state
    Matrix4 pose;            // Full 4x4 transform matrix for IMU pose
    float accel[3];         // x, y, z acceleration in m/s^2
    float gyro[3];         // x, y, z angular velocity in rad/s
    // Rotation control
    float angular_vel[3];  // Rotation speed for roll, pitch, yaw
    // Sensor biases
    float accel_bias[3];   // accelerometer bias in m/s^2
    float gyro_bias[3];    // gyroscope bias in rad/s
    // Integrated IMU state
    Matrix4 integrated_pose;  // Integrated transform from gyro readings
    // Motion state
    int motion_mode;        // 0 for rotation, 1 for linear motion
    Vector3 position;       // Position in world frame
    Vector3 velocity;       // Velocity in world frame
    Vector3 acceleration;   // Acceleration in world frame
    Matrix4 simple_pose;  // Pose for the simple fusion result
} IMUData;

static float rand_gaussian(void) {
    // Box-Muller transform for Gaussian noise
    float u1 = (float)rand() / RAND_MAX;
    float u2 = (float)rand() / RAND_MAX;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}

static void draw_imu_box(void) {
    glPushMatrix();
    
    // Draw a 20x20x10 cm box (0.2x0.2x0.1 meters)
    glColor3f(0.7f, 0.7f, 0.7f);  // Gray color
    glScalef(0.2f, 0.2f, 0.1f);   // 20cm x 10cm x 20cm
    glutSolidCube(1.0f);
    
    // Draw axes on the IMU
    glBegin(GL_LINES);
    // X axis - red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.6f, 0.0f, 0.0f);
    // Y axis - green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.6f, 0.0f);
    // Z axis - blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.6f);
    glEnd();
    
    glPopMatrix();
}

static Matrix4 calculate_alignment_rotation(Vector3 current_z, Vector3 target, float blend_factor) {
    // Get current Z axis
    Vector3 z_axis = current_z;
    // Normalize target vector (accelerometer reading)
    Vector3 target_norm = vector3_normalize(target);
    
    // Calculate rotation axis and angle
    Vector3 rotation_axis = vector3_cross(z_axis, target_norm);
    float cos_angle = vector3_dot(z_axis, target_norm);
    float angle = acosf(cos_angle) * blend_factor;  // Blend factor for smooth transition
    
    if (vector3_norm(rotation_axis) < 0.0001f) {
        return matrix4_identity();  // No rotation needed
    }
    
    return matrix4_from_axis_angle(vector3_normalize(rotation_axis), angle);
}

static void update_imu(IMUData* data) {
    const float dt = 1.0f/60.0f;
    const float g = 9.81f;
    
    // Friction coefficients
    const float LINEAR_FRICTION = 5.0f;
    const float MIN_VELOCITY = 0.001f;
    
    // Noise parameters
    const float GYRO_NOISE = 0.01f;   // rad/s
    const float ACCEL_NOISE = 0.02f;  // m/s²
    
    if (data->motion_mode) {
        // Motion mode - handle linear motion
        data->acceleration = vector3_create(
            data->angular_vel[0],
            data->angular_vel[1],
            data->angular_vel[2]
        );
        data->acceleration = vector3_scale(data->acceleration, 2.0);
        
        Vector3 friction = vector3_scale(data->velocity, -LINEAR_FRICTION);
        data->acceleration = vector3_add(data->acceleration, friction);
        
        data->velocity = vector3_add(data->velocity, 
                                   vector3_scale(data->acceleration, dt));
        
        float speed = vector3_norm(data->velocity);
        if (speed < MIN_VELOCITY) {
            data->velocity = vector3_create(0.0, 0.0, 0.0);
        }
        
        data->position = vector3_add(data->position, 
                                   vector3_scale(data->velocity, dt));
        
        // Create translation matrix
        Matrix4 pos_matrix = matrix4_translation(
            data->position.x,
            data->position.y,
            data->position.z
        );
        
        // Get current rotation matrix
        Matrix4 rot_only = data->pose;
        rot_only.data[12] = 0.0;  // Zero out translation components
        rot_only.data[13] = 0.0;
        rot_only.data[14] = 0.0;
        
        // Combine translation with existing rotation
        data->pose = matrix4_multiply(pos_matrix, rot_only);
        
        // In motion mode, gyro only reads bias + noise (no rotation)
        for(int i = 0; i < 3; i++) {
            data->gyro[i] = data->gyro_bias[i] + GYRO_NOISE * rand_gaussian();
        }
    } else {
        // Rotation mode - handle rotational motion
        Matrix4 delta_rot = matrix4_identity();
        for(int i = 0; i < 3; i++) {
            if(data->angular_vel[i] != 0.0f) {
                Matrix4 rot;
                float angle = data->angular_vel[i] * dt;
                if(i == 0) rot = matrix4_rotation_x(angle);
                else if(i == 1) rot = matrix4_rotation_y(angle);
                else rot = matrix4_rotation_z(angle);
                delta_rot = matrix4_multiply(delta_rot, rot);
            }
        }
        data->pose = matrix4_multiply(data->pose, delta_rot);
        
        // In rotation mode, gyro reads angular velocity + bias + noise
        for(int i = 0; i < 3; i++) {
            data->gyro[i] = data->angular_vel[i] + data->gyro_bias[i] + 
                           GYRO_NOISE * rand_gaussian();
        }
    }
    
    // Always integrate gyro readings regardless of mode
    Matrix4 delta_gyro = matrix4_identity();
    Vector3 gyro_vec = vector3_create(data->gyro[0], data->gyro[1], data->gyro[2]);
    float gyro_mag = vector3_norm(gyro_vec);
    if(gyro_mag > 0.0001f) {
        gyro_vec = vector3_scale(gyro_vec, dt/gyro_mag);
        delta_gyro = matrix4_from_axis_angle(gyro_vec, gyro_mag * dt);
    }
    data->integrated_pose = matrix4_multiply(data->integrated_pose, delta_gyro);
    
    // Calculate accelerometer readings in IMU frame
    Vector3 grav_world = vector3_create(0.0, 0.0, 1.0);  // Unit vector pointing up in Z
    grav_world = vector3_scale(grav_world, g);  // Scale by g to get acceleration
    
    // Get only the rotation part of the pose for transforming to IMU frame
    Matrix4 rot_only = data->pose;
    rot_only.data[12] = 0.0;  // Zero out translation components
    rot_only.data[13] = 0.0;
    rot_only.data[14] = 0.0;
    Matrix4 inv_rot = matrix4_inverse(rot_only);
    
    // Transform gravity and acceleration to IMU's local frame using only rotation
    Vector3 grav_local = matrix4_transform_vector3(inv_rot, grav_world);
    Vector3 accel_local = matrix4_transform_vector3(inv_rot, data->acceleration);
    
    // Add noise and bias to accelerometer readings in IMU's local frame
    for(int i = 0; i < 3; i++) {
        float components[3] = {
            grav_local.x + accel_local.x,
            grav_local.y + accel_local.y,
            grav_local.z + accel_local.z
        };
        data->accel[i] = components[i] + data->accel_bias[i] + 
                        ACCEL_NOISE * rand_gaussian();
    }

    // Integrate gyro readings into IMU_SIMPLE's pose
    Matrix4 simple_delta_gyro = matrix4_identity();
    Vector3 simple_gyro_vec = vector3_create(data->gyro[0], data->gyro[1], data->gyro[2]);
    float simple_gyro_mag = vector3_norm(simple_gyro_vec);
    if(simple_gyro_mag > 0.0001f) {
        simple_gyro_vec = vector3_scale(simple_gyro_vec, dt/simple_gyro_mag);
        simple_delta_gyro = matrix4_from_axis_angle(simple_gyro_vec, simple_gyro_mag * dt);
    }
    data->simple_pose = matrix4_multiply(data->simple_pose, simple_delta_gyro);
}

// Add this helper function to draw an arrow/vector
static void draw_vector(float x, float y, float z) {
    glBegin(GL_LINES);
    // Draw main line
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(x, y, z);
    // Draw arrow head (simple version)
    float head_size = 0.05f;
    glVertex3f(x, y, z);
    glVertex3f(x - head_size, y, z);
    glVertex3f(x, y, z);
    glVertex3f(x, y - head_size, z);
    glVertex3f(x, y, z);
    glVertex3f(x, y, z - head_size);
    glEnd();
}

static void draw_callback(Canvas* canvas, void* user_data) {
    IMUData* data = (IMUData*)user_data;
    
    // Update IMU readings
    update_imu(data);
    
    // Calculate gravity vector in world coordinates (now points down in Z)
    float grav_world[3] = {0.0f, 0.0f, -9.81f};
    
    // Calculate magnitude of measured acceleration using math3d
    float acc_mag = vector3_mag(data->accel);
    
    // Print IMU data, gravity vector, and acceleration magnitude
    printf("\rAcc: %+09.4f %+09.4f %+09.4f m/s², |Acc|: %08.4f m/s², Gyro: %+09.4f %+09.4f %+09.4f rad/s, Grav: %+09.4f %+09.4f %+09.4f m/s²", 
           data->accel[0], data->accel[1], data->accel[2],
           acc_mag,
           data->gyro[0], data->gyro[1], data->gyro[2],
           grav_world[0], grav_world[1], grav_world[2]);
    fflush(stdout);
    
    // Draw grid and coordinate frame
    canvas_draw_grid(5.0f, 1.0f, 0.1f);
    canvas_draw_axis_cross();
    
    // Draw coordinate cross at (1,0,0) with integrated rotation
    glPushMatrix();
    Matrix4 sensor_transform = matrix4_translation(1.0, 0.0, 0.0);
    sensor_transform = matrix4_multiply(sensor_transform, data->integrated_pose);
    double* sensor_matrix = sensor_transform.data;
    glMultMatrixd(sensor_matrix);
    canvas_draw_axis_cross();
    glColor3f(1.0f, 1.0f, 1.0f);
    canvas_draw_text_3d(0.0f, 0.3f, 0.0f, "IMU_GYR");
    glPopMatrix();
    
    // Draw accelerometer vector at (2,0,0)
    glPushMatrix();
    Matrix4 acc_transform = matrix4_translation(2.0, 0.0, 0.0);
    double* acc_matrix = acc_transform.data;
    glMultMatrixd(acc_matrix);
    
    // Draw coordinate frame
    canvas_draw_axis_cross();
    
    // Draw acceleration vector (scaled by 0.1)
    glColor3f(1.0f, 1.0f, 0.0f);  // Yellow for acceleration vector
    draw_vector(
        data->accel[0] * 0.1f,
        data->accel[1] * 0.1f,
        data->accel[2] * 0.1f
    );
    
    // Draw label
    glColor3f(1.0f, 1.0f, 1.0f);
    canvas_draw_text_3d(0.0f, 0.3f, 0.0f, "IMU_ACC");
    
    glPopMatrix();
    
    // Draw IMU with current orientation and position
    glPushMatrix();
    // Use the pose matrix directly - it already contains our position and rotation
    double* imu_matrix = data->pose.data;
    glMultMatrixd(imu_matrix);
    draw_imu_box();
    glPopMatrix();
    
    // Draw IMU_SIMPLE at (-1,0,0)
    glPushMatrix();
    Matrix4 simple_transform = matrix4_translation(-1.0, 0.0, 0.0);
    simple_transform = matrix4_multiply(simple_transform, data->simple_pose);
    double* simple_matrix = simple_transform.data;
    glMultMatrixd(simple_matrix);
    
    // Draw coordinate frame
    canvas_draw_axis_cross();
    
    // Draw accelerometer vector in yellow (in IMU_SIMPLE's frame)
    glColor3f(1.0f, 1.0f, 0.0f);   // Yellow for acceleration
    draw_vector(
        data->accel[0] * 0.1f,  // Scale by 0.1 like IMU_ACC
        data->accel[1] * 0.1f,
        data->accel[2] * 0.1f
    );
    
    glPopMatrix();
    
    // Draw vertical up arrow at IMU_SIMPLE location
    glPushMatrix();
    glTranslatef(-1.0, 0.0, 0.0);  // Move to IMU_SIMPLE location
    glColor3f(1.0f, 0.0f, 1.0f);   // Purple color
    
    // Draw vertical line with arrow head
    glBegin(GL_LINES);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.5f);
    // ... arrow head code ...
    glEnd();
    
    // Calculate rotation axis
    // First get accelerometer vector in world space
    Vector3 acc_local = vector3_create(data->accel[0], data->accel[1], data->accel[2]);
    Vector3 acc_world = matrix4_transform_vector3(data->simple_pose, acc_local);
    acc_world = vector3_normalize(acc_world);
    
    // Up vector is just (0,0,1)
    Vector3 up = vector3_create(0.0, 0.0, 1.0);
    
    // Calculate rotation axis (swapped order here too)
    Vector3 rot_axis = vector3_cross(up, acc_world);  // Changed from (acc_world, up)
    if (vector3_norm(rot_axis) > 0.0001f) {
        rot_axis = vector3_normalize(rot_axis);
        
        // Draw rotation axis in yellow
        glColor3f(1.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(
            rot_axis.x * 1.5f,
            rot_axis.y * 1.5f,
            rot_axis.z * 1.5f
        );
        glEnd();
    }
    
    glPopMatrix();
    
    // Draw IMU data overlay
    canvas_begin_2d_overlay(canvas);
    
    int y = 20;
    canvas_draw_text_at(10, y, "=== IMU DATA ==="); y += 15;
    char buf[128];
    snprintf(buf, sizeof(buf), "Accel: X: %+09.4f Y: %+09.4f Z: %+09.4f m/s^2", 
             data->accel[0], data->accel[1], data->accel[2]);
    canvas_draw_text_at(10, y, buf); y += 15;
    snprintf(buf, sizeof(buf), "Gyro:  X: %+09.4f Y: %+09.4f Z: %+09.4f rad/s", 
             data->gyro[0], data->gyro[1], data->gyro[2]);
    canvas_draw_text_at(10, y, buf); y += 15;
    
    y += 10;
    canvas_draw_text_at(10, y, "=== CONTROLS ==="); y += 15;
    canvas_draw_text_at(10, y, "Left Mouse - Orbit camera"); y += 15;
    canvas_draw_text_at(10, y, "Right Mouse - Pan camera"); y += 15;
    canvas_draw_text_at(10, y, "Mouse Wheel - Zoom camera"); y += 15;
    canvas_draw_text_at(10, y, "ESC - Exit viewer"); y += 15;
    
    canvas_end_2d_overlay();
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data) {
    IMUData* data = (IMUData*)user_data;
    const float ROT_SPEED = 1.0f;     // radians per second
    const float MOVE_SPEED = 2.0f;    // m/s²
    const float FUSION_STEP = 0.5f;   // Half the angle for smooth correction
    
    // Reset control inputs
    data->angular_vel[0] = 0.0f;
    data->angular_vel[1] = 0.0f;
    data->angular_vel[2] = 0.0f;
    
    switch(key) {
        case XK_i:  // Perform one step of IMU fusion
            {
                // Get accelerometer vector in world space
                Vector3 acc_local = vector3_create(data->accel[0], data->accel[1], data->accel[2]);
                Vector3 acc_world = matrix4_transform_vector3(data->simple_pose, acc_local);
                acc_world = vector3_normalize(acc_world);
                
                // Up vector is just (0,0,1)
                Vector3 up = vector3_create(0.0, 0.0, 1.0);
                
                // Calculate rotation axis (swapped order of cross product)
                Vector3 rot_axis = vector3_cross(up, acc_world);  // Changed from (acc_world, up)
                if (vector3_norm(rot_axis) > 0.0001f) {
                    rot_axis = vector3_normalize(rot_axis);
                    
                    // Calculate angle between vectors
                    float cos_angle = vector3_dot(acc_world, up);
                    float angle = acosf(cos_angle) * FUSION_STEP;  // Take half the angle
                    
                    // Create and apply rotation
                    Matrix4 correction = matrix4_from_axis_angle(rot_axis, angle);
                    data->simple_pose = matrix4_multiply(data->simple_pose, correction);
                }
            }
            break;
            
        case XK_r:  // Reset everything
            data->pose = matrix4_identity();
            data->integrated_pose = matrix4_identity();
            data->position = vector3_create(0.0, 0.0, 0.0);
            data->velocity = vector3_create(0.0, 0.0, 0.0);
            data->acceleration = vector3_create(0.0, 0.0, 0.0);
            printf("\nReset IMU position and orientation\n");
            break;
            
        case XK_m:  // Toggle motion mode
            data->motion_mode = !data->motion_mode;
            printf("\nSwitched to %s mode\n", data->motion_mode ? "motion" : "rotation");
            break;
            
        case XK_Left:
            data->angular_vel[0] = data->motion_mode ? -MOVE_SPEED : ROT_SPEED;  // X axis
            break;
        case XK_Right:
            data->angular_vel[0] = data->motion_mode ? MOVE_SPEED : -ROT_SPEED;  // X axis
            break;
        case XK_Up:
            data->angular_vel[1] = data->motion_mode ? MOVE_SPEED : ROT_SPEED;   // Y axis
            break;
        case XK_Down:
            data->angular_vel[1] = data->motion_mode ? -MOVE_SPEED : -ROT_SPEED; // Y axis
            break;
        case XK_q:
            data->angular_vel[2] = data->motion_mode ? MOVE_SPEED : ROT_SPEED;   // Z axis
            break;
        case XK_e:
            data->angular_vel[2] = data->motion_mode ? -MOVE_SPEED : -ROT_SPEED; // Z axis
            break;
        case XK_space:  // Already zeroed above
            break;
        case XK_Escape:
            canvas->should_exit = 1;
            break;
    }
}

static void mouse_button_callback(Canvas* canvas, 
                                int button, int state, int x, int y, 
                                void* user_data __attribute__((unused))) {
    canvas_handle_mouse_button(canvas, button, state, x, y);
}

static void mouse_motion_callback(Canvas* canvas, 
                                int x, int y, 
                                void* user_data __attribute__((unused))) {
    canvas_handle_mouse_motion(canvas, x, y);
}

int main(int argc, char* argv[]) {
    // Initialize random number generator
    srand(time(NULL));
    
    // Initialize GLUT
    glutInit(&argc, argv);
    
    // Create viewer data with biases
    IMUData data = {
        .mouse_button_pressed = 0,
        .last_mouse_x = 0,
        .last_mouse_y = 0,
        .pose = matrix4_identity(),
        .accel = {0.0f, -9.81f, 0.0f},
        .gyro = {0.0f, 0.0f, 0.0f},
        .angular_vel = {0.0f, 0.0f, 0.0f},
        // Small random biases
        .accel_bias = {0.05f, -0.03f, 0.04f},  // ~0.05 m/s² bias
        .gyro_bias = {0.008f, -0.016f, 0.008f}, // ~0.008 rad/s bias
        .integrated_pose = matrix4_identity(),
        .motion_mode = 0,
        .position = vector3_create(0.0, 0.0, 0.0),
        .velocity = vector3_create(0.0, 0.0, 0.0),
        .acceleration = vector3_create(0.0, 0.0, 0.0),
        .simple_pose = matrix4_rotation_x(-30.0 * M_PI / 180.0),  // Start at -30 degrees around X
    };
    
    // Create and setup canvas
    Canvas* canvas = canvas_create("IMU Simulator", 1000, 600);
    if (!canvas) {
        fprintf(stderr, "Failed to create canvas\n");
        return 1;
    }
    
    canvas_set_draw_callback(canvas, draw_callback, &data);
    canvas_set_key_callback(canvas, key_callback, &data);
    canvas_set_mouse_button_callback(canvas, mouse_button_callback, &data);
    canvas_set_mouse_motion_callback(canvas, mouse_motion_callback, &data);
    
    // Set default camera position
    canvas_set_default_camera(canvas);
    
    printf("Starting IMU Simulator...\n");
    printf("Controls:\n");
    printf("  Left Mouse Button - Orbit camera\n");
    printf("  Right Mouse Button - Pan camera\n");
    printf("  Mouse Wheel - Zoom camera\n");
    printf("  ESC - Exit\n");
    
    // Main loop
    canvas_main_loop(canvas);
    
    // Cleanup
    canvas_destroy(canvas);
    
    return 0;
} 