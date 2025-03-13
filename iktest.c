#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "urdf_import.h"
#include "vegasik.h"
#include "draw_stl.h"
#include "math3d.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "canvas.h"
#include <X11/keysym.h>
#include <sys/time.h>
#include <GL/freeglut.h>

// Define benchmark parameters
#define BENCHMARK_DISTANCE_THRESHOLD 0.01  // 1cm position+orientation threshold
#define BENCHMARK_MAX_TIME_MS 5000        // 5 second timeout
#define BENCHMARK_TARGET_X 1.5            // Fixed target position
#define BENCHMARK_TARGET_Y 1.3
#define BENCHMARK_TARGET_Z 0.4

// Replace the viewer structure with Canvas-based version
typedef struct {
    URDFRobot* robot;
    VegasIK* solver;
    Vector3 target_position;
    Matrix3 target_orientation;
    double last_cost;
    int ik_active;
    double target_step;
    int circle_mode;
    double circle_angle;
    Vector3 circle_center;
    double circle_radius;
    double circle_speed;
    const char** joint_names;
    int num_joints;
    STLMesh** link_meshes;
    char** mesh_names;
    int num_meshes;
    double target_x;
    double target_y;
    double target_z;
    double target_rx;
    double target_ry;
    double target_rz;
    double* joint_angles;
    double* joint_variations;
    Vector3 elbow_attractor;  // New field for elbow attractor position
    int draw_wireframe;   // Add wireframe toggle flag
    int show_labels;      // Add toggle for joint/link labels
    int show_lines;       // Add toggle for connection lines
} IKTestData;

// Forward declarations
static void* malloc_safe(size_t size);
static char* strdup_safe(const char* str);
static void draw_axis_cross(void);
static void draw_target(const Vector3* pos, const Matrix3* rot);
static void draw_robot(IKTestData* data);
static void generate_random_target(IKTestData* viewer);
static void step_ik(IKTestData* viewer);
static void do_rollout(IKTestData* viewer) __attribute__((unused));
static void print_current_pose(IKTestData* viewer) __attribute__((unused));
static void update_circle_mode(IKTestData* viewer);
static void draw_klines(void) __attribute__((unused));
static void draw_callback(Canvas* canvas, void* user_data);
static void key_callback(Canvas* canvas, KeySym key, void* user_data);
static void mouse_motion_callback(Canvas* canvas, 
                                int x, int y, 
                                void* user_data __attribute__((unused)));
static int count_revolute_joints(URDFRobot* robot);
static Matrix4 create_benchmark_target();
static long get_time_ms();
static void run_benchmark(URDFRobot* robot, VegasIK* solver);
static void draw_elbow_attractor(const Vector3* attractor_pos);
static void mouse_button_callback(Canvas* canvas, 
                                int button, int state, int x, int y, 
                                void* user_data __attribute__((unused)));

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

static char* strdup_safe(const char* str) {
    if (!str) return NULL;
    char* dup = strdup(str);
    if (!dup) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    return dup;
}

// Drawing functions
static void draw_axis_cross(void) {
    // Use the canvas drawing function instead
    canvas_draw_axis_cross();
}

static void draw_target(const Vector3* pos, const Matrix3* rot) {
    canvas_draw_axis_cross_with_label(pos, rot, "Target");
}

static void draw_elbow_attractor(const Vector3* attractor_pos) {
    // Create an identity rotation for the attractor
    Matrix3 identity = {{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    }};
    
    // Draw a smaller axis cross for the attractor
    glPushMatrix();
    glScalef(0.5f, 0.5f, 0.5f);  // Make it smaller
    canvas_draw_axis_cross_with_label(attractor_pos, &identity, "Elbow");
    glPopMatrix();
}

static void draw_robot(IKTestData* data) {
    // Draw robot elements
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    
    // Function to recursively draw links and joints with labels
    void draw_link_chain(const char* link_name) {
        if (!link_name) return;

        // Get all child joints
        for (int i = 0; i < urdf_robot_get_num_joints(data->robot); i++) {
            const char* joint_name = urdf_robot_get_joint_name(data->robot, i);
            const char* parent = urdf_robot_get_joint_parent_link(data->robot, joint_name);
            const char* child = urdf_robot_get_joint_child_link(data->robot, joint_name);
            
            if (parent && strcmp(parent, link_name) == 0) {
                // Get joint transform
                Matrix4 child_transform = urdf_robot_get_link_fk(data->robot, child);
                
                // Draw joint frame with label
                Vector3 joint_pos = {
                    child_transform.data[12],
                    child_transform.data[13],
                    child_transform.data[14]
                };
                
                Matrix3 joint_rot = {{
                    child_transform.data[0], child_transform.data[1], child_transform.data[2],
                    child_transform.data[4], child_transform.data[5], child_transform.data[6],
                    child_transform.data[8], child_transform.data[9], child_transform.data[10]
                }};
                
                glPushMatrix();
                glTranslatef(joint_pos.x, joint_pos.y, joint_pos.z);
                glScalef(0.1f, 0.1f, 0.1f);  // Make joint axes smaller
                canvas_draw_axis_cross_with_label(&joint_pos, &joint_rot, joint_name);
                glPopMatrix();
                
                // Recursive call
                draw_link_chain(child);
            }
        }
    }
    
    // Start recursive drawing from base link
    const char* base_link = NULL;
    for (int i = 0; i < urdf_robot_get_num_links(data->robot); i++) {
        const char* link = urdf_robot_get_link_name(data->robot, i);
        if (!urdf_robot_get_link_parent_name(data->robot, link)) {
            base_link = link;
            break;
        }
    }
    
    if (base_link) {
        draw_link_chain(base_link);
    }
    
    // Draw target position with label
    char target_text[64];
    snprintf(target_text, sizeof(target_text), "Target (%.2f, %.2f, %.2f)", 
             data->target_position.x,
             data->target_position.y,
             data->target_position.z);
    
    canvas_draw_axis_cross_with_label(&data->target_position, &data->target_orientation, target_text);
    
    // Draw end-effector axes (2 meters long, RGB colored)
    const char* end_effector = urdf_robot_get_end_effector_link(data->robot);
    Matrix4 tool_transform = urdf_robot_get_link_fk(data->robot, end_effector);
    
    glPushMatrix();
    glTranslatef(tool_transform.data[12], tool_transform.data[13], tool_transform.data[14]);
    
    GLdouble rot_matrix[16] = {
        tool_transform.data[0], tool_transform.data[1], tool_transform.data[2], 0,
        tool_transform.data[4], tool_transform.data[5], tool_transform.data[6], 0,
        tool_transform.data[8], tool_transform.data[9], tool_transform.data[10], 0,
        0, 0, 0, 1
    };
    glMultMatrixd(rot_matrix);
    
    // Draw large axes (2 meters long, RGB colored)
    glLineWidth(3.0);
    glBegin(GL_LINES);
    
    // X axis - Red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(2.0f, 0.0f, 0.0f);
    
    // Y axis - Green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 2.0f, 0.0f);
    
    // Z axis - Blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 2.0f);
    
    glEnd();
    glLineWidth(1.0);
    
    glPopMatrix();
    
    // Add back the mesh drawing code:
    // Draw meshes
    if (data->link_meshes) {
        glEnable(GL_LIGHTING);
        glEnable(GL_DEPTH_TEST);
        
        // Set material properties
        GLfloat mat_ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
        GLfloat mat_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        GLfloat mat_specular[] = {0.1f, 0.1f, 0.1f, 1.0f};
        GLfloat mat_shininess[] = {50.0f};
        
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

        // Set wireframe mode if enabled
        if (data->draw_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        
        // Draw each mesh with proper transform
        for (int i = 0; i < data->num_meshes; i++) {
            if (data->link_meshes[i]) {
                const char* link_name = urdf_robot_get_link_name(data->robot, i);
                Matrix4 transform = urdf_robot_get_link_fk(data->robot, link_name);
                
                // Convert to OpenGL matrix
                GLfloat gl_matrix[16] = {
                    transform.data[0], transform.data[1], transform.data[2], transform.data[3],
                    transform.data[4], transform.data[5], transform.data[6], transform.data[7],
                    transform.data[8], transform.data[9], transform.data[10], transform.data[11],
                    transform.data[12], transform.data[13], transform.data[14], transform.data[15]
                };
                
                glPushMatrix();
                glMultMatrixf(gl_matrix);
                stl_mesh_draw(data->link_meshes[i]);
                glPopMatrix();
            }
        }

        // Reset polygon mode
        if (data->draw_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }

    // Print joint types for debugging
    for (int i = 0; i < data->num_joints; i++) {
        int joint_type = urdf_robot_get_joint_type(data->robot, data->joint_names[i]);
        printf("Joint %d: Type %d\n", i, joint_type);  // Print joint type
    }

    // Calculate the elbow position (3rd revolute joint)
    Vector3 elbow_position;
    int revolute_count = 0;
    int elbow_joint_index = -1;

    for (int i = 0; i < data->num_joints; i++) {
        if (urdf_robot_get_joint_type(data->robot, data->joint_names[i]) == 1) {  // 1 indicates revolute
            revolute_count++;
            if (revolute_count == 3) {  // We found the 3rd revolute joint
                elbow_joint_index = i;
            }
        }
    }

    if (elbow_joint_index != -1) {
        Matrix4 elbow_transform = urdf_robot_get_link_fk(data->robot, data->joint_names[elbow_joint_index]);  // 3rd revolute joint
        elbow_position = vector3_create(elbow_transform.data[12], 
                                         elbow_transform.data[13],  // Keep the same Y position
                                         elbow_transform.data[14]);  // Z position
        data->elbow_attractor = vector3_create(elbow_transform.data[12], 
                                                elbow_transform.data[13],  // Keep the same Y position
                                                elbow_transform.data[14] + 5.0);  // Position it 5m above the elbow in Z
    }

    // Print elbow attractor position for debugging
    printf("Elbow Attractor Position: (%f, %f, %f)\n", 
           data->elbow_attractor.x, 
           data->elbow_attractor.y, 
           data->elbow_attractor.z);

    // Draw the elbow attractor
    draw_elbow_attractor(&data->elbow_attractor);

    // Draw a line from the elbow joint to the elbow attractor
    glBegin(GL_LINES);
    glColor3f(1.0f, 1.0f, 0.0f);  // Yellow color for the line
    glVertex3f(elbow_position.x, elbow_position.y, elbow_position.z);  // Elbow joint position
    glVertex3f(data->elbow_attractor.x, data->elbow_attractor.y, data->elbow_attractor.z);  // Attractor position
    glEnd();

    // Draw lines from each revolute joint to the elbow attractor
    for (int i = 0; i < data->num_joints; i++) {
        if (urdf_robot_get_joint_type(data->robot, data->joint_names[i]) == 1) {  // Check if it's a revolute joint
            Matrix4 joint_transform = urdf_robot_get_link_fk(data->robot, data->joint_names[i]);
            Vector3 joint_position = vector3_create(joint_transform.data[12], 
                                                    joint_transform.data[13], 
                                                    joint_transform.data[14]);

            // Draw a line from the revolute joint to the elbow attractor
            glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f);  // Green color for the line
            glVertex3f(joint_position.x, joint_position.y, joint_position.z);  // Joint position
            glVertex3f(data->elbow_attractor.x, data->elbow_attractor.y, data->elbow_attractor.z);  // Attractor position
            glEnd();
        }
    }

    // Calculate the position of the end effector
    Vector3 end_effector_position;
    const char* end_effector_joint = urdf_robot_get_end_effector_link(data->robot);
    Matrix4 end_effector_transform = urdf_robot_get_link_fk(data->robot, end_effector_joint);
    end_effector_position = vector3_create(end_effector_transform.data[12], 
                                            end_effector_transform.data[13], 
                                            end_effector_transform.data[14]);

    // Draw a line from the end effector to the elbow attractor
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 1.0f);  // Blue color for the line
    glVertex3f(end_effector_position.x, end_effector_position.y, end_effector_position.z);  // End effector position
    glVertex3f(data->elbow_attractor.x, data->elbow_attractor.y, data->elbow_attractor.z);  // Attractor position
    glEnd();

    // Only draw labels if enabled
    if (data->show_labels) {
        // Draw joint names
        for (int i = 0; i < data->num_joints; i++) {
            const char* joint_name = data->joint_names[i];
            Matrix4 joint_transform = urdf_robot_get_link_fk(data->robot, joint_name);
            Vector3 joint_position = vector3_create(joint_transform.data[12], 
                                                    joint_transform.data[13], 
                                                    joint_transform.data[14]);
            canvas_draw_text_3d(joint_position.x + 0.1f, joint_position.y + 0.1f, joint_position.z + 0.1f, joint_name);
        }
    }

    // Only draw connection lines if enabled
    if (data->show_lines) {
        // Draw connection lines
        for (int i = 0; i < data->num_joints; i++) {
            const char* joint_name = data->joint_names[i];
            const char* parent_name = urdf_robot_get_joint_parent_link(data->robot, joint_name);
            if (parent_name) {
                Matrix4 parent_transform = urdf_robot_get_link_fk(data->robot, parent_name);
                Vector3 parent_position = vector3_create(parent_transform.data[12], 
                                                        parent_transform.data[13], 
                                                        parent_transform.data[14]);
                Vector3 joint_position = vector3_create(urdf_robot_get_link_fk(data->robot, joint_name).data[12], 
                                                        urdf_robot_get_link_fk(data->robot, joint_name).data[13], 
                                                        urdf_robot_get_link_fk(data->robot, joint_name).data[14]);
                glBegin(GL_LINES);
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow color for connection lines
                glVertex3f(parent_position.x, parent_position.y, parent_position.z);
                glVertex3f(joint_position.x, joint_position.y, joint_position.z);
                glEnd();
            }
        }
    }
}

static void draw_klines(void) {
    // Disable lighting for drawing lines
    glDisable(GL_LIGHTING);
    
    glLineWidth(1.0f);  // Set thin lines
    
    // Draw a grid of lines
    glBegin(GL_LINES);
    
    // Parameters
    float step = 1.0f;     // 1 meter between lines
    float size = 5.0f;     // 5x5 meter grid
    
    // Draw minor grid lines (10cm spacing)
    glColor3f(0.2f, 0.2f, 0.2f);  // Darker gray for minor lines
    float minor_step = 0.1f;  // 10cm between minor lines
    
    // Draw minor lines along X axis
    for (float z = -size; z <= size + 0.01f; z += minor_step) {
        // Skip if this is a major line
        if (fabs(fmod(z, step)) < 0.01f) continue;
        glVertex3f(-size, 0.0f, z);
        glVertex3f(size, 0.0f, z);
    }
    
    // Draw minor lines along Z axis
    for (float x = -size; x <= size + 0.01f; x += minor_step) {
        // Skip if this is a major line
        if (fabs(fmod(x, step)) < 0.01f) continue;
        glVertex3f(x, 0.0f, -size);
        glVertex3f(x, 0.0f, size);
    }
    
    // Draw major grid lines (1m spacing)
    glColor3f(0.4f, 0.4f, 0.4f);  // Brighter gray for major lines
    
    // Draw lines along X axis
    for (float z = -size; z <= size + 0.01f; z += step) {
        glVertex3f(-size, 0.0f, z);
        glVertex3f(size, 0.0f, z);
    }
    
    // Draw lines along Z axis
    for (float x = -size; x <= size + 0.01f; x += step) {
        glVertex3f(x, 0.0f, -size);
        glVertex3f(x, 0.0f, size);
    }
    
    glEnd();
    
    // Re-enable lighting
    glEnable(GL_LIGHTING);
}

// IK functions
static void generate_random_target(IKTestData* viewer) {
    // Generate random position in a larger workspace range
    viewer->target_position.x = (rand() / (float)RAND_MAX) * 1.0 - 0.2;   // -0.2 to 0.8 meters
    viewer->target_position.y = (rand() / (float)RAND_MAX) * 1.2 - 0.6;   // -0.6 to 0.6 meters
    viewer->target_position.z = (rand() / (float)RAND_MAX) * 0.8 + 0.2;   // 0.2 to 1.0 meters
    
    // Keep the same orientation generation
    viewer->target_orientation = matrix3_identity();
    
    // Reset circle mode parameters
    viewer->circle_mode = 0;
    viewer->circle_center = viewer->target_position;
    viewer->circle_angle = 0.0;
    
    printf("Generated new random target at (%.3f, %.3f, %.3f)\n",
           viewer->target_position.x,
           viewer->target_position.y,
           viewer->target_position.z);
}

static void step_ik(IKTestData* viewer) {
    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    // Create target pose matrix from position and orientation
    Matrix4 target_pose = matrix4_from_matrix3_vector3(viewer->target_orientation, 
                                                      viewer->target_position);
    
    // Get current joint angles
    double current_angles[viewer->num_joints];
    for (int i = 0; i < viewer->num_joints; i++) {
        current_angles[i] = viewer->joint_angles[i];
    }
    
    // Always try to improve the solution
    IKSolution* solution = vegas_ik_solve(viewer->solver, 
                                        &target_pose,
                                        viewer->joint_names,
                                        current_angles,
                                        viewer->num_joints,
                                        500,
                                        1);
    
    if (solution) {
        // Update joint angles with solution
        for (int i = 0; i < viewer->num_joints; i++) {
            viewer->joint_angles[i] = solution->joint_angles[i];
            urdf_robot_set_joint_angles(viewer->robot, 
                                      &viewer->joint_names[i],
                                      &viewer->joint_angles[i], 
                                      1);
        }
        
        // Free solution using proper destructor
        ik_solution_destroy(solution);
    }
    
    // End timing
    struct timespec end_time;
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    double elapsed = (end_time.tv_sec - start_time.tv_sec) +
                    (end_time.tv_nsec - start_time.tv_nsec) / 1e9;
    
    // Print timing info
    printf("IK step took %.3f ms\n", elapsed * 1000.0);
    
    // Print current end effector pose
    Matrix4 current_pose = urdf_robot_get_link_fk(viewer->robot,
                                                 urdf_robot_get_end_effector_link(viewer->robot));
    Vector3 current_pos = vector3_create(current_pose.data[12], 
                                       current_pose.data[13], 
                                       current_pose.data[14]);
    printf("Current end effector: x=%.3f y=%.3f z=%.3f\n",
           current_pos.x, current_pos.y, current_pos.z);

    // Print target vs current orientation
    const char* tool_link = urdf_robot_get_end_effector_link(viewer->robot);
    Matrix4 tool_pose = urdf_robot_get_link_fk(viewer->robot, tool_link);
    
    printf("\nTarget orientation:\n");
    for (int i = 0; i < 3; i++) {
        printf("[%.3f %.3f %.3f]\n",
               viewer->target_orientation.data[i*3],
               viewer->target_orientation.data[i*3+1],
               viewer->target_orientation.data[i*3+2]);
    }
    
    printf("\nTool orientation (%s):\n", tool_link);
    for (int i = 0; i < 3; i++) {
        printf("[%.3f %.3f %.3f]\n",
               tool_pose.data[i*4],    // Note: Matrix4 has 4 elements per row
               tool_pose.data[i*4+1],
               tool_pose.data[i*4+2]);
    }
}

static void do_rollout(IKTestData* viewer) {
    // Create target pose matrix
    Matrix4 target_pose = matrix4_from_matrix3_vector3(viewer->target_orientation, 
                                                      viewer->target_position);
    
    // Get current joint angles
    double* current_angles = (double*)malloc_safe(viewer->num_joints * sizeof(double));
    for (int i = 0; i < viewer->num_joints; i++) {
        current_angles[i] = urdf_robot_get_joint_angle(viewer->robot, viewer->joint_names[i]);
    }
    
    // Get a rollout
    IKSolution* rollout = vegas_ik_get_rollout(viewer->solver,
                                              &target_pose,
                                              (const char**)viewer->joint_names,
                                              current_angles,
                                              viewer->num_joints);
    
    if (rollout) {
        // Apply rollout angles
        urdf_robot_set_joint_angles(viewer->robot,
                                  (const char**)rollout->joint_names,
                                  rollout->joint_angles,
                                  rollout->num_joints);
        
        viewer->last_cost = rollout->cost;
        ik_solution_destroy(rollout);
    }
    
    free(current_angles);
}

static void print_current_pose(IKTestData* viewer) {
    Matrix4 current_pose = urdf_robot_get_link_fk(viewer->robot,
                                                 urdf_robot_get_end_effector_link(viewer->robot));
    Vector3 current_pos = vector3_create(current_pose.data[12], 
                                       current_pose.data[13], 
                                       current_pose.data[14]);
    printf("Current end effector: x=%.3f y=%.3f z=%.3f\n",
           current_pos.x, current_pos.y, current_pos.z);
}

static void update_circle_mode(IKTestData* viewer) {
    if (viewer->circle_mode) {
        // Update target position in a circular path
        Vector3 offset = vector3_create(
            viewer->circle_radius * cos(viewer->circle_angle),
            viewer->circle_radius * sin(viewer->circle_angle),
            0  // Keep same height
        );
        viewer->target_position = vector3_add(viewer->circle_center, offset);
        
        viewer->circle_angle += viewer->circle_speed;
        if (viewer->circle_angle >= 2 * M_PI) {
            viewer->circle_angle -= 2 * M_PI;
        }
    }
}

// Add these callback functions before main():

static void draw_callback(Canvas* canvas, void* user_data) {
    IKTestData* data = (IKTestData*)user_data;
    
    // Update IK if active
    if (data->ik_active) {
        step_ik(data);
    }
    
    // Update circle mode if active
    if (data->circle_mode) {
        update_circle_mode(data);
    }
    
    // Draw scene
    canvas_draw_grid(5.0f, 1.0f, 0.1f);
    canvas_draw_axis_cross();
    draw_robot(data);
    draw_target(&data->target_position, &data->target_orientation);
    
    // Draw target axes (0.5 meters long, colored)
    glPushMatrix();
    glTranslatef(data->target_position.x, data->target_position.y, data->target_position.z);
    
    // Convert target orientation Matrix3 to OpenGL matrix
    GLdouble rot_matrix[16] = {
        data->target_orientation.data[0], data->target_orientation.data[1], data->target_orientation.data[2], 0,
        data->target_orientation.data[3], data->target_orientation.data[4], data->target_orientation.data[5], 0,
        data->target_orientation.data[6], data->target_orientation.data[7], data->target_orientation.data[8], 0,
        0, 0, 0, 1
    };
    glMultMatrixd(rot_matrix);
    
    // Draw axes using canvas function
    canvas_draw_axis_cross();
    
    glPopMatrix();

    // Draw help text using canvas overlay
    canvas_begin_2d_overlay(canvas);
    
    int y = 20;
    canvas_draw_text_at(10, y, "=== CONTROLS ==="); y += 15;
    canvas_draw_text_at(10, y, "Left Mouse - Orbit camera"); y += 15;
    canvas_draw_text_at(10, y, "Right Mouse - Zoom camera"); y += 15;
    canvas_draw_text_at(10, y, "SPACE - Toggle IK solver"); y += 15;
    canvas_draw_text_at(10, y, "R - Generate random target"); y += 15;
    canvas_draw_text_at(10, y, "C - Toggle circle mode"); y += 15;
    canvas_draw_text_at(10, y, "W - Toggle wireframe"); y += 15;
    canvas_draw_text_at(10, y, "L - Toggle joint labels"); y += 15;
    canvas_draw_text_at(10, y, "P - Cycle perturbation strategy"); y += 15;
    canvas_draw_text_at(10, y, "Arrow keys - Move target"); y += 15;
    canvas_draw_text_at(10, y, "ESC - Exit"); y += 15;
    
    canvas_end_2d_overlay();
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data) {
    IKTestData* data = (IKTestData*)user_data;
    
    if (key == XK_Escape) {
        canvas->should_exit = 1;
    } else if (key == XK_space) {
        data->ik_active = !data->ik_active;
        printf("IK solver %s\n", data->ik_active ? "started" : "stopped");
    } else if (key == XK_r) {
        generate_random_target(data);
    } else if (key == XK_c) {
        data->circle_mode = !data->circle_mode;
        if (data->circle_mode) {
            data->circle_center = data->target_position;
            data->circle_angle = 0;
        }
    } else if (key == XK_m) {  // Add 'm' key to toggle sampling strategy
        if (data->solver->perturb_strategy == PERTURB_MULTISCALE) {
            vegas_ik_set_strategy(data->solver, PERTURB_UNIFORM);
            printf("Switched to adaptive sampling\n");
        } else {
            vegas_ik_set_strategy(data->solver, PERTURB_MULTISCALE);
            printf("Switched to multiscale sampling\n");
        }
    } else if (key == XK_p) {
        // Cycle through perturbation strategies
        data->solver->perturb_strategy = (data->solver->perturb_strategy + 1) % 4;
        const char* strat_names[] = {
            "uniform", 
            "position (large base)", 
            "weighted",
            "endeffector (large wrist)"
        };
        printf("Switched to %s perturbation strategy\n", strat_names[data->solver->perturb_strategy]);
    } else if (key == XK_l) {
        // Toggle all labels at once
        int new_state = !canvas_get_show_axis_labels(canvas);
        canvas_set_show_axis_labels(canvas, new_state);
        printf("Labels %s\n", new_state ? "enabled" : "disabled");
    } else if (key == XK_w) {  // 'W' key to toggle wireframe
        data->draw_wireframe = !data->draw_wireframe;
        printf("Wireframe mode %s\n", data->draw_wireframe ? "enabled" : "disabled");
    } else if (key == XK_v) {  // 'V' key to toggle connection lines
        data->show_lines = !data->show_lines;
        printf("Connection lines %s\n", data->show_lines ? "enabled" : "disabled");
    }
    // Add cursor key controls
    else if (key == XK_Left) {
        data->target_position.y -= 0.05;  // Move left
        data->circle_mode = 0;  // Disable circle mode when manually moving
    }
    else if (key == XK_Right) {
        data->target_position.y += 0.05;  // Move right
        data->circle_mode = 0;
    }
    else if (key == XK_Up) {
        data->target_position.z += 0.05;  // Move up
        data->circle_mode = 0;
    }
    else if (key == XK_Down) {
        data->target_position.z -= 0.05;  // Move down
        data->circle_mode = 0;
    }
    else if (key == XK_Page_Up) {
        data->target_position.x += 0.05;  // Move forward
        data->circle_mode = 0;
    }
    else if (key == XK_Page_Down) {
        data->target_position.x -= 0.05;  // Move back
        data->circle_mode = 0;
    }
}

static void mouse_motion_callback(Canvas* canvas, 
                                int x, int y, 
                                void* user_data __attribute__((unused))) {
    canvas_handle_mouse_motion(canvas, x, y);
}

static void mouse_button_callback(Canvas* canvas, 
                                int button, int state, int x, int y, 
                                void* user_data __attribute__((unused))) {
    canvas_handle_mouse_button(canvas, button, state, x, y);
}

static int count_revolute_joints(URDFRobot* robot) {
    int revolute_count = 0;
    printf("\nJoint Information:\n");
    
    for (int i = 0; i < urdf_robot_get_num_joints(robot); i++) {
        const char* joint_name = urdf_robot_get_joint_name(robot, i);
        int joint_type = urdf_robot_get_joint_type(robot, joint_name);
        
        if (joint_type == 1) {  // 1 = revolute
            revolute_count++;
            printf("Joint %d: '%s' (revolute)\n", i+1, joint_name);
        } else {
            printf("Joint %d: '%s' (%s)\n", i+1, joint_name, 
                   joint_type == 0 ? "fixed" : "prismatic");
        }
    }
    
    printf("\nFound %d revolute joints\n", revolute_count);
    return revolute_count;
}

static Matrix4 create_benchmark_target() {
    // Create a fixed target pose for benchmarking
    Matrix4 target = matrix4_identity();
    target.data[12] = BENCHMARK_TARGET_X;
    target.data[13] = BENCHMARK_TARGET_Y;
    target.data[14] = BENCHMARK_TARGET_Z;
    
    // Add a fixed orientation if desired
    // For now, keeping identity rotation
    
    return target;
}

// Returns time in milliseconds
static long get_time_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

static void run_benchmark(URDFRobot* robot, VegasIK* solver) {
    printf("\n=== Starting IK Solver Benchmark ===\n");
    
    // Create fixed target pose
    Matrix4 target_pose = create_benchmark_target();
    
    // Get initial joint configuration
    int num_joints = robot->num_joints;
    const char** joint_names = (const char**)malloc(num_joints * sizeof(char*));
    double* initial_angles = (double*)malloc(num_joints * sizeof(double));
    
    // Initialize with zero angles
    for (int i = 0; i < num_joints; i++) {
        joint_names[i] = robot->joints[i].name;
        initial_angles[i] = 0.0;
    }
    
    // Start timing
    long start_time = get_time_ms();
    long current_time;
    IKSolution* solution = NULL;
    int iterations = 0;
    double best_cost = DBL_MAX;
    
    // Run until convergence or timeout
    do {
        solution = vegas_ik_solve(solver, &target_pose, joint_names, 
                                initial_angles, num_joints, 20, 1);
        
        if (solution) {
            best_cost = solution->cost;
            // Update initial angles for next iteration
            memcpy(initial_angles, solution->joint_angles, num_joints * sizeof(double));
            ik_solution_destroy(solution);
        }
        
        current_time = get_time_ms();
        iterations++;
        
    } while (best_cost > BENCHMARK_DISTANCE_THRESHOLD && 
             (current_time - start_time) < BENCHMARK_MAX_TIME_MS);
    
    // Print results
    long total_time = current_time - start_time;
    printf("\n=== Benchmark Results ===\n");
    printf("Time taken: %ld ms\n", total_time);
    printf("Iterations: %d\n", iterations);
    printf("Final cost: %.6f\n", best_cost);
    if (best_cost <= BENCHMARK_DISTANCE_THRESHOLD) {
        printf("Status: CONVERGED\n");
    } else {
        printf("Status: TIMEOUT\n");
    }
    printf("========================\n");
    
    free(joint_names);
    free(initial_angles);
}

// Main function using Canvas
int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <urdf_file> [-perturb uniform|position|weighted|multiscale] [--benchmark]\n", argv[0]);
        return 1;
    }
    
    // Initialize GLUT first
    glutInit(&argc, argv);
    
    // Initialize test data
    IKTestData data = {0};
    int benchmark_mode = 0;
    
    // Load URDF file
    const char* urdf_file = argv[1];
    
    // Initialize random seed
    srand(time(NULL));
    
    // Get base directory from URDF path
    char* base_dir = strdup_safe(urdf_file);
    char* last_slash = strrchr(base_dir, '/');
    if (last_slash) {
        *last_slash = '\0';  // Cut off filename to get directory
    } else {
        free(base_dir);
        base_dir = strdup_safe(".");
    }
    
    // Load robot
    data.robot = urdf_robot_create(urdf_file);
    if (!data.robot) {
        fprintf(stderr, "Failed to load URDF file\n");
        return 1;
    }
    
    // Create solver
    data.solver = vegas_ik_create(data.robot);
    vegas_ik_set_seed(data.solver, 123456789);
    
    // Parse command line arguments
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-perturb") == 0 && i + 1 < argc) {
            i++;  // Move to strategy argument
            if (strcmp(argv[i], "uniform") == 0) {
                data.solver->perturb_strategy = PERTURB_UNIFORM;
            } else if (strcmp(argv[i], "position") == 0) {
                data.solver->perturb_strategy = PERTURB_POSITION;
            } else if (strcmp(argv[i], "weighted") == 0) {
                data.solver->perturb_strategy = PERTURB_WEIGHTED;
            } else if (strcmp(argv[i], "multiscale") == 0) {
                data.solver->perturb_strategy = PERTURB_MULTISCALE;
            } else {
                fprintf(stderr, "Unknown perturbation strategy: %s\n", argv[i]);
                return 1;
            }
            printf("Using perturbation strategy: %s\n", argv[i]);
        } else if (strcmp(argv[i], "--benchmark") == 0) {
            benchmark_mode = 1;
            printf("Benchmark mode enabled\n");
        }
    }

    // If benchmark mode is enabled, run it and exit
    if (benchmark_mode) {
        run_benchmark(data.robot, data.solver);
        // Cleanup and exit
        urdf_robot_destroy(data.robot);
        vegas_ik_destroy(data.solver);
        free(base_dir);
        return 0;
    }
    
    // Count revolute joints and print info
    int num_revolute = count_revolute_joints(data.robot);
    if (num_revolute == 0) {
        fprintf(stderr, "Error: Robot has no revolute joints!\n");
        return 1;
    }
    
    // Allocate arrays only for revolute joints
    data.joint_names = malloc(num_revolute * sizeof(const char*));
    data.joint_angles = malloc(num_revolute * sizeof(double));
    data.joint_variations = malloc(num_revolute * sizeof(double));
    
    // Fill arrays with only revolute joints
    int revolute_idx = 0;
    for (int i = 0; i < urdf_robot_get_num_joints(data.robot); i++) {
        const char* joint_name = urdf_robot_get_joint_name(data.robot, i);
        int joint_type = urdf_robot_get_joint_type(data.robot, joint_name);
        
        if (joint_type == 1) {  // revolute
            data.joint_names[revolute_idx] = strdup(joint_name);
            data.joint_angles[revolute_idx] = 0.0;
            data.joint_variations[revolute_idx] = 0.1;
            revolute_idx++;
        }
    }
    
    data.num_joints = num_revolute;  // Use only revolute count
    printf("IK test initialized with %d revolute joints\n", num_revolute);
    
    // Load meshes
    data.num_meshes = urdf_robot_get_num_links(data.robot);
    data.link_meshes = (STLMesh**)malloc_safe(data.num_meshes * sizeof(STLMesh*));
    data.mesh_names = (char**)malloc_safe(data.num_meshes * sizeof(char*));
    
    for (int i = 0; i < data.num_meshes; i++) {
        const char* link_name = urdf_robot_get_link_name(data.robot, i);
        data.mesh_names[i] = strdup_safe(link_name);
        
        char stl_path[256];
        snprintf(stl_path, sizeof(stl_path), "%s/meshes/%s.stl", base_dir, link_name);
        
        data.link_meshes[i] = stl_mesh_create();
        if (!stl_mesh_load(data.link_meshes[i], stl_path)) {
            printf("Note: No mesh found for link %s\n", link_name);
            stl_mesh_destroy(data.link_meshes[i]);
            data.link_meshes[i] = NULL;
        }
    }
    
    free(base_dir);
    
    // Initialize IK parameters
    data.circle_radius = 0.3;  // 30cm
    data.circle_speed = 0.02;  // radians per frame
    data.target_step = 0.01;   // 1cm
    
    // Initialize target orientation to identity
    data.target_orientation = matrix3_identity();
    
    // Generate initial target
    generate_random_target(&data);

    // Initialize new fields
    data.draw_wireframe = 0;
    data.show_labels = 1;    // Labels on by default
    data.show_lines = 1;      // Lines on by default
    
    // Create and setup canvas
    Canvas* canvas = canvas_create("IK Test", 1000, 600);
    canvas_set_draw_callback(canvas, draw_callback, &data);
    canvas_set_key_callback(canvas, key_callback, &data);
    canvas_set_mouse_button_callback(canvas, mouse_button_callback, &data);
    canvas_set_mouse_motion_callback(canvas, mouse_motion_callback, &data);
    
    // Print controls
    printf("\nIK Test Controls:\n");
    printf("================\n");
    printf("ESC - Exit program\n");
    printf("SPACE - Toggle IK solver (start/stop)\n");
    printf("R - Generate new random target\n");
    printf("C - Toggle circle mode (target moves in circle)\n");
    printf("W - Toggle wireframe mode\n");
    printf("L - Toggle joint labels\n");
    printf("P - Cycle through perturbation strategies\n");
    printf("Arrow keys - Move target\n");
    printf("\nVisualization:\n");
    printf("  Yellow lines - Joint connections\n");
    printf("  RGB axes - Joint coordinate frames\n");
    printf("  Gray meshes - Link geometry\n");
    printf("  Colored axes at target - Target pose\n");
    printf("\n");
    
    // Main loop using canvas
    canvas_main_loop(canvas);
    
    // Cleanup
    canvas_destroy(canvas);
    urdf_robot_destroy(data.robot);
    vegas_ik_destroy(data.solver);
    
    for (int i = 0; i < data.num_meshes; i++) {
        if (data.link_meshes[i]) {
            stl_mesh_destroy(data.link_meshes[i]);
        }
        free(data.mesh_names[i]);
    }
    free(data.link_meshes);
    free(data.mesh_names);
    
    // Cleanup
    for (int i = 0; i < data.num_joints; i++) {
        free((void*)data.joint_names[i]);  // Cast to void* to avoid const warning
    }
    free(data.joint_names);
    free(data.joint_angles);
    free(data.joint_variations);
    
    return 0;
} 