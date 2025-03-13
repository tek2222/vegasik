#include "canvas.h"
#include "urdf_import.h"
#include "draw_stl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <GL/glut.h>  // For text rendering
#include <GL/freeglut.h>

typedef struct {
    URDFRobot* robot;
    URDFMeshes* meshes;  // Updated to use URDFMeshes
    char** mesh_names;      // Array of link names for meshes
    int num_meshes;
    int draw_visual_meshes; // Flag to control visual mesh drawing
    int draw_collision_meshes; // Flag to control collision mesh drawing
    float mesh_scale;       // Add mesh scale factor
    // Animation state
    int animate;
    int current_joint;
    double current_angle;
    double angle_step;
    int angle_direction;
    int highlighted_joint;  // Index of highlighted joint (-1 for none)
    int loading_phase;      // 0=not started, 1=showing structure, 2=done
    int current_element;     // Which element we're currently drawing
    double last_element_time; // When we last added an element
    int selected_joint_index;  // Renamed from selected_link_index to be more clear
    double manual_joint_angle; // Angle for manual joint control
    double* joint_angles;   // Array to store current angles for each joint
} ViewerData;

static void print_joint_tree(URDFRobot* robot, const char* current_link, int depth) {
    // Print current link
    for (int i = 0; i < depth; i++) printf("  ");
    printf("%s\n", current_link);
    
    // Find and print children
    for (int i = 0; i < urdf_robot_get_num_joints(robot); i++) {
        const char* joint_name = urdf_robot_get_joint_name(robot, i);
        // Remove unused parent variable since we're using current_link
        const char* child = urdf_robot_get_joint_child_link(robot, joint_name);
        
        // If this joint connects from our current link, recurse to its child
        if (strcmp(urdf_robot_get_joint_parent_link(robot, joint_name), current_link) == 0) {
            for (int i = 0; i < depth; i++) printf("  ");
            printf("└─%s\n", joint_name);
            print_joint_tree(robot, child, depth + 1);
        }
    }
}

static void draw_robot(ViewerData* data) {
    // Remove the delay logic and show everything immediately
    if (data->loading_phase == 1) {
        data->loading_phase = 2;  // Skip directly to completed state
        data->current_element = urdf_robot_get_num_joints(data->robot) + 1;
        printf("\nRobot structure complete!\n");
    }
    
    // Draw robot elements up to current_element
    glDisable(GL_LIGHTING);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    
    // Find base link
    const char* base_link = NULL;
    for (int i = 0; i < urdf_robot_get_num_links(data->robot); i++) {
        const char* link = urdf_robot_get_link_name(data->robot, i);
        if (!urdf_robot_get_link_parent_name(data->robot, link)) {
            base_link = link;
            break;
        }
    }
    
    // Function to recursively draw links and joints
    void draw_link_chain(const char* current_link, int* joint_index) {
        Matrix4 current_transform = urdf_robot_get_link_fk(data->robot, current_link);
        
        // Draw coordinate frame for current link
        glPushMatrix();
        glTranslatef(current_transform.data[12], current_transform.data[13], current_transform.data[14]);
        
        GLdouble rot_matrix[16] = {
            current_transform.data[0], current_transform.data[1], current_transform.data[2], 0,
            current_transform.data[4], current_transform.data[5], current_transform.data[6], 0,
            current_transform.data[8], current_transform.data[9], current_transform.data[10], 0,
            0, 0, 0, 1
        };
        glMultMatrixd(rot_matrix);
        
        // Draw axes
        if (*joint_index == data->highlighted_joint) {
            glLineWidth(3.0);
            glBegin(GL_LINES);
            
            // X axis - Red, 1 meter
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(1.0f, 0.0f, 0.0f);
            
            // Y axis - Green, 1 meter
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 1.0f, 0.0f);
            
            // Z axis - Blue, 1 meter
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(0.0f, 0.0f, 0.0f);
            glVertex3f(0.0f, 0.0f, 1.0f);
            
            glEnd();
            glLineWidth(1.0);
        } else {
            glScalef(0.1f, 0.1f, 0.1f);
            canvas_draw_axis_cross();
        }
        glPopMatrix();
        
        // Find the joint that has this link as parent
        for (int i = 0; i < urdf_robot_get_num_joints(data->robot); i++) {
            const char* joint_name = urdf_robot_get_joint_name(data->robot, i);
            const char* parent = urdf_robot_get_joint_parent_link(data->robot, joint_name);
            const char* child = urdf_robot_get_joint_child_link(data->robot, joint_name);
            
            if (parent && strcmp(parent, current_link) == 0) {
                // Draw line to child
                Matrix4 child_transform = urdf_robot_get_link_fk(data->robot, child);
                glColor3f(1.0f, 1.0f, 0.0f);  // Yellow
                glVertex3f(current_transform.data[12], current_transform.data[13], current_transform.data[14]);
                glVertex3f(child_transform.data[12], child_transform.data[13], child_transform.data[14]);
                
                (*joint_index)++;  // Increment joint index before recursing
                draw_link_chain(child, joint_index);  // Continue with child link
            }
        }
    }
    
    glEnd();
    
    // Start drawing from base
    if (base_link) {
        int joint_index = 0;
        draw_link_chain(base_link, &joint_index);
    }
    
    // After drawing all links and joints, draw a large axis cross at the tool
    const char* end_effector = urdf_robot_get_end_effector_link(data->robot);
    if (end_effector && (data->loading_phase == 2 || data->current_element >= urdf_robot_get_num_joints(data->robot))) {
        Matrix4 tool_transform = urdf_robot_get_link_fk(data->robot, end_effector);
        
        glPushMatrix();
        glTranslatef(tool_transform.data[12], tool_transform.data[13], tool_transform.data[14]);
        
        // Apply tool orientation
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
    }
    
    // Draw visual meshes if enabled
    if (data->draw_visual_meshes) {
        glEnable(GL_LIGHTING);
        for (int i = 0; i < data->num_meshes; i++) {
            if (data->meshes->meshes[i] && data->mesh_names[i]) {
                // Set color based on selection
                if (i == data->selected_joint_index) {
                    GLfloat blue_material[] = {0.2f, 0.2f, 1.0f, 1.0f};
                    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blue_material);
                } else {
                    GLfloat default_material[] = {0.7f, 0.7f, 0.7f, 1.0f};
                    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, default_material);
                }
                
                // Find which joint this link belongs to
                int joint_index = -1;  // -1 for base link
                const char* link_name = data->mesh_names[i];
                
                // Special case for base link
                if (!urdf_robot_get_link_parent_name(data->robot, link_name)) {
                    // Base link is always shown (element 0)
                    if (data->current_element >= 0 || data->loading_phase == 2) {
                        Matrix4 link_transform = urdf_robot_get_link_fk(data->robot, link_name);
                        glPushMatrix();
                        GLdouble transform[16] = {
                            link_transform.data[0], link_transform.data[1], link_transform.data[2], link_transform.data[3],
                            link_transform.data[4], link_transform.data[5], link_transform.data[6], link_transform.data[7],
                            link_transform.data[8], link_transform.data[9], link_transform.data[10], link_transform.data[11],
                            link_transform.data[12], link_transform.data[13], link_transform.data[14], link_transform.data[15]
                        };
                        glMultMatrixd(transform);
                        
                        stl_mesh_draw(data->meshes->meshes[i]);
                        glPopMatrix();
                    }
                    continue;
                }
                
                // For other links, find which joint connects to this link
                for (int j = 0; j < urdf_robot_get_num_joints(data->robot); j++) {
                    const char* joint_name = urdf_robot_get_joint_name(data->robot, j);
                    const char* child = urdf_robot_get_joint_child_link(data->robot, joint_name);
                    if (strcmp(child, link_name) == 0) {
                        joint_index = j;
                        break;
                    }
                }
                
                // Only draw if we've reached this joint in the animation
                if ((joint_index >= 0 && joint_index < data->current_element) || 
                    data->loading_phase == 2) {
                    Matrix4 link_transform = urdf_robot_get_link_fk(data->robot, link_name);
                    glPushMatrix();
                    GLdouble transform[16] = {
                        link_transform.data[0], link_transform.data[1], link_transform.data[2], link_transform.data[3],
                        link_transform.data[4], link_transform.data[5], link_transform.data[6], link_transform.data[7],
                        link_transform.data[8], link_transform.data[9], link_transform.data[10], link_transform.data[11],
                        link_transform.data[12], link_transform.data[13], link_transform.data[14], link_transform.data[15]
                    };
                    glMultMatrixd(transform);
                    
                    stl_mesh_draw(data->meshes->meshes[i]);
                    glPopMatrix();
                }
            }
        }
    }

    // Draw collision meshes if enabled
    if (data->draw_collision_meshes) {
        glDisable(GL_LIGHTING);
        for (int i = 0; i < data->num_meshes; i++) {
            if (data->meshes->meshes[i] && data->mesh_names[i]) {
                // Optionally, you can use a different color or style for collision meshes
                glColor3f(1.0f, 0.0f, 0.0f); // Example: Red for collision meshes
                stl_mesh_draw(data->meshes->meshes[i]);
            }
        }
    }
}

static void update_animation(ViewerData* data) {
    if (!data->animate) return;
    
    // Find next actuated joint
    while (data->current_joint < urdf_robot_get_num_joints(data->robot)) {
        const char* joint_name = urdf_robot_get_joint_name(data->robot, data->current_joint);
        int joint_type = urdf_robot_get_joint_type(data->robot, joint_name);
        
        // If we found a revolute joint, animate it
        if (joint_type == 1) {  // 1 = revolute
            // Update angle
            data->current_angle += data->angle_step * data->angle_direction;
            
            // Check bounds and switch direction or move to next joint
            if (data->current_angle > M_PI/4) {  // 45 degrees
                data->current_angle = M_PI/4;
                data->angle_direction = -1;
            } else if (data->current_angle < -M_PI/4) {  // -45 degrees
                data->current_angle = -M_PI/4;
                data->angle_direction = 1;
                
                // Move to next joint
                data->current_joint++;
                data->current_angle = 0.0;  // Reset angle for next joint
            }
            
            // Apply the angle
            urdf_robot_set_joint_angles(data->robot, &joint_name, &data->current_angle, 1);
            return;  // Exit after updating one joint
        } else {
            // Skip non-revolute joints
            data->current_joint++;
        }
    }
    
    // If we've animated all joints, start over
    if (data->current_joint >= urdf_robot_get_num_joints(data->robot)) {
        data->current_joint = 0;
        data->current_angle = 0.0;
        data->angle_direction = 1;
    }
}

// Suppress unused function warning
__attribute__((unused)) static void print_joint_info(URDFRobot* robot, int joint_idx) {
    const char* joint_name = urdf_robot_get_joint_name(robot, joint_idx);
    int joint_type = urdf_robot_get_joint_type(robot, joint_name);
    const char* parent = urdf_robot_get_joint_parent_link(robot, joint_name);
    const char* child = urdf_robot_get_joint_child_link(robot, joint_name);
    
    // Convert joint type to string
    const char* type_str = "fixed";
    if (joint_type == 1) type_str = "revolute";
    else if (joint_type == 2) type_str = "prismatic";
    
    printf("Selected Joint %d:\n", joint_idx + 1);
    printf("  [%s] Joint '%s'\n", type_str, joint_name);
    printf("    Parent Link: %s\n", parent ? parent : "none");
    printf("    Child Link: %s\n", child ? child : "none");
}

static void draw_callback(Canvas* canvas, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;
    
    update_animation(data);
    
    // Draw 3D scene
    canvas_draw_grid(1.0f, 0.5f, 0.1f);
    canvas_draw_axis_cross();
    draw_robot(data);
    
    // Draw help text
    canvas_begin_2d_overlay(canvas);
    
    int y = 20;
    canvas_draw_text_at(10, y, "=== CONTROLS ==="); y += 15;
    canvas_draw_text_at(10, y, "Left Mouse - Orbit camera"); y += 15;
    canvas_draw_text_at(10, y, "Right Mouse - Zoom camera"); y += 15;
    canvas_draw_text_at(10, y, "ESC - Exit"); y += 15;
    canvas_draw_text_at(10, y, "SPACE - Toggle animation"); y += 15;
    canvas_draw_text_at(10, y, "UP - Next joint in chain"); y += 15;
    canvas_draw_text_at(10, y, "DOWN - Previous joint in chain"); y += 15;
    canvas_draw_text_at(10, y, "LEFT - Rotate selected joint left"); y += 15;
    canvas_draw_text_at(10, y, "RIGHT - Rotate selected joint right"); y += 15;
    canvas_draw_text_at(10, y, "V - Toggle visual meshes"); y += 15;  // New line for visual meshes
    canvas_draw_text_at(10, y, "C - Toggle collision meshes"); y += 15; // New line for collision meshes
    
    canvas_end_2d_overlay();
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;

    switch (key) {
        case XK_Escape:
            canvas->should_exit = 1;  // Exit the program
            break;
        case XK_space:
            data->animate = !data->animate;  // Toggle animation
            printf("Animation %s\n", data->animate ? "enabled" : "disabled");
            break;
        case XK_v:  // Toggle visual meshes
            data->draw_visual_meshes = !data->draw_visual_meshes;
            printf("Visual mesh drawing %s\n", data->draw_visual_meshes ? "enabled" : "disabled");
            break;
        case XK_c:  // Toggle collision meshes
            data->draw_collision_meshes = !data->draw_collision_meshes;
            printf("Collision mesh drawing %s\n", data->draw_collision_meshes ? "enabled" : "disabled");
            break;
        case XK_Up:  // Move to next joint in chain
            if (data->selected_joint_index < urdf_robot_get_num_joints(data->robot) - 1) {
                data->selected_joint_index++;
                const char* joint_name = urdf_robot_get_joint_name(data->robot, data->selected_joint_index - 1);
                const char* child_link = urdf_robot_get_joint_child_link(data->robot, joint_name);
                printf("Selected joint: %s (index: %d) -> %s\n", joint_name, data->selected_joint_index - 1, child_link);
            }
            break;
        case XK_Down:  // Move to previous joint in chain
            if (data->selected_joint_index > 1) {  // Don't go below index 1 (keep base link at 0)
                data->selected_joint_index--;
                const char* joint_name = urdf_robot_get_joint_name(data->robot, data->selected_joint_index - 1);
                const char* child_link = urdf_robot_get_joint_child_link(data->robot, joint_name);
                printf("Selected joint: %s (index: %d) -> %s\n", joint_name, data->selected_joint_index - 1, child_link);
            }
            break;
        case XK_Left:  // Rotate selected joint left
            if (data->selected_joint_index > 0) {  // Don't rotate base link (index 0)
                const char* joint_name = urdf_robot_get_joint_name(data->robot, data->selected_joint_index - 1);
                // Check if joint is revolute (type == 1)
                if (urdf_robot_get_joint_type(data->robot, joint_name) == 1) {
                    double* joint_angle = &data->joint_angles[data->selected_joint_index - 1];
                    *joint_angle -= 0.1;  // Decrease angle
                    printf("Joint '%s' angle: %.1f degrees (index: %d)\n", 
                           joint_name, *joint_angle * 180.0 / M_PI, data->selected_joint_index - 1);
                    urdf_robot_set_joint_angles(data->robot, &joint_name, joint_angle, 1);
                } else {
                    printf("Joint '%s' is not revolute - cannot rotate\n", joint_name);
                }
            }
            break;
        case XK_Right:  // Rotate selected joint right
            if (data->selected_joint_index > 0) {  // Don't rotate base link (index 0)
                const char* joint_name = urdf_robot_get_joint_name(data->robot, data->selected_joint_index - 1);
                // Check if joint is revolute (type == 1)
                if (urdf_robot_get_joint_type(data->robot, joint_name) == 1) {
                    double* joint_angle = &data->joint_angles[data->selected_joint_index - 1];
                    *joint_angle += 0.1;  // Increase angle
                    printf("Joint '%s' angle: %.1f degrees (index: %d)\n", 
                           joint_name, *joint_angle * 180.0 / M_PI, data->selected_joint_index - 1);
                    urdf_robot_set_joint_angles(data->robot, &joint_name, joint_angle, 1);
                } else {
                    printf("Joint '%s' is not revolute - cannot rotate\n", joint_name);
                }
            }
            break;
        default:
            break;
    }
}

// Add this function to get link depth
static int get_link_depth(URDFRobot* robot, const char* link_name) {
    int depth = 0;
    const char* current = link_name;
    
    while (current) {
        current = urdf_robot_get_link_parent_name(robot, current);
        if (current) depth++;
    }
    
    return depth;
}

// Add this function to print all links and their depths
static void print_link_depths(URDFRobot* robot) {
    printf("\nLink depths in robot tree:\n");
    for (int i = 0; i < urdf_robot_get_num_links(robot); i++) {
        const char* link_name = urdf_robot_get_link_name(robot, i);
        int depth = get_link_depth(robot, link_name);
        printf("  Link '%s': depth %d\n", link_name, depth);
    }
    printf("\n");
}

// Update mouse callbacks to use new canvas functions
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
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <urdf_file> [-animate] [-no-mesh] [--mesh_scale <scale>] [--draw-visual] [--draw-collision]\n", argv[0]);
        return 1;
    }
    
    const char* urdf_file = argv[1];
    int animate_flag = 0;
    int draw_visual_meshes = 1; // Default to drawing visual meshes
    int draw_collision_meshes = 0; // Default to not drawing collision meshes
    float mesh_scale = 1.0f;  // Default scale is 1.0
    
    // Parse command line flags
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-animate") == 0) {
            animate_flag = 1;
        } else if (strcmp(argv[i], "-no-mesh") == 0) {
            draw_visual_meshes = 0;
            printf("Visual mesh drawing disabled\n");
        } else if (strcmp(argv[i], "--mesh_scale") == 0 && i + 1 < argc) {
            i++;  // Move to scale value
            mesh_scale = atof(argv[i]);
            printf("Using mesh scale factor: %f\n", mesh_scale);
        } else if (strcmp(argv[i], "--draw-visual") == 0) {
            draw_visual_meshes = 1;
            printf("Visual mesh drawing enabled\n");
        } else if (strcmp(argv[i], "--draw-collision") == 0) {
            draw_collision_meshes = 1;
            printf("Collision mesh drawing enabled\n");
        }
    }
    
    // Create viewer data
    ViewerData data = {
        .robot = NULL,
        .meshes = NULL,  // Update ViewerData structure to use URDFMeshes
        .mesh_names = NULL,
        .num_meshes = 0,
        .draw_visual_meshes = draw_visual_meshes,
        .draw_collision_meshes = draw_collision_meshes,
        .mesh_scale = mesh_scale,
        .animate = animate_flag,
        .current_joint = 0,
        .current_angle = 0.0,
        .angle_step = 0.05,
        .angle_direction = 1,
        .highlighted_joint = -1,
        .loading_phase = 1,
        .current_element = 0,
        .selected_joint_index = 0,
        .manual_joint_angle = 0.0,
        .joint_angles = NULL
    };
    
    printf("Starting URDF Viewer...\n");
    printf("Loading URDF from: %s\n", urdf_file);
    
    // Load robot
    data.robot = urdf_robot_create(urdf_file);
    if (!data.robot) {
        fprintf(stderr, "Failed to load URDF file\n");
        return 1;
    }
    
    // Load meshes using the new function
    if (draw_visual_meshes) {
        data.meshes = urdf_robot_load_meshes(data.robot, mesh_scale);
        if (!data.meshes) {
            fprintf(stderr, "Failed to load meshes\n");
            urdf_robot_destroy(data.robot);
            return 1;
        }
    }
    
    // Print link depths
    print_link_depths(data.robot);
    
    // Find deepest link (should be tool0)
    int max_depth = -1;
    const char* deepest_link = NULL;
    
    for (int i = 0; i < urdf_robot_get_num_links(data.robot); i++) {
        const char* link_name = urdf_robot_get_link_name(data.robot, i);
        int depth = get_link_depth(data.robot, link_name);
        if (depth > max_depth) {
            max_depth = depth;
            deepest_link = link_name;
        }
    }
    
    printf("Deepest link is '%s' at depth %d\n", deepest_link, max_depth);
    printf("Current end-effector is '%s'\n", urdf_robot_get_end_effector_link(data.robot));
    
    // Initialize mesh arrays
    data.num_meshes = urdf_robot_get_num_links(data.robot);
    data.mesh_names = (char**)malloc(data.num_meshes * sizeof(char*));
    memset(data.mesh_names, 0, data.num_meshes * sizeof(char*));
    
    // Load meshes for each link
    for (int i = 0; i < data.num_meshes; i++) {
        const char* link_name = urdf_robot_get_link_name(data.robot, i);
        
        // Try to load visual mesh first
        const char* mesh_file = urdf_robot_get_link_visual_mesh(data.robot, link_name, 0);
        if (!mesh_file) {
            // If no visual mesh, try collision mesh
            mesh_file = urdf_robot_get_link_collision_mesh(data.robot, link_name, 0);
        }
        
        if (mesh_file) {
            // Create and load mesh
            data.mesh_names[i] = strdup(link_name);
            
            // Scale the actual triangles
            for (int j = 0; j < data.meshes->meshes[i]->num_vertices / 3; j++) {
                Triangle* tri = &data.meshes->meshes[i]->triangles[j];
                for (int k = 0; k < 3; k++) {
                    tri->vertices[k].x *= mesh_scale;
                    tri->vertices[k].y *= mesh_scale;
                    tri->vertices[k].z *= mesh_scale;
                }
            }
            
            // Now compute the buffers from the scaled triangles
            stl_mesh_compute_buffers(data.meshes->meshes[i]);
        } else {
            printf("No mesh found for link %s\n", link_name);
            data.mesh_names[i] = NULL;
        }
    }
    
    // Initialize joint angles array
    data.joint_angles = (double*)calloc(urdf_robot_get_num_joints(data.robot), sizeof(double));
    if (!data.joint_angles) {
        fprintf(stderr, "Failed to allocate joint angles array\n");
        return 1;
    }
    
    // Print robot structure first
    printf("\nRobot Structure:\n");
    const char* base_link = NULL;
    for (int i = 0; i < urdf_robot_get_num_links(data.robot); i++) {
        const char* link = urdf_robot_get_link_name(data.robot, i);
        if (!urdf_robot_get_link_parent_name(data.robot, link)) {
            base_link = link;
            break;
        }
    }
    if (base_link) {
        print_joint_tree(data.robot, base_link, 0);
    }
    printf("\nStarting visualization...\n");
    
    // Get initial time
    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    data.last_element_time = start_time.tv_sec + start_time.tv_nsec / 1e9;
    
    // Initialize GLUT
    glutInit(&argc, argv);
    
    // Create and setup canvas
    Canvas* canvas = canvas_create("URDF Viewer", 1000, 600);
    canvas_set_draw_callback(canvas, draw_callback, &data);
    canvas_set_key_callback(canvas, key_callback, &data);
    canvas_set_mouse_button_callback(canvas, mouse_button_callback, &data);
    canvas_set_mouse_motion_callback(canvas, mouse_motion_callback, &data);
    
    printf("Starting main loop (press ESC to exit)\n");
    printf("Controls:\n");
    printf("  Left Mouse Button - Orbit camera\n");
    printf("  Right Mouse Button - Zoom camera\n");
    printf("  ESC - Exit\n");
    printf("  SPACE - Toggle animation\n");
    printf("Visualization:\n");
    printf("  Yellow lines - Joint connections\n");
    printf("  RGB axes - Joint coordinate frames\n");
    if (draw_visual_meshes) {
        printf("  Gray meshes - Link geometry\n");
    }
    
    canvas_main_loop(canvas);
    
    // Cleanup
    canvas_destroy(canvas);
    if (data.meshes) {
        urdf_meshes_destroy(data.meshes);
    }
    free(data.mesh_names);
    free(data.joint_angles);  // Free joint angles array
    urdf_robot_destroy(data.robot);
    
    return 0;
} 