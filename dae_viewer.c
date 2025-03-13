#include "canvas.h"
#include "draw_dae.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <math.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

typedef struct {
    DAEMesh* mesh;
    float rotation_angle;  // For model rotation
    int animate;          // Animation toggle flag
    int draw_wireframe;   // Wireframe toggle flag
} ViewerData;

static void draw_callback(Canvas* canvas, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;
    
    // Draw grid (using canvas helper)
    canvas_draw_grid(1.0f, 0.5f, 0.1f);
    
    // Draw world coordinate frame
    canvas_draw_axis_cross();
    
    // Draw DAE mesh
    if (data->mesh) {
        glPushMatrix();
        
        // Allow model rotation
        if (data->animate) {
            glRotatef(data->rotation_angle, 0.0f, 1.0f, 0.0f);
        }
        
        // Set material properties
        GLfloat mat_ambient[] = {0.7f, 0.7f, 0.7f, 1.0f};
        GLfloat mat_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        GLfloat mat_specular[] = {0.2f, 0.2f, 0.2f, 1.0f};
        GLfloat mat_shininess[] = {50.0f};
        
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
        
        // Set wireframe mode if enabled
        if (data->draw_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
        
        // Enable texture if the mesh has it
        glEnable(GL_TEXTURE_2D);
        dae_mesh_draw(data->mesh);
        glDisable(GL_TEXTURE_2D);
        
        // Reset polygon mode
        if (data->draw_wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
        
        glPopMatrix();
        
        // Update rotation for animation
        if (data->animate) {
            data->rotation_angle += 0.5f;
            if (data->rotation_angle >= 360.0f) {
                data->rotation_angle -= 360.0f;
            }
        }
    }
    
    // Draw help text using canvas overlay
    canvas_begin_2d_overlay(canvas);
    
    int y = 20;
    canvas_draw_text_at(10, y, "=== CONTROLS ==="); y += 15;
    canvas_draw_text_at(10, y, "Left Mouse - Orbit camera"); y += 15;
    canvas_draw_text_at(10, y, "Right Mouse - Zoom camera"); y += 15;
    canvas_draw_text_at(10, y, "SPACE - Toggle animation"); y += 15;
    canvas_draw_text_at(10, y, "W - Toggle wireframe"); y += 15;
    canvas_draw_text_at(10, y, "R - Reset view"); y += 15;
    canvas_draw_text_at(10, y, "ESC - Exit viewer"); y += 15;
    
    canvas_end_2d_overlay();
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;
    
    switch (key) {
        case XK_Escape:
            canvas->should_exit = 1;
            break;
        case XK_space:  // Space key to toggle animation
            data->animate = !data->animate;
            printf("Animation %s\n", data->animate ? "started" : "stopped");
            break;
        case XK_w:  // 'W' key to toggle wireframe
            data->draw_wireframe = !data->draw_wireframe;
            printf("Wireframe mode %s\n", data->draw_wireframe ? "enabled" : "disabled");
            break;
        case XK_r:  // 'R' key to reset view
            data->rotation_angle = 0.0f;
            canvas_reset_view(canvas);
            canvas_set_default_camera(canvas);
            printf("View reset\n");
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
    // Initialize GLUT first
    glutInit(&argc, argv);
    
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <dae_file>\n", argv[0]);
        return 1;
    }
    
    // Get absolute path of the DAE file
    char abs_path[PATH_MAX];
    if (realpath(argv[1], abs_path) == NULL) {
        fprintf(stderr, "Error resolving path '%s': %s\n", argv[1], strerror(errno));
        return 1;
    }
    printf("Loading DAE file from: %s\n", abs_path);
    
    // Create viewer data with new fields
    ViewerData data = {
        .mesh = NULL,
        .rotation_angle = 0.0f,
        .animate = 0,
        .draw_wireframe = 0
    };
    
    // Load DAE mesh
    data.mesh = dae_mesh_create();
    if (!data.mesh) {
        fprintf(stderr, "Failed to create mesh structure\n");
        return 1;
    }
    
    if (!dae_mesh_load(data.mesh, abs_path)) {
        fprintf(stderr, "Failed to load DAE file: %s\n", abs_path);
        dae_mesh_destroy(data.mesh);
        return 1;
    }
    
    // Create and setup canvas with new callbacks
    Canvas* canvas = canvas_create("DAE Viewer", 1000, 600);
    if (!canvas) {
        fprintf(stderr, "Failed to create canvas\n");
        dae_mesh_destroy(data.mesh);
        return 1;
    }
    
    canvas_set_draw_callback(canvas, draw_callback, &data);
    canvas_set_key_callback(canvas, key_callback, &data);
    canvas_set_mouse_button_callback(canvas, mouse_button_callback, &data);
    canvas_set_mouse_motion_callback(canvas, mouse_motion_callback, &data);
    
    // Set initial camera position
    canvas_set_default_camera(canvas);
    
    printf("Starting DAE Viewer...\n");
    printf("Controls:\n");
    printf("  Left Mouse Button - Orbit camera\n");
    printf("  Right Mouse Button - Zoom camera\n");
    printf("  SPACE - Toggle animation\n");
    printf("  W - Toggle wireframe\n");
    printf("  R - Reset view\n");
    printf("  ESC - Exit\n");
    
    // Run main loop
    canvas_main_loop(canvas);
    
    // Cleanup
    canvas_destroy(canvas);
    dae_mesh_destroy(data.mesh);
    
    return 0;
} 