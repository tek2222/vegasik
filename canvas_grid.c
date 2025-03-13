#include "canvas.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <math.h>

typedef struct {
    int mouse_button_pressed;
    double last_mouse_x;
    double last_mouse_y;
} GridViewerData;

static void draw_callback(Canvas* canvas, void* user_data) {
    // Draw grid
    canvas_draw_grid(5.0f, 1.0f, 0.1f);
    
    // Draw world coordinate frame
    canvas_draw_axis_cross();
    
    // Draw help text using new canvas functions
    canvas_begin_2d_overlay(canvas);
    
    int y = 20;
    canvas_draw_text_at(10, y, "=== CONTROLS ==="); y += 15;
    canvas_draw_text_at(10, y, "Left Mouse - Orbit camera"); y += 15;
    canvas_draw_text_at(10, y, "Right Mouse - Pan camera"); y += 15;
    canvas_draw_text_at(10, y, "Mouse Wheel - Zoom camera"); y += 15;
    canvas_draw_text_at(10, y, "ESC - Exit viewer"); y += 15;
    
    canvas_end_2d_overlay();
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data __attribute__((unused))) {
    if (key == XK_Escape) {
        canvas->should_exit = 1;
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
    // Initialize GLUT
    glutInit(&argc, argv);
    
    // Create viewer data (simplified)
    GridViewerData data = {
        .mouse_button_pressed = 0,
        .last_mouse_x = 0,
        .last_mouse_y = 0
    };
    
    // Create and setup canvas
    Canvas* canvas = canvas_create("Grid Viewer", 1000, 600);
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
    
    printf("Starting Grid Viewer...\n");
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