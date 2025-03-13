#include "canvas.h"
#include "draw_stl.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    STLMesh* mesh;
    float scale;
} ViewerData;

static void print_usage(const char* program_name) {
    fprintf(stderr, "Usage: %s <stl_file> [options]\n", program_name);
    fprintf(stderr, "Options:\n");
    fprintf(stderr, "  --scale <value>    Set mesh scale factor (default: 1.0)\n");
}

static void draw_callback(Canvas* canvas, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;
    
    // Draw grid
    canvas_draw_grid(5.0f, 1.0f, 0.1f);
    
    // Draw world coordinate frame
    canvas_draw_axis_cross();
    
    // Draw STL mesh
    if (data->mesh) {
        // Set material properties
        GLfloat mat_ambient[] = {0.7f, 0.7f, 0.7f, 1.0f};
        GLfloat mat_diffuse[] = {0.8f, 0.8f, 0.8f, 1.0f};
        GLfloat mat_specular[] = {0.2f, 0.2f, 0.2f, 1.0f};
        GLfloat mat_shininess[] = {50.0f};
        
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
        glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
        
        // Apply scale
        glPushMatrix();
        glScalef(data->scale, data->scale, data->scale);
        
        stl_mesh_draw(data->mesh);
        
        glPopMatrix();
    }
}

static void key_callback(Canvas* canvas, KeySym key, void* user_data) {
    ViewerData* data = (ViewerData*)user_data;
    
    switch (key) {
        case XK_Escape:
            canvas->should_exit = 1;
            break;
            
        case XK_plus:
        case XK_equal:
            // Increase scale
            data->scale *= 1.1f;
            printf("Scale: %.2f\n", data->scale);
            canvas_request_redraw(canvas);
            break;
            
        case XK_minus:
            // Decrease scale
            data->scale /= 1.1f;
            printf("Scale: %.2f\n", data->scale);
            canvas_request_redraw(canvas);
            break;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }
    
    // Create viewer data
    ViewerData data = {0};
    data.scale = 1.0f;  // Default scale
    
    // Parse command line arguments
    const char* stl_file = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--scale") == 0 && i + 1 < argc) {
            data.scale = atof(argv[++i]);
            if (data.scale <= 0) {
                fprintf(stderr, "Scale must be positive\n");
                return 1;
            }
        } else if (!stl_file) {
            stl_file = argv[i];
        } else {
            print_usage(argv[0]);
            return 1;
        }
    }
    
    if (!stl_file) {
        print_usage(argv[0]);
        return 1;
    }
    
    // Load STL mesh
    data.mesh = stl_mesh_create();
    if (!stl_mesh_load(data.mesh, stl_file)) {
        fprintf(stderr, "Failed to load STL file: %s\n", stl_file);
        stl_mesh_destroy(data.mesh);
        return 1;
    }
    
    printf("Initial scale: %.2f\n", data.scale);
    printf("Use '+'/'-' keys to adjust scale\n");
    
    // Create and setup canvas
    Canvas* canvas = canvas_create("STL Viewer", 1000, 600);
    canvas_set_draw_callback(canvas, draw_callback, &data);
    canvas_set_key_callback(canvas, key_callback, &data);
    
    // Run main loop
    canvas_main_loop(canvas);
    
    // Cleanup
    canvas_destroy(canvas);
    stl_mesh_destroy(data.mesh);
    
    return 0;
}