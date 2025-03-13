#ifndef CANVAS_H
#define CANVAS_H

#include <X11/Xlib.h>
#include <GL/glx.h>
#include <GL/gl.h>
#include <X11/keysym.h>
#include "math3d.h"  // Add this at the top with other includes

// Forward declare callback types
typedef struct Canvas Canvas;
typedef void (*DrawCallback)(Canvas*, void*);
typedef void (*KeyCallback)(Canvas*, KeySym, void*);

// Canvas struct definition
typedef struct Canvas {
    Display* display;
    Window window;
    GLXContext gl_context;
    int width;
    int height;
    float rotX, rotY, rotZ;
    float scale;
    int isDragging;
    int should_exit;
    int lastX, lastY;
    int activeButton;  // Add this field to track which button is pressed
    
    // Camera
    float camera_distance;  // Distance from target
    float camera_theta;     // Horizontal angle (azimuth)
    float camera_phi;       // Vertical angle (elevation)
    float camera_target[3]; // Look-at point
    
    // Callbacks
    DrawCallback draw_callback;
    KeyCallback key_callback;
    void* user_data;
    
    // Mouse callbacks
    void (*mouse_button_callback)(Canvas*, int, int, int, int, void*);
    void* mouse_button_user_data;
    void (*mouse_motion_callback)(Canvas*, int, int, void*);
    void* mouse_motion_user_data;

    // Add these new fields
    int isPanning;         // Flag for panning mode
    float panX, panY;      // Pan offset
    float lastPanX, lastPanY;  // Last pan position
    int show_axis_labels;  // Toggle for showing axis cross labels
} Canvas;

// Function declarations
Canvas* canvas_create(const char* title, int width, int height);
void canvas_destroy(Canvas* canvas);
void canvas_set_draw_callback(Canvas* canvas, DrawCallback callback, void* user_data);
void canvas_set_key_callback(Canvas* canvas, KeyCallback callback, void* user_data);
void canvas_main_loop(Canvas* canvas);
void canvas_draw_grid(float size, float major_step, float minor_step);
void canvas_draw_axis_cross(void);
void canvas_draw_axis_cross_at(const Vector3* pos, const Matrix3* rot);
float canvas_get_scale(const Canvas* canvas);
void canvas_set_camera_distance(Canvas* canvas, float distance);
void canvas_reset_view(Canvas* canvas);

// Mouse control functions
void canvas_set_mouse_button_callback(Canvas* canvas, 
                                    void (*callback)(Canvas*, int, int, int, int, void*),
                                    void* user_data);

void canvas_set_mouse_motion_callback(Canvas* canvas,
                                    void (*callback)(Canvas*, int, int, void*),
                                    void* user_data);

// Camera control
void canvas_set_camera_position(Canvas* canvas, float x, float y, float z);
void canvas_look_at(Canvas* canvas, float x, float y, float z);

// Add these new functions
void canvas_orbit_camera(Canvas* canvas, float dx, float dy);
void canvas_zoom_camera(Canvas* canvas, float delta);

// Add M_PI definition if not already present
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Add these new function declarations
void canvas_handle_mouse_button(Canvas* canvas, int button, int state, int x, int y);
void canvas_handle_mouse_motion(Canvas* canvas, int x, int y);
void canvas_begin_text_rendering(Canvas* canvas);
void canvas_end_text_rendering(void);
void canvas_draw_text_line(int* y, const char* text);
void canvas_request_redraw(Canvas* canvas);

// Add these function declarations
void canvas_draw_text_2d(int x, int y, const char* text);
void canvas_draw_text_3d(float x, float y, float z, const char* text);

// Text and overlay functions
void canvas_begin_2d_overlay(Canvas* canvas);
void canvas_end_2d_overlay(void);
void canvas_draw_text_at(int x, int y, const char* text);

// Camera functions
void canvas_set_default_camera(Canvas* canvas);

// Add these new constants
#define SCROLL_UP 4
#define SCROLL_DOWN 5

// Add these new function declarations
void canvas_handle_mouse_scroll(Canvas* canvas, int button);
void canvas_pan_camera(Canvas* canvas, float dx, float dy);
void canvas_draw_axis_cross_with_label(const Vector3* pos, const Matrix3* rot, const char* label);
void canvas_set_show_axis_labels(Canvas* canvas, int show);
int canvas_get_show_axis_labels(const Canvas* canvas);

#endif // CANVAS_H 