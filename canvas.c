#include "canvas.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <GL/freeglut.h>  // Use FreeGLUT instead of GLUT

#define MAX_CANVASES 16
static Canvas* all_canvases[MAX_CANVASES] = {0};

// Forward declare setup_view
static void setup_view(Canvas* canvas);

static void* malloc_safe(size_t size) {
    void* ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(1);
    }
    memset(ptr, 0, size);
    return ptr;
}

static void canvas_init_gl(Canvas* canvas) {
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // Dark background
    
    // Set up perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)canvas->width / (float)canvas->height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    
    // Set up lighting
    GLfloat light_position[] = { 1.0f, 1.0f, 1.0f, 0.0f };
    GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat light_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
}

void canvas_draw_axis_cross(void) {
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);  // Make lines thicker
    
    glBegin(GL_LINES);
    // X axis - Red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);  // Make axes longer (1m)
    
    // Y axis - Green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);
    
    // Z axis - Blue (up)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);
    glEnd();
    
    glEnable(GL_LIGHTING);
}

void canvas_draw_axis_cross_at(const Vector3* pos, const Matrix3* rot) {
    glPushMatrix();
    
    // Apply position
    glTranslatef(pos->x, pos->y, pos->z);
    
    // Apply rotation
    GLfloat rot_matrix[16] = {
        rot->data[0], rot->data[3], rot->data[6], 0,  // First column
        rot->data[1], rot->data[4], rot->data[7], 0,  // Second column
        rot->data[2], rot->data[5], rot->data[8], 0,  // Third column
        0, 0, 0, 1                                     // Fourth column
    };
    glMultMatrixf(rot_matrix);
    
    // Draw the axes
    canvas_draw_axis_cross();
    
    glPopMatrix();
}

void canvas_draw_axis_cross_with_label(const Vector3* pos, const Matrix3* rot, const char* label) {
    // Get the current canvas instance
    Canvas* current_canvas = NULL;
    // Get the current window ID
    Window current_window = glXGetCurrentDrawable();
    
    // Find the canvas instance for this window
    for (int i = 0; i < MAX_CANVASES; i++) {
        if (all_canvases[i] && all_canvases[i]->window == current_window) {
            current_canvas = all_canvases[i];
            break;
        }
    }
    
    // First draw the axis cross
    canvas_draw_axis_cross_at(pos, rot);
    
    // Only draw label if labels are enabled
    if (label && current_canvas && canvas_get_show_axis_labels(current_canvas)) {
        // Draw the label slightly offset from the axis cross
        glPushMatrix();
        glTranslatef(pos->x, pos->y, pos->z);
        
        // Get the current modelview matrix to calculate screen position
        GLfloat mv[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, mv);
        
        // Calculate offset in object space (offset in positive X and Y)
        float offset = 0.1f;  // Offset distance
        canvas_draw_text_3d(offset, offset, offset, label);
        
        glPopMatrix();
    }
}

void canvas_draw_grid(float size, float major_step, float minor_step) {
    glDisable(GL_LIGHTING);
    glLineWidth(1.0f);
    
    glBegin(GL_LINES);
    
    // Draw minor grid lines
    glColor3f(0.2f, 0.2f, 0.2f);
    for (float y = -size; y <= size + 0.01f; y += minor_step) {
        if (fabs(fmod(y, major_step)) < 0.01f) continue;
        glVertex3f(-size, y, 0.0f);
        glVertex3f(size, y, 0.0f);
    }
    for (float x = -size; x <= size + 0.01f; x += minor_step) {
        if (fabs(fmod(x, major_step)) < 0.01f) continue;
        glVertex3f(x, -size, 0.0f);
        glVertex3f(x, size, 0.0f);
    }
    
    // Draw major grid lines
    glColor3f(0.4f, 0.4f, 0.4f);
    for (float y = -size; y <= size + 0.01f; y += major_step) {
        glVertex3f(-size, y, 0.0f);
        glVertex3f(size, y, 0.0f);
    }
    for (float x = -size; x <= size + 0.01f; x += major_step) {
        glVertex3f(x, -size, 0.0f);
        glVertex3f(x, size, 0.0f);
    }
    
    glEnd();
    glEnable(GL_LIGHTING);
}

// Add near the top with other static functions
static void update_camera_position(Canvas* canvas) {
    // Calculate camera position in spherical coordinates relative to target
    float temp_x = canvas->camera_distance * cos(canvas->camera_phi) * cos(canvas->camera_theta);
    float temp_y = canvas->camera_distance * cos(canvas->camera_phi) * sin(canvas->camera_theta);
    float temp_z = canvas->camera_distance * sin(canvas->camera_phi);
    
    // Update the view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(canvas->camera_target[0] + temp_x,  // Camera position = target + offset
              canvas->camera_target[1] + temp_y, 
              canvas->camera_target[2] + temp_z,
              canvas->camera_target[0],           // Look at target
              canvas->camera_target[1], 
              canvas->camera_target[2],
              0.0f, 0.0f, 1.0f);                 // Up vector (Z is up)
}

// Add these new functions
void canvas_orbit_camera(Canvas* canvas, float dx, float dy) {
    canvas->camera_theta += dx * 0.01f;
    canvas->camera_phi += dy * 0.01f;
    
    // Clamp vertical angle to avoid gimbal lock
    canvas->camera_phi = fmax(fmin(canvas->camera_phi, M_PI/2 - 0.1f), -M_PI/2 + 0.1f);
    
    update_camera_position(canvas);
}

void canvas_zoom_camera(Canvas* canvas, float delta) {
    canvas->camera_distance *= (1.0f + delta * 0.1f);
    canvas->camera_distance = fmax(fmin(canvas->camera_distance, 20.0f), 0.1f);
    update_camera_position(canvas);
}

// Add near the top with other static functions
static void create_font(void) {
    // Create display lists for first 128 ASCII characters
    for (int i = 32; i < 127; i++) {
        glNewList(i + 128, GL_COMPILE);
        // Just draw a simple rectangle for each character
        glBegin(GL_QUADS);
        glVertex2f(0.0f, 0.0f);
        glVertex2f(8.0f, 0.0f);
        glVertex2f(8.0f, 13.0f);
        glVertex2f(0.0f, 13.0f);
        glEnd();
        glTranslatef(10.0f, 0.0f, 0.0f);
        glEndList();
    }
}

// Add this function to begin 2D text rendering mode
void canvas_begin_text_rendering(Canvas* canvas) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas->width, canvas->height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    // Disable lighting and depth test for 2D text
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    
    glColor3f(1.0f, 1.0f, 1.0f);  // White text
}

// Add this function to end 2D text rendering mode
void canvas_end_text_rendering(void) {
    // Restore 3D state
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

// Add this function to draw a line of text
void canvas_draw_text_line(int* y, const char* text) {
    glRasterPos2i(10, *y);
    for (const char* c = text; *c != '\0'; c++) {
        glCallList(128 + *c);
    }
    *y += 15;
}

// Remove static from canvas_draw_text_at
void canvas_draw_text_at(int x, int y, const char* text) {
    glRasterPos2i(x, y);
    for (const char* c = text; *c != '\0'; c++) {
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
    }
}

// Add function to begin/end text overlay
void canvas_begin_2d_overlay(Canvas* canvas) {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, canvas->width, canvas->height, 0, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glColor3f(1.0f, 1.0f, 1.0f);  // White text
}

void canvas_end_2d_overlay(void) {
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

Canvas* canvas_create(const char* title, int width, int height) {
    Canvas* canvas = (Canvas*)malloc_safe(sizeof(Canvas));
    
    canvas->display = XOpenDisplay(NULL);
    if (!canvas->display) {
        fprintf(stderr, "Cannot open display\n");
        exit(1);
    }
    
    int screen = DefaultScreen(canvas->display);
    
    static int visual_attribs[] = {
        GLX_RGBA,
        GLX_DEPTH_SIZE, 24,
        GLX_DOUBLEBUFFER,
        None
    };
    
    XVisualInfo* vi = glXChooseVisual(canvas->display, screen, visual_attribs);
    if (!vi) {
        fprintf(stderr, "No appropriate visual found\n");
        exit(1);
    }
    
    Colormap cmap = XCreateColormap(canvas->display, RootWindow(canvas->display, screen),
                                  vi->visual, AllocNone);
    
    XSetWindowAttributes swa;
    swa.colormap = cmap;
    swa.event_mask = ExposureMask | KeyPressMask | ButtonPressMask |
                     ButtonReleaseMask | PointerMotionMask | StructureNotifyMask;
    
    canvas->width = width;
    canvas->height = height;
    canvas->scale = 1.0f;
    canvas->camera_distance = 5.0f;
    canvas->camera_theta = 0.0f;
    canvas->camera_phi = 0.0f;
    canvas->show_axis_labels = 1;  // Default to showing labels
    
    // Store canvas in global array
    for (int i = 0; i < MAX_CANVASES; i++) {
        if (!all_canvases[i]) {
            all_canvases[i] = canvas;
            break;
        }
    }
    
    canvas->window = XCreateWindow(canvas->display, RootWindow(canvas->display, screen),
                                 0, 0, width, height, 0, vi->depth, InputOutput,
                                 vi->visual, CWColormap | CWEventMask, &swa);
    
    XMapWindow(canvas->display, canvas->window);
    XStoreName(canvas->display, canvas->window, title);
    
    canvas->gl_context = glXCreateContext(canvas->display, vi, NULL, GL_TRUE);
    glXMakeCurrent(canvas->display, canvas->window, canvas->gl_context);
    
    canvas_init_gl(canvas);
    create_font();  // Add this line
    
    canvas->rotX = 0;
    canvas->rotY = 0;
    canvas->rotZ = 0;
    canvas->isDragging = 0;
    canvas->should_exit = 0;
    
    // Initialize camera
    canvas->camera_distance = 5.0f;
    canvas->camera_theta = M_PI/4;  // 45 degrees
    canvas->camera_phi = M_PI/6;    // 30 degrees
    canvas->camera_target[0] = 0.0f;
    canvas->camera_target[1] = 0.0f;
    canvas->camera_target[2] = 0.0f;
    
    update_camera_position(canvas);
    
    canvas->isPanning = 0;
    canvas->panX = 0.0f;
    canvas->panY = 0.0f;
    canvas->lastPanX = 0.0f;
    canvas->lastPanY = 0.0f;
    
    return canvas;
}

void canvas_destroy(Canvas* canvas) {
    if (!canvas) return;
    
    // Remove from global array
    for (int i = 0; i < MAX_CANVASES; i++) {
        if (all_canvases[i] == canvas) {
            all_canvases[i] = NULL;
            break;
        }
    }
    
    if (canvas->display) {
        if (canvas->gl_context) {
            glXMakeCurrent(canvas->display, None, NULL);
            glXDestroyContext(canvas->display, canvas->gl_context);
        }
        if (canvas->window) {
            XDestroyWindow(canvas->display, canvas->window);
        }
        XCloseDisplay(canvas->display);
    }
    free(canvas);
}

void canvas_set_draw_callback(Canvas* canvas, DrawCallback callback, void* user_data) {
    canvas->draw_callback = callback;
    canvas->user_data = user_data;
}

void canvas_set_key_callback(Canvas* canvas, KeyCallback callback, void* user_data) {
    canvas->key_callback = callback;
    canvas->user_data = user_data;
}

void canvas_main_loop(Canvas* canvas) {
    XEvent event;
    
    while (!canvas->should_exit) {
        while (XPending(canvas->display)) {
            XNextEvent(canvas->display, &event);
            
            switch (event.type) {
                case ButtonPress:
                case ButtonRelease: {
                    XButtonEvent* button_event = (XButtonEvent*)&event;
                    if (canvas->mouse_button_callback) {
                        canvas->mouse_button_callback(canvas, 
                                                    button_event->button,
                                                    event.type == ButtonPress,
                                                    button_event->x,
                                                    button_event->y,
                                                    canvas->mouse_button_user_data);
                    }
                    break;
                }
                case MotionNotify: {
                    XMotionEvent* motion = (XMotionEvent*)&event;
                    if (canvas->mouse_motion_callback) {
                        canvas->mouse_motion_callback(canvas,
                                                    motion->x,
                                                    motion->y,
                                                    canvas->mouse_motion_user_data);
                    }
                    break;
                }
                case KeyPress:
                    if (canvas->key_callback) {
                        KeySym key = XLookupKeysym(&event.xkey, 0);
                        canvas->key_callback(canvas, key, canvas->user_data);
                    }
                    break;
            }
        }
        
        if (canvas->draw_callback) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            
            // Set up view matrix with camera position and rotation
            setup_view(canvas);
            
            canvas->draw_callback(canvas, canvas->user_data);
            
            glXSwapBuffers(canvas->display, canvas->window);
        }
        
        usleep(16666);  // ~60 FPS
    }
}

float canvas_get_scale(const Canvas* canvas) {
    return canvas->scale;
}

void canvas_set_camera_distance(Canvas* canvas, float distance) {
    canvas->scale = distance / 3.0f;  // Normalize to our default -3.0f distance
}

void canvas_draw_text_2d(int x, int y, const char* text) {
    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0, 1000, 600, 0, -1, 1);  // Match window size, origin at top-left
    
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    
    glColor3f(1.0f, 1.0f, 1.0f);  // White text
    glRasterPos2i(x, y);
    
    for (const char* p = text; *p; p++) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *p);  // Changed font
    }
    
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING);
}

void canvas_draw_text_3d(float x, float y, float z, const char* text) {
    glDisable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f);  // White text
    
    glRasterPos3f(x, y, z);
    for (const char* p = text; *p; p++) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *p);  // Changed font
    }
    
    glEnable(GL_LIGHTING);
}

void canvas_reset_view(Canvas* canvas) {
    canvas->camera_distance = 5.0f;
    canvas->camera_theta = M_PI/4;    // 45 degrees
    canvas->camera_phi = M_PI/6;      // 30 degrees
    canvas->camera_target[0] = 0.0f;  // Reset target to origin
    canvas->camera_target[1] = 0.0f;
    canvas->camera_target[2] = 0.0f;
    update_camera_position(canvas);
}

void canvas_start_rotation(Canvas* canvas, int x, int y) {
    if (!canvas) return;
    canvas->isDragging = 1;
    canvas->lastX = x;
    canvas->lastY = y;
}

void canvas_stop_rotation(Canvas* canvas) {
    if (!canvas) return;
    canvas->isDragging = 0;
}

void canvas_start_pan(Canvas* canvas, int x, int y) {
    if (!canvas) return;
    canvas->isDragging = 2;  // Use 2 for panning mode
    canvas->lastX = x;
    canvas->lastY = y;
}

void canvas_stop_pan(Canvas* canvas) {
    if (!canvas) return;
    canvas->isDragging = 0;
}

void canvas_set_mouse_button_callback(Canvas* canvas, 
                                    void (*callback)(Canvas*, int, int, int, int, void*),
                                    void* user_data) {
    canvas->mouse_button_callback = callback;
    canvas->mouse_button_user_data = user_data;
}

void canvas_set_mouse_motion_callback(Canvas* canvas,
                                    void (*callback)(Canvas*, int, int, void*),
                                    void* user_data) {
    canvas->mouse_motion_callback = callback;
    canvas->mouse_motion_user_data = user_data;
}

void canvas_set_camera_position(Canvas* canvas, float x, float y, float z) {
    // Convert to spherical coordinates
    float r = sqrt(x*x + y*y + z*z);
    canvas->camera_distance = r;
    canvas->camera_phi = asin(z/r);
    canvas->camera_theta = atan2(y, x);
    update_camera_position(canvas);
}

// Update setup_view function
static void setup_view(Canvas* canvas) {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (float)canvas->width / (float)canvas->height, 0.1f, 100.0f);
    
    // Update camera position and view matrix
    update_camera_position(canvas);
}

// Add these near the top with other static functions
static void canvas_handle_mouse_drag(Canvas* canvas, int button, double dx, double dy) {
    if (button == Button1) {  // Left button - orbit
        canvas_orbit_camera(canvas, dx, dy);
    } else if (button == Button3) {  // Right button - zoom
        canvas_zoom_camera(canvas, -dy * 0.01f);
    }
}

// Add this function to handle mouse button events
void canvas_handle_mouse_button(Canvas* canvas, int button, int state, int x, int y) {
    if (state == 1) {  // Press
        canvas->lastX = x;
        canvas->lastY = y;
        
        if (button == Button1) {  // Left button - orbit
            canvas->isDragging = 1;
            canvas->activeButton = button;
        } else if (button == Button3) {  // Right button - pan
            canvas->isPanning = 1;
            canvas->activeButton = button;
            canvas->lastPanX = x;
            canvas->lastPanY = y;
        } else if (button == SCROLL_UP || button == SCROLL_DOWN) {
            canvas_handle_mouse_scroll(canvas, button);
        }
    } else {  // Release
        if (button == Button1) {
            canvas->isDragging = 0;
        } else if (button == Button3) {
            canvas->isPanning = 0;
        }
    }
}

// Add this function to handle mouse motion
void canvas_handle_mouse_motion(Canvas* canvas, int x, int y) {
    if (canvas->isDragging) {
        double dx = x - canvas->lastX;
        double dy = y - canvas->lastY;
        
        canvas_orbit_camera(canvas, dx, dy);
        
        canvas->lastX = x;
        canvas->lastY = y;
    } else if (canvas->isPanning) {
        double dx = x - canvas->lastPanX;
        double dy = y - canvas->lastPanY;
        
        canvas_pan_camera(canvas, dx, dy);
        
        canvas->lastPanX = x;
        canvas->lastPanY = y;
    }
}

// Update canvas_set_default_camera to use the canvas parameter
void canvas_set_default_camera(Canvas* canvas) {
    // Set up camera in spherical coordinates
    canvas->camera_distance = 5.0f;  // Distance from origin
    canvas->camera_theta = M_PI/4;   // 45 degrees azimuth (around Z)
    canvas->camera_phi = M_PI/6;     // 30 degrees elevation
    
    // Look at origin
    canvas->camera_target[0] = 0.0f;
    canvas->camera_target[1] = 0.0f;
    canvas->camera_target[2] = 0.0f;
    
    // Update the view
    update_camera_position(canvas);
}

// Add this function to handle mouse scroll
void canvas_handle_mouse_scroll(Canvas* canvas, int button) {
    if (button == SCROLL_UP) {
        canvas_zoom_camera(canvas, -0.1f);  // Zoom in
    } else if (button == SCROLL_DOWN) {
        canvas_zoom_camera(canvas, 0.1f);   // Zoom out
    }
}

// Update the pan_camera function to properly handle camera orientation
void canvas_pan_camera(Canvas* canvas, float dx, float dy) {
    // Convert screen coordinates to world space movement
    float scale = 0.005f * canvas->camera_distance;  // Scale pan by distance
    
    // Calculate right and up vectors based on current camera orientation
    float right_x = -sin(canvas->camera_theta);
    float right_y = cos(canvas->camera_theta);
    float up_x = -cos(canvas->camera_theta) * sin(canvas->camera_phi);
    float up_y = -sin(canvas->camera_theta) * sin(canvas->camera_phi);
    float up_z = cos(canvas->camera_phi);
    
    // Update camera target using right and up vectors
    canvas->camera_target[0] += (-dx * right_x + dy * up_x) * scale;
    canvas->camera_target[1] += (-dx * right_y + dy * up_y) * scale;
    canvas->camera_target[2] += dy * up_z * scale;
    
    update_camera_position(canvas);
}

void canvas_set_show_axis_labels(Canvas* canvas, int show) {
    if (canvas) {
        canvas->show_axis_labels = show;
    }
}

int canvas_get_show_axis_labels(const Canvas* canvas) {
    if (canvas) {
        return canvas->show_axis_labels;
    }
    return 1;  // Default to showing labels if no canvas provided
}

void canvas_request_redraw(Canvas* canvas) {
    if (canvas) {
        // Trigger a redraw by sending an Expose event
        XEvent event;
        memset(&event, 0, sizeof(event));
        event.type = Expose;
        event.xexpose.window = canvas->window;
        XSendEvent(canvas->display, canvas->window, False, ExposureMask, &event);
        XFlush(canvas->display);
    }
}