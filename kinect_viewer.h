#ifndef KINECT_VIEWER_H
#define KINECT_VIEWER_H

#include <GLFW/glfw3.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>

// Camera state
struct CameraState
{
    float distance;
    float angle_x;
    float angle_y;
    float pos_x;
    float pos_y;
    float pos_z;
    double last_mouse_x;
    double last_mouse_y;
    bool mouse_dragging;
};

// Initialize camera with default values
void init_camera(CameraState &cam);

// Reset camera to default view
void reset_camera(CameraState &cam);

// GLFW callbacks
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);

// Rendering
void setup_camera_view(const CameraState &cam);
void render_point_cloud(libfreenect2::Frame *depth, libfreenect2::Frame *rgb,
                        libfreenect2::Registration *registration);

#endif