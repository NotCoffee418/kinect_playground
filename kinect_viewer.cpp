#include "kinect_viewer.h"
#include <iostream>
#include <cmath>

// Global camera state (for GLFW callbacks)
static CameraState *g_camera = nullptr;

void init_camera(CameraState &cam)
{
    cam.distance = 3.0f;
    cam.angle_x = -20.0f;
    cam.angle_y = 180.0f;
    cam.pos_x = 0.0f;
    cam.pos_y = 0.0f;
    cam.pos_z = 0.0f;
    cam.last_mouse_x = 0;
    cam.last_mouse_y = 0;
    cam.mouse_dragging = false;
    g_camera = &cam;
}

void reset_camera(CameraState &cam)
{
    cam.distance = 3.0f;
    cam.angle_x = -20.0f;
    cam.angle_y = 180.0f;
    cam.pos_x = 0.0f;
    cam.pos_y = 0.0f;
    cam.pos_z = 0.0f;
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (!g_camera)
        return;

    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            g_camera->mouse_dragging = true;
            glfwGetCursorPos(window, &g_camera->last_mouse_x, &g_camera->last_mouse_y);
        }
        else if (action == GLFW_RELEASE)
        {
            g_camera->mouse_dragging = false;
        }
    }
}

void cursor_position_callback(GLFWwindow *window, double xpos, double ypos)
{
    if (!g_camera || !g_camera->mouse_dragging)
        return;

    double dx = xpos - g_camera->last_mouse_x;
    double dy = ypos - g_camera->last_mouse_y;

    g_camera->angle_x += dy * 0.5f;
    g_camera->angle_y += dx * 0.5f;

    g_camera->last_mouse_x = xpos;
    g_camera->last_mouse_y = ypos;
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
    if (!g_camera)
        return;

    g_camera->distance -= yoffset * 0.5f;
    if (g_camera->distance < 0.1f)
        g_camera->distance = 0.1f;
    if (g_camera->distance > 20.0f)
        g_camera->distance = 20.0f;
}

void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (!g_camera)
        return;

    float move_speed = 0.1f;

    if (action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        if (key == GLFW_KEY_R)
        {
            reset_camera(*g_camera);
            std::cout << "Camera reset" << std::endl;
        }
        if (key == GLFW_KEY_ESCAPE)
        {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (key == GLFW_KEY_W)
        {
            g_camera->pos_z += move_speed;
        }
        if (key == GLFW_KEY_S)
        {
            g_camera->pos_z -= move_speed;
        }
        if (key == GLFW_KEY_A)
        {
            g_camera->pos_x += move_speed;
        }
        if (key == GLFW_KEY_D)
        {
            g_camera->pos_x -= move_speed;
        }
        if (key == GLFW_KEY_Q)
        {
            g_camera->pos_y -= move_speed;
        }
        if (key == GLFW_KEY_E)
        {
            g_camera->pos_y += move_speed;
        }
    }
}

void setup_camera_view(const CameraState &cam)
{
    // Get current window size
    int width, height;
    GLFWwindow *window = glfwGetCurrentContext();
    glfwGetFramebufferSize(window, &width, &height);

    // Update viewport to match window size
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = (float)width / (float)height;
    float fov = 45.0f;
    float near = 0.01f;
    float far = 100.0f;
    float top = near * tanf(fov * 0.5f * M_PI / 180.0f);
    float right = top * aspect;
    glFrustum(-right, right, -top, top, near, far);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(cam.pos_x, cam.pos_y, -cam.distance + cam.pos_z);
    glRotatef(cam.angle_x, 1, 0, 0);
    glRotatef(cam.angle_y, 0, 1, 0);
    glRotatef(180, 0, 0, 1);
}

void render_point_cloud(libfreenect2::Frame *depth, libfreenect2::Frame *rgb,
                        libfreenect2::Registration *registration)
{
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    registration->apply(rgb, depth, &undistorted, &registered);

    glBegin(GL_POINTS);

    float *depth_data = (float *)undistorted.data;
    unsigned char *rgb_data = registered.data;

    for (int y = 0; y < 424; y += 1)
    {
        for (int x = 0; x < 512; x += 1)
        {
            int idx = y * 512 + x;
            float d = depth_data[idx];

            if (d > 0 && d < 4500)
            {
                float px, py, pz;
                registration->getPointXYZ(&undistorted, y, x, px, py, pz);

                float r = rgb_data[idx * 4 + 2] / 255.0f;
                float g = rgb_data[idx * 4 + 1] / 255.0f;
                float b = rgb_data[idx * 4 + 0] / 255.0f;

                glColor3f(r, g, b);
                glVertex3f(px, py, pz);
            }
        }
    }

    glEnd();
}