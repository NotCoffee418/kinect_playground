#include <iostream>
#include <GLFW/glfw3.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include "kinect_viewer.h"

int main()
{
    // Initialize GLFW
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow *window = glfwCreateWindow(1280, 720, "Kinect Live Point Cloud - WASD/QE=Move, Drag=Rotate, Scroll=Zoom, R=Reset", NULL, NULL);
    if (!window)
    {
        std::cout << "Failed to create window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    // Setup camera
    CameraState camera;
    init_camera(camera);

    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // Initialize Kinect
    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "No Kinect detected" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();

    libfreenect2::PacketPipeline *pipeline = new libfreenect2::OpenGLPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    if (!dev)
    {
        std::cout << "Failed to open device" << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->start())
    {
        std::cout << "Failed to start device" << std::endl;
        return -1;
    }

    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
    libfreenect2::Registration *registration = new libfreenect2::Registration(ir_params, color_params);

    std::cout << "Kinect started!" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  WASD - Move forward/back/left/right" << std::endl;
    std::cout << "  Q/E - Move down/up" << std::endl;
    std::cout << "  Left click + drag - Rotate view" << std::endl;
    std::cout << "  Scroll wheel - Zoom in/out" << std::endl;
    std::cout << "  R - Reset camera" << std::endl;
    std::cout << "  ESC - Exit" << std::endl;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        libfreenect2::FrameMap frames;

        if (!listener.waitForNewFrame(frames, 1000))
        {
            continue;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setup_camera_view(camera);
        render_point_cloud(depth, rgb, registration);

        listener.release(frames);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    delete registration;
    dev->stop();
    dev->close();
    delete dev;

    glfwTerminate();
    return 0;
}