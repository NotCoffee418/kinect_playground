#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <sys/stat.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main()
{
    // Create output directory if it doesn't exist
    mkdir("testframes", 0755);

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = nullptr;

    if (freenect2.enumerateDevices() == 0)
    {
        std::cout << "No Kinect detected" << std::endl;
        return -1;
    }

    std::string serial = freenect2.getDefaultDeviceSerialNumber();
    std::cout << "Opening device: " << serial << std::endl;

    libfreenect2::PacketPipeline *pipeline = new libfreenect2::CpuPacketPipeline();
    dev = freenect2.openDevice(serial, pipeline);

    if (!dev)
    {
        std::cout << "Failed to open device" << std::endl;
        return -1;
    }

    // Add IR to the listener
    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->start())
    {
        std::cout << "Failed to start device" << std::endl;
        return -1;
    }

    std::cout << "Capturing frames..." << std::endl;

    libfreenect2::FrameMap frames;
    int frame_count = 0;

    while (frame_count < 10)
    {
        if (!listener.waitForNewFrame(frames, 10 * 1000))
        {
            std::cout << "Timeout!" << std::endl;
            break;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

        // Save RGB (BGRX -> RGB conversion)
        unsigned char *rgb_data = new unsigned char[rgb->width * rgb->height * 3];
        for (size_t i = 0; i < rgb->width * rgb->height; i++)
        {
            rgb_data[i * 3 + 0] = rgb->data[i * 4 + 2]; // R
            rgb_data[i * 3 + 1] = rgb->data[i * 4 + 1]; // G
            rgb_data[i * 3 + 2] = rgb->data[i * 4 + 0]; // B
        }

        std::string rgb_filename = "testframes/rgb_" + std::to_string(frame_count) + ".png";
        stbi_write_png(rgb_filename.c_str(), rgb->width, rgb->height, 3, rgb_data, rgb->width * 3);
        delete[] rgb_data;

        // Save depth (normalize to 0-255)
        unsigned char *depth_data = new unsigned char[depth->width * depth->height];
        float *depth_float = (float *)depth->data;
        for (size_t i = 0; i < depth->width * depth->height; i++)
        {
            float val = depth_float[i] / 4500.0f; // normalize to 0-4.5m
            depth_data[i] = (unsigned char)(val * 255.0f);
        }

        std::string depth_filename = "testframes/depth_" + std::to_string(frame_count) + ".png";
        stbi_write_png(depth_filename.c_str(), depth->width, depth->height, 1, depth_data, depth->width);
        delete[] depth_data;

        // Save IR (normalize to 0-255)
        unsigned char *ir_data = new unsigned char[ir->width * ir->height];
        float *ir_float = (float *)ir->data;

        // Find max value for normalization
        float max_ir = 0;
        for (size_t i = 0; i < ir->width * ir->height; i++)
        {
            if (ir_float[i] > max_ir)
                max_ir = ir_float[i];
        }

        for (size_t i = 0; i < ir->width * ir->height; i++)
        {
            float val = ir_float[i] / max_ir;
            ir_data[i] = (unsigned char)(val * 255.0f);
        }

        std::string ir_filename = "testframes/ir_" + std::to_string(frame_count) + ".png";
        stbi_write_png(ir_filename.c_str(), ir->width, ir->height, 1, ir_data, ir->width);
        delete[] ir_data;

        std::cout << "Saved frame " << frame_count << " (RGB, Depth, IR)" << std::endl;

        listener.release(frames);
        frame_count++;
    }

    dev->stop();
    dev->close();
    delete dev;

    return 0;
}