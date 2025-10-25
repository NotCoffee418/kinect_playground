#include <iostream>
#include <sys/stat.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "kinect_capture.h"

int main()
{
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

    for (int i = 0; i < 10; i++)
    {
        FrameCapture capture = getFrame(dev, listener);

        if (capture.rgb_data == nullptr)
        {
            std::cout << "Failed to capture frame " << i << std::endl;
            continue;
        }

        // Save frames
        std::string rgb_filename = "testframes/rgb_" + std::to_string(i) + ".png";
        std::string depth_filename = "testframes/depth_" + std::to_string(i) + ".png";
        std::string ir_filename = "testframes/ir_" + std::to_string(i) + ".png";

        stbi_write_png(rgb_filename.c_str(), capture.rgb_width, capture.rgb_height, 3,
                       capture.rgb_data, capture.rgb_width * 3);
        stbi_write_png(depth_filename.c_str(), capture.depth_width, capture.depth_height, 1,
                       capture.depth_data, capture.depth_width);
        stbi_write_png(ir_filename.c_str(), capture.ir_width, capture.ir_height, 1,
                       capture.ir_data, capture.ir_width);

        std::cout << "Saved frame " << i << std::endl;

        freeFrameCapture(capture);
    }

    dev->stop();
    dev->close();
    delete dev;

    return 0;
}