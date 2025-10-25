#include <iostream>
#include <sys/stat.h>
#include <chrono>
#include <thread>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

#include "kinect_capture.h"

int main()
{
    mkdir("scans", 0755);

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
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (!dev->start())
    {
        std::cout << "Failed to start device" << std::endl;
        return -1;
    }

    // Get camera parameters and create registration
    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
    libfreenect2::Registration *registration = new libfreenect2::Registration(ir_params, color_params);

    int num_scans = 8;
    int delay_seconds = 5;

    std::cout << "Will capture " << num_scans << " scans with " << delay_seconds << " seconds between each." << std::endl;
    std::cout << "Position yourself for the first scan..." << std::endl;

    for (int i = 0; i < num_scans; i++)
    {
        std::cout << "\nCapture " << (i + 1) << "/" << num_scans << " in:" << std::endl;

        for (int countdown = delay_seconds; countdown > 0; countdown--)
        {
            std::cout << countdown << "... " << std::flush;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "CAPTURING!" << std::endl;

        PointCloudData cloud = getPointCloud(dev, listener, registration);

        if (cloud.num_points > 0)
        {
            std::string filename = "scans/scan_" + std::to_string(i) + ".ply";
            savePointCloudPLY(filename.c_str(), cloud);
            freePointCloud(cloud);
        }
        else
        {
            std::cout << "Failed to capture scan " << i << std::endl;
        }

        if (i < num_scans - 1)
        {
            std::cout << "Move to next position..." << std::endl;
        }
    }

    std::cout << "\nDone! Captured " << num_scans << " scans." << std::endl;
    std::cout << "Use CloudCompare or MeshLab to align and merge the point clouds:" << std::endl;
    std::cout << "  1. Import all scan_*.ply files" << std::endl;
    std::cout << "  2. Use ICP (Iterative Closest Point) alignment" << std::endl;
    std::cout << "  3. Merge into single mesh" << std::endl;

    delete registration;
    dev->stop();
    dev->close();
    delete dev;

    return 0;
}