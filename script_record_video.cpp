#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>

#include "kinect_capture.h"

void writeRawVideo(const char *filename, int width, int height, int channels, int num_frames,
                   libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener,
                   int frame_type)
{
    std::ofstream file(filename, std::ios::binary);

    // Write header info (width, height, channels, num_frames)
    file.write((char *)&width, sizeof(int));
    file.write((char *)&height, sizeof(int));
    file.write((char *)&channels, sizeof(int));
    file.write((char *)&num_frames, sizeof(int));

    for (int i = 0; i < num_frames; i++)
    {
        if (frame_type == 0)
        { // RGB
            RGBFrame frame = getRGBFrame(dev, listener);
            if (frame.data)
            {
                file.write((char *)frame.data, width * height * channels);
                freeRGBFrame(frame);
                std::cout << "Captured RGB frame " << i << "/" << num_frames << "\r" << std::flush;
            }
        }
        else if (frame_type == 1)
        { // Depth
            DepthFrame frame = getDepthFrame(dev, listener);
            if (frame.data)
            {
                file.write((char *)frame.data, width * height * channels);
                freeDepthFrame(frame);
                std::cout << "Captured depth frame " << i << "/" << num_frames << "\r" << std::flush;
            }
        }
        else if (frame_type == 2)
        { // IR
            IRFrame frame = getIRFrame(dev, listener);
            if (frame.data)
            {
                file.write((char *)frame.data, width * height * channels);
                freeIRFrame(frame);
                std::cout << "Captured IR frame " << i << "/" << num_frames << "\r" << std::flush;
            }
        }
    }

    file.close();
    std::cout << std::endl;
}

int main()
{
    mkdir("videos", 0755);

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

    std::cout << "Recording 300 frames (~10 seconds at 30fps)..." << std::endl;

    // Record RGB video
    std::cout << "Recording RGB..." << std::endl;
    writeRawVideo("videos/rgb_video.raw", 1920, 1080, 3, 300, dev, listener, 0);

    // Record depth video
    std::cout << "Recording depth..." << std::endl;
    writeRawVideo("videos/depth_video.raw", 512, 424, 1, 300, dev, listener, 1);

    // Record IR video
    std::cout << "Recording IR..." << std::endl;
    writeRawVideo("videos/ir_video.raw", 512, 424, 1, 300, dev, listener, 2);

    std::cout << "Done! Use FFmpeg to convert to playable video:" << std::endl;
    std::cout << "ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1920x1080 -framerate 30 -i videos/rgb_video.raw videos/rgb_output.mp4" << std::endl;
    std::cout << "ffmpeg -f rawvideo -pixel_format gray -video_size 512x424 -framerate 30 -i videos/depth_video.raw videos/depth_output.mp4" << std::endl;
    std::cout << "ffmpeg -f rawvideo -pixel_format gray -video_size 512x424 -framerate 30 -i videos/ir_video.raw videos/ir_output.mp4" << std::endl;

    dev->stop();
    dev->close();
    delete dev;

    return 0;
}