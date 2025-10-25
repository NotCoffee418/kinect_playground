#include "kinect_capture.h"
#include <iostream>
#include <fstream>

FrameCapture getFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener)
{
    libfreenect2::FrameMap frames;
    FrameCapture capture = {};

    if (!listener.waitForNewFrame(frames, 10 * 1000))
    {
        std::cout << "Timeout waiting for frames!" << std::endl;
        return capture;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

    // Convert RGB (BGRX -> RGB)
    capture.rgb_width = rgb->width;
    capture.rgb_height = rgb->height;
    capture.rgb_data = new unsigned char[rgb->width * rgb->height * 3];
    for (size_t i = 0; i < rgb->width * rgb->height; i++)
    {
        capture.rgb_data[i * 3 + 0] = rgb->data[i * 4 + 2]; // R
        capture.rgb_data[i * 3 + 1] = rgb->data[i * 4 + 1]; // G
        capture.rgb_data[i * 3 + 2] = rgb->data[i * 4 + 0]; // B
    }

    // Convert depth (normalize to 0-255)
    capture.depth_width = depth->width;
    capture.depth_height = depth->height;
    capture.depth_data = new unsigned char[depth->width * depth->height];
    float *depth_float = (float *)depth->data;
    for (size_t i = 0; i < depth->width * depth->height; i++)
    {
        float val = depth_float[i] / 4500.0f; // normalize to 0-4.5m
        capture.depth_data[i] = (unsigned char)(val * 255.0f);
    }

    // Convert IR (normalize to 0-255)
    capture.ir_width = ir->width;
    capture.ir_height = ir->height;
    capture.ir_data = new unsigned char[ir->width * ir->height];
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
        capture.ir_data[i] = (unsigned char)(val * 255.0f);
    }

    listener.release(frames);
    return capture;
}

void freeFrameCapture(FrameCapture &capture)
{
    delete[] capture.rgb_data;
    delete[] capture.depth_data;
    delete[] capture.ir_data;
}

RGBFrame getRGBFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener)
{
    libfreenect2::FrameMap frames;
    RGBFrame frame = {};

    if (!listener.waitForNewFrame(frames, 10 * 1000))
    {
        std::cout << "Timeout waiting for RGB frame!" << std::endl;
        return frame;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];

    frame.width = rgb->width;
    frame.height = rgb->height;
    frame.data = new unsigned char[rgb->width * rgb->height * 3];
    for (size_t i = 0; i < rgb->width * rgb->height; i++)
    {
        frame.data[i * 3 + 0] = rgb->data[i * 4 + 2]; // R
        frame.data[i * 3 + 1] = rgb->data[i * 4 + 1]; // G
        frame.data[i * 3 + 2] = rgb->data[i * 4 + 0]; // B
    }

    listener.release(frames);
    return frame;
}

DepthFrame getDepthFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener)
{
    libfreenect2::FrameMap frames;
    DepthFrame frame = {};

    if (!listener.waitForNewFrame(frames, 10 * 1000))
    {
        std::cout << "Timeout waiting for depth frame!" << std::endl;
        return frame;
    }

    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    frame.width = depth->width;
    frame.height = depth->height;
    frame.data = new unsigned char[depth->width * depth->height];
    float *depth_float = (float *)depth->data;
    for (size_t i = 0; i < depth->width * depth->height; i++)
    {
        float val = depth_float[i] / 4500.0f;
        frame.data[i] = (unsigned char)(val * 255.0f);
    }

    listener.release(frames);
    return frame;
}

IRFrame getIRFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener)
{
    libfreenect2::FrameMap frames;
    IRFrame frame = {};

    if (!listener.waitForNewFrame(frames, 10 * 1000))
    {
        std::cout << "Timeout waiting for IR frame!" << std::endl;
        return frame;
    }

    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];

    frame.width = ir->width;
    frame.height = ir->height;
    frame.data = new unsigned char[ir->width * ir->height];
    float *ir_float = (float *)ir->data;

    float max_ir = 0;
    for (size_t i = 0; i < ir->width * ir->height; i++)
    {
        if (ir_float[i] > max_ir)
            max_ir = ir_float[i];
    }

    for (size_t i = 0; i < ir->width * ir->height; i++)
    {
        float val = ir_float[i] / max_ir;
        frame.data[i] = (unsigned char)(val * 255.0f);
    }

    listener.release(frames);
    return frame;
}

void freeRGBFrame(RGBFrame &frame)
{
    delete[] frame.data;
}

void freeDepthFrame(DepthFrame &frame)
{
    delete[] frame.data;
}

void freeIRFrame(IRFrame &frame)
{
    delete[] frame.data;
}

PointCloudData getPointCloud(libfreenect2::Freenect2Device *dev,
                             libfreenect2::SyncMultiFrameListener &listener,
                             libfreenect2::Registration *registration)
{
    libfreenect2::FrameMap frames;
    PointCloudData cloud = {};

    if (!listener.waitForNewFrame(frames, 10 * 1000))
    {
        std::cout << "Timeout waiting for frames!" << std::endl;
        return cloud;
    }

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    // Create undistorted and registered frames
    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);

    registration->apply(rgb, depth, &undistorted, &registered);

    // Count valid points (depth > 0)
    float *depth_data = (float *)undistorted.data;
    int valid_count = 0;
    for (int i = 0; i < 512 * 424; i++)
    {
        if (depth_data[i] > 0 && depth_data[i] < 4500)
        {
            valid_count++;
        }
    }

    cloud.num_points = valid_count;
    cloud.points = new float[valid_count * 3];
    cloud.colors = new unsigned char[valid_count * 3];

    int point_idx = 0;
    for (int y = 0; y < 424; y++)
    {
        for (int x = 0; x < 512; x++)
        {
            int idx = y * 512 + x;
            float d = depth_data[idx];

            // Skip invalid depth
            if (d <= 0 || d >= 4500)
                continue;

            // Get 3D point using registration
            float px, py, pz;
            registration->getPointXYZ(&undistorted, y, x, px, py, pz);

            cloud.points[point_idx * 3 + 0] = px;
            cloud.points[point_idx * 3 + 1] = py;
            cloud.points[point_idx * 3 + 2] = pz;

            // Get color from registered RGB frame
            unsigned char *rgb_data = registered.data;
            cloud.colors[point_idx * 3 + 0] = rgb_data[idx * 4 + 2]; // R
            cloud.colors[point_idx * 3 + 1] = rgb_data[idx * 4 + 1]; // G
            cloud.colors[point_idx * 3 + 2] = rgb_data[idx * 4 + 0]; // B

            point_idx++;
        }
    }

    listener.release(frames);
    return cloud;
}

void freePointCloud(PointCloudData &cloud)
{
    delete[] cloud.points;
    delete[] cloud.colors;
}

void savePointCloudPLY(const char *filename, const PointCloudData &cloud)
{
    std::ofstream file(filename, std::ios::binary);

    // Write PLY header
    std::string header =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex " +
        std::to_string(cloud.num_points) + "\n"
                                           "property float x\n"
                                           "property float y\n"
                                           "property float z\n"
                                           "property uchar red\n"
                                           "property uchar green\n"
                                           "property uchar blue\n"
                                           "end_header\n";

    file.write(header.c_str(), header.size());

    // Write points in binary
    for (int i = 0; i < cloud.num_points; i++)
    {
        float x = cloud.points[i * 3 + 0];
        float y = cloud.points[i * 3 + 1];
        float z = cloud.points[i * 3 + 2];

        file.write((char *)&x, sizeof(float));
        file.write((char *)&y, sizeof(float));
        file.write((char *)&z, sizeof(float));
        file.write((char *)&cloud.colors[i * 3 + 0], 1);
        file.write((char *)&cloud.colors[i * 3 + 1], 1);
        file.write((char *)&cloud.colors[i * 3 + 2], 1);
    }

    file.close();
    std::cout << "Saved point cloud with " << cloud.num_points << " points to " << filename << std::endl;
}