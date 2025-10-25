#ifndef KINECT_CAPTURE_H
#define KINECT_CAPTURE_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>

struct FrameCapture
{
    unsigned char *rgb_data;
    unsigned char *depth_data;
    unsigned char *ir_data;
    int rgb_width, rgb_height;
    int depth_width, depth_height;
    int ir_width, ir_height;
};

struct RGBFrame
{
    unsigned char *data;
    int width, height;
};

struct DepthFrame
{
    unsigned char *data;
    int width, height;
};

struct IRFrame
{
    unsigned char *data;
    int width, height;
};

struct PointCloudData
{
    float *points;         // X, Y, Z coordinates (3 floats per point)
    unsigned char *colors; // R, G, B colors (3 bytes per point)
    int num_points;
};

// Get all three frames
FrameCapture getFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener);
void freeFrameCapture(FrameCapture &capture);

// Get individual frames
RGBFrame getRGBFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener);
DepthFrame getDepthFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener);
IRFrame getIRFrame(libfreenect2::Freenect2Device *dev, libfreenect2::SyncMultiFrameListener &listener);

void freeRGBFrame(RGBFrame &frame);
void freeDepthFrame(DepthFrame &frame);
void freeIRFrame(IRFrame &frame);

// Point cloud functions
PointCloudData getPointCloud(libfreenect2::Freenect2Device *dev,
                             libfreenect2::SyncMultiFrameListener &listener,
                             libfreenect2::Registration *registration);
void freePointCloud(PointCloudData &cloud);
void savePointCloudPLY(const char *filename, const PointCloudData &cloud);

#endif