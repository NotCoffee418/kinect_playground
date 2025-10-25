#include <iostream>
#include <vector>
#include <unordered_map>
#include <GLFW/glfw3.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <Eigen/Dense>
#include <thread>
#include <atomic>
#include <mutex>

#include "kinect_viewer.h"

struct Point3D
{
    float x, y, z;
    unsigned char r, g, b;
};

struct PointCloud
{
    std::vector<Point3D> points;
};

struct VoxelKey
{
    int x, y, z;
    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

namespace std
{
    template <>
    struct hash<VoxelKey>
    {
        size_t operator()(const VoxelKey &k) const
        {
            return ((hash<int>()(k.x) ^ (hash<int>()(k.y) << 1)) >> 1) ^ (hash<int>()(k.z) << 1);
        }
    };
}

class VoxelGrid
{
private:
    std::unordered_map<VoxelKey, Point3D> voxels;
    float voxel_size;
    std::mutex mutex;

public:
    VoxelGrid(float size = 0.03f) : voxel_size(size) {}

    VoxelKey get_voxel_key(float x, float y, float z)
    {
        return {
            (int)floor(x / voxel_size),
            (int)floor(y / voxel_size),
            (int)floor(z / voxel_size)};
    }

    void add_point(const Point3D &p)
    {
        std::lock_guard<std::mutex> lock(mutex);
        VoxelKey key = get_voxel_key(p.x, p.y, p.z);
        voxels[key] = p;
    }

    PointCloud to_point_cloud()
    {
        std::lock_guard<std::mutex> lock(mutex);
        PointCloud cloud;
        for (const auto &pair : voxels)
        {
            cloud.points.push_back(pair.second);
        }
        return cloud;
    }

    size_t size()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return voxels.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex);
        voxels.clear();
    }
};

PointCloud extract_point_cloud(libfreenect2::Frame *depth, libfreenect2::Frame *rgb,
                               libfreenect2::Registration *registration, int skip = 6)
{
    PointCloud cloud;

    libfreenect2::Frame undistorted(512, 424, 4);
    libfreenect2::Frame registered(512, 424, 4);
    registration->apply(rgb, depth, &undistorted, &registered);

    float *depth_data = (float *)undistorted.data;
    unsigned char *rgb_data = registered.data;

    for (int y = 0; y < 424; y += skip)
    {
        for (int x = 0; x < 512; x += skip)
        {
            int idx = y * 512 + x;
            float d = depth_data[idx];

            if (d > 500 && d < 4000)
            {
                float px, py, pz;
                registration->getPointXYZ(&undistorted, y, x, px, py, pz);

                Point3D p;
                p.x = px;
                p.y = py;
                p.z = pz;
                p.r = rgb_data[idx * 4 + 2];
                p.g = rgb_data[idx * 4 + 1];
                p.b = rgb_data[idx * 4 + 0];

                cloud.points.push_back(p);
            }
        }
    }

    return cloud;
}

Eigen::Vector3f align_fast(const PointCloud &source, const PointCloud &target)
{
    if (source.points.empty() || target.points.empty())
    {
        return Eigen::Vector3f(0, 0, 0);
    }

    Eigen::Vector3f centroid_source(0, 0, 0);
    Eigen::Vector3f centroid_target(0, 0, 0);

    for (const auto &p : source.points)
    {
        centroid_source += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid_source /= source.points.size();

    for (const auto &p : target.points)
    {
        centroid_target += Eigen::Vector3f(p.x, p.y, p.z);
    }
    centroid_target /= target.points.size();

    return centroid_target - centroid_source;
}

void render_map(const PointCloud &map)
{
    glBegin(GL_POINTS);
    for (const auto &p : map.points)
    {
        glColor3f(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f);
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
}

// Background thread for SLAM processing
void slam_thread(std::atomic<bool> &running, VoxelGrid &voxel_map,
                 libfreenect2::Freenect2Device *dev,
                 libfreenect2::Registration *registration)
{

    libfreenect2::SyncMultiFrameListener listener(
        libfreenect2::Frame::Color | libfreenect2::Frame::Depth);
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    PointCloud previous_cloud;
    Eigen::Vector3f cumulative_offset(0, 0, 0);
    bool has_previous = false;
    int frame_skip_counter = 0;

    while (running)
    {
        libfreenect2::FrameMap frames;
        if (!listener.waitForNewFrame(frames, 1000))
        {
            continue;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        PointCloud current_cloud = extract_point_cloud(depth, rgb, registration, 8);

        frame_skip_counter++;
        if (frame_skip_counter % 10 == 0)
        { // Process every 10th frame
            if (!has_previous)
            {
                for (const auto &p : current_cloud.points)
                {
                    voxel_map.add_point(p);
                }
                previous_cloud = current_cloud;
                has_previous = true;
                std::cout << "First frame added" << std::endl;
            }
            else
            {
                Eigen::Vector3f offset = align_fast(current_cloud, previous_cloud);

                if (offset.norm() < 0.5f)
                { // Reasonable movement
                    cumulative_offset += offset;

                    for (auto p : current_cloud.points)
                    {
                        p.x += cumulative_offset.x();
                        p.y += cumulative_offset.y();
                        p.z += cumulative_offset.z();
                        voxel_map.add_point(p);
                    }

                    previous_cloud = current_cloud;
                    std::cout << "Frame added. Voxels: " << voxel_map.size() << std::endl;
                }
            }
        }

        listener.release(frames);
    }
}

int main()
{
    if (!glfwInit())
    {
        std::cout << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    GLFWwindow *window = glfwCreateWindow(1280, 720, "Kinect SLAM - Live Mapping", NULL, NULL);
    if (!window)
    {
        std::cout << "Failed to create window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    CameraState camera;
    init_camera(camera);

    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

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

    if (!dev->start())
    {
        std::cout << "Failed to start device" << std::endl;
        return -1;
    }

    libfreenect2::Freenect2Device::IrCameraParams ir_params = dev->getIrCameraParams();
    libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();
    libfreenect2::Registration *registration = new libfreenect2::Registration(ir_params, color_params);

    std::cout << "Kinect SLAM - Background Processing" << std::endl;
    std::cout << "Move slowly. Map builds automatically." << std::endl;
    std::cout << "C - Clear map" << std::endl;

    VoxelGrid voxel_map(0.03f);
    std::atomic<bool> slam_running(true);

    std::thread slam_worker(slam_thread, std::ref(slam_running), std::ref(voxel_map), dev, registration);

    while (!glfwWindowShouldClose(window))
    {
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
        {
            voxel_map.clear();
            std::cout << "Map cleared" << std::endl;
        }

        PointCloud display_cloud = voxel_map.to_point_cloud();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        setup_camera_view(camera);
        render_map(display_cloud);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    slam_running = false;
    slam_worker.join();

    std::cout << "Final: " << voxel_map.size() << " voxels" << std::endl;

    delete registration;
    dev->stop();
    dev->close();
    delete dev;
    glfwTerminate();
    return 0;
}