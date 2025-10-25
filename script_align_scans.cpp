#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/SVD>

struct Point
{
    float x, y, z;
    unsigned char r, g, b;
};

struct PointCloud
{
    std::vector<Point> points;
};

PointCloud loadPLY(const std::string &filename)
{
    PointCloud cloud;
    std::ifstream file(filename, std::ios::binary);

    if (!file.is_open())
    {
        std::cout << "Failed to open " << filename << std::endl;
        return cloud;
    }

    std::string line;
    int num_vertices = 0;
    bool is_binary = false;

    // Parse header
    while (std::getline(file, line))
    {
        if (line.find("element vertex") != std::string::npos)
        {
            sscanf(line.c_str(), "element vertex %d", &num_vertices);
        }
        if (line.find("format binary") != std::string::npos)
        {
            is_binary = true;
        }
        if (line == "end_header")
            break;
    }

    // Read binary data
    for (int i = 0; i < num_vertices; i++)
    {
        Point p;
        file.read((char *)&p.x, sizeof(float));
        file.read((char *)&p.y, sizeof(float));
        file.read((char *)&p.z, sizeof(float));
        file.read((char *)&p.r, 1);
        file.read((char *)&p.g, 1);
        file.read((char *)&p.b, 1);
        cloud.points.push_back(p);
    }

    file.close();
    std::cout << "Loaded " << filename << " with " << cloud.points.size() << " points" << std::endl;
    return cloud;
}

void savePLY(const std::string &filename, const PointCloud &cloud)
{
    std::ofstream file(filename, std::ios::binary);

    std::string header =
        "ply\n"
        "format binary_little_endian 1.0\n"
        "element vertex " +
        std::to_string(cloud.points.size()) + "\n"
                                              "property float x\n"
                                              "property float y\n"
                                              "property float z\n"
                                              "property uchar red\n"
                                              "property uchar green\n"
                                              "property uchar blue\n"
                                              "end_header\n";

    file.write(header.c_str(), header.size());

    for (const auto &p : cloud.points)
    {
        file.write((char *)&p.x, sizeof(float));
        file.write((char *)&p.y, sizeof(float));
        file.write((char *)&p.z, sizeof(float));
        file.write((char *)&p.r, 1);
        file.write((char *)&p.g, 1);
        file.write((char *)&p.b, 1);
    }

    file.close();
}

// Downsample point cloud
PointCloud downsample(const PointCloud &cloud, int skip)
{
    PointCloud result;
    for (size_t i = 0; i < cloud.points.size(); i += skip)
    {
        result.points.push_back(cloud.points[i]);
    }
    return result;
}

// Fast ICP with downsampling
Eigen::Matrix4f fastICP(const PointCloud &source, const PointCloud &target, int max_iterations = 10)
{
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    // Downsample heavily for speed
    PointCloud source_down = downsample(source, 10);
    PointCloud target_down = downsample(target, 10);

    std::cout << "  Using " << source_down.points.size() << " source points and "
              << target_down.points.size() << " target points" << std::endl;

    PointCloud transformed = source_down;

    for (int iter = 0; iter < max_iterations; iter++)
    {
        std::vector<std::pair<int, int>> correspondences;
        float total_error = 0;

        // Find closest points with early termination
        for (size_t i = 0; i < transformed.points.size(); i++)
        {
            float min_dist = std::numeric_limits<float>::max();
            int closest_idx = -1;

            for (size_t j = 0; j < target_down.points.size(); j++)
            {
                float dx = transformed.points[i].x - target_down.points[j].x;
                float dy = transformed.points[i].y - target_down.points[j].y;
                float dz = transformed.points[i].z - target_down.points[j].z;
                float dist = dx * dx + dy * dy + dz * dz;

                if (dist < min_dist)
                {
                    min_dist = dist;
                    closest_idx = j;
                }

                // Early exit if we found a really close match
                if (dist < 0.001f)
                    break;
            }

            if (closest_idx >= 0 && min_dist < 0.25f)
            { // 50cm threshold
                correspondences.push_back({i, closest_idx});
                total_error += min_dist;
            }
        }

        if (correspondences.empty() || correspondences.size() < 10)
        {
            std::cout << "  Iteration " << iter << ": Too few correspondences, stopping" << std::endl;
            break;
        }

        std::cout << "  Iteration " << iter << ": " << correspondences.size()
                  << " correspondences, avg error: " << sqrt(total_error / correspondences.size()) << "m" << std::endl;

        // Compute centroids
        Eigen::Vector3f centroid_source(0, 0, 0);
        Eigen::Vector3f centroid_target(0, 0, 0);

        for (const auto &pair : correspondences)
        {
            centroid_source += Eigen::Vector3f(
                transformed.points[pair.first].x,
                transformed.points[pair.first].y,
                transformed.points[pair.first].z);
            centroid_target += Eigen::Vector3f(
                target_down.points[pair.second].x,
                target_down.points[pair.second].y,
                target_down.points[pair.second].z);
        }

        centroid_source /= correspondences.size();
        centroid_target /= correspondences.size();

        // Compute cross-covariance matrix
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();

        for (const auto &pair : correspondences)
        {
            Eigen::Vector3f p_source(
                transformed.points[pair.first].x - centroid_source.x(),
                transformed.points[pair.first].y - centroid_source.y(),
                transformed.points[pair.first].z - centroid_source.z());
            Eigen::Vector3f p_target(
                target_down.points[pair.second].x - centroid_target.x(),
                target_down.points[pair.second].y - centroid_target.y(),
                target_down.points[pair.second].z - centroid_target.z());

            H += p_source * p_target.transpose();
        }

        // SVD to get rotation
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

        // Handle reflection case
        if (R.determinant() < 0)
        {
            Eigen::Matrix3f V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        Eigen::Vector3f t = centroid_target - R * centroid_source;

        // Build transformation matrix
        Eigen::Matrix4f iter_transform = Eigen::Matrix4f::Identity();
        iter_transform.block<3, 3>(0, 0) = R;
        iter_transform.block<3, 1>(0, 3) = t;

        // Apply transformation
        for (auto &p : transformed.points)
        {
            Eigen::Vector4f point(p.x, p.y, p.z, 1.0f);
            Eigen::Vector4f transformed_point = iter_transform * point;
            p.x = transformed_point.x();
            p.y = transformed_point.y();
            p.z = transformed_point.z();
        }

        transformation = iter_transform * transformation;

        // Check convergence
        if (sqrt(total_error / correspondences.size()) < 0.01f)
        {
            std::cout << "  Converged!" << std::endl;
            break;
        }
    }

    return transformation;
}

PointCloud transformCloud(const PointCloud &cloud, const Eigen::Matrix4f &transform)
{
    PointCloud result = cloud;

    for (auto &p : result.points)
    {
        Eigen::Vector4f point(p.x, p.y, p.z, 1.0f);
        Eigen::Vector4f transformed = transform * point;
        p.x = transformed.x();
        p.y = transformed.y();
        p.z = transformed.z();
    }

    return result;
}

int main()
{
    int num_scans = 8;
    std::vector<PointCloud> clouds;

    // Load all scans
    std::cout << "Loading scans..." << std::endl;
    for (int i = 0; i < num_scans; i++)
    {
        std::string filename = "scans/scan_" + std::to_string(i) + ".ply";
        PointCloud cloud = loadPLY(filename);
        if (cloud.points.empty())
        {
            std::cout << "Failed to load scan " << i << std::endl;
            return -1;
        }
        clouds.push_back(cloud);
    }

    // Start with first scan as base
    PointCloud merged = clouds[0];

    std::cout << "\nAligning scans..." << std::endl;

    // Align each scan to the merged cloud
    for (int i = 1; i < num_scans; i++)
    {
        std::cout << "Aligning scan " << i << "..." << std::endl;

        Eigen::Matrix4f transform = fastICP(clouds[i], merged);
        PointCloud aligned = transformCloud(clouds[i], transform);

        // Merge
        merged.points.insert(merged.points.end(), aligned.points.begin(), aligned.points.end());
        std::cout << "  Merged cloud now has " << merged.points.size() << " points" << std::endl;
    }

    std::cout << "\nSaving merged point cloud..." << std::endl;
    savePLY("scans/merged.ply", merged);
    std::cout << "Saved merged.ply with " << merged.points.size() << " points" << std::endl;

    return 0;
}