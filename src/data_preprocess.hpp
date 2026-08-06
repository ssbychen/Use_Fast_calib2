/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <vector>
#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,
    Mech    = 2
};

class DataPreprocess
{
private:
    static const pcl::PCLPointField* findField(const pcl::PCLPointCloud2& cloud, const std::string& name)
    {
        for (const auto& field : cloud.fields)
        {
            if (field.name == name) return &field;
        }
        return nullptr;
    }

    static double readNumericField(const pcl::PCLPointCloud2& cloud, const pcl::PCLPointField& field, std::size_t point_index)
    {
        const std::size_t point_step = cloud.point_step;
        const std::size_t offset = point_index * point_step + field.offset;
        if (offset >= cloud.data.size())
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        const std::uint8_t* ptr = cloud.data.data() + offset;
        switch (field.datatype)
        {
            case pcl::PCLPointField::INT8:   return static_cast<double>(*reinterpret_cast<const std::int8_t*>(ptr));
            case pcl::PCLPointField::UINT8:  return static_cast<double>(*reinterpret_cast<const std::uint8_t*>(ptr));
            case pcl::PCLPointField::INT16:  return static_cast<double>(*reinterpret_cast<const std::int16_t*>(ptr));
            case pcl::PCLPointField::UINT16: return static_cast<double>(*reinterpret_cast<const std::uint16_t*>(ptr));
            case pcl::PCLPointField::INT32:  return static_cast<double>(*reinterpret_cast<const std::int32_t*>(ptr));
            case pcl::PCLPointField::UINT32: return static_cast<double>(*reinterpret_cast<const std::uint32_t*>(ptr));
            case pcl::PCLPointField::FLOAT32:return static_cast<double>(*reinterpret_cast<const float*>(ptr));
            case pcl::PCLPointField::FLOAT64:return *reinterpret_cast<const double*>(ptr);
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    }

public:
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        img_input_ = cv::imread(params.image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::cerr << "[DataPreprocess] Failed to load image: " << params.image_path << std::endl;
            return;
        }

        pcl::PCLPointCloud2 cloud2;
        if (pcl::io::loadPCDFile(params.pcd_path, cloud2) < 0)
        {
            std::cerr << "[DataPreprocess] Failed to load PCD: " << params.pcd_path << std::endl;
            return;
        }

        const auto* x_field = findField(cloud2, "x");
        const auto* y_field = findField(cloud2, "y");
        const auto* z_field = findField(cloud2, "z");
        const auto* ring_field = findField(cloud2, "ring");
        const auto* intensity_field = findField(cloud2, "intensity");
        const auto* reflectivity_field = findField(cloud2, "reflectivity");
        const auto* scan_id_field = findField(cloud2, "scan_id");

        if (!x_field || !y_field || !z_field)
        {
            std::cerr << "[DataPreprocess] PCD is missing x/y/z fields: " << params.pcd_path << std::endl;
            return;
        }

        lidar_type_ = ring_field ? LiDARType::Mech : LiDARType::Solid;

        const std::size_t point_count = static_cast<std::size_t>(cloud2.width) * static_cast<std::size_t>(cloud2.height);
        cloud_input_->reserve(point_count);
        for (std::size_t i = 0; i < point_count; ++i)
        {
            Common::Point p;
            p.x = static_cast<float>(readNumericField(cloud2, *x_field, i));
            p.y = static_cast<float>(readNumericField(cloud2, *y_field, i));
            p.z = static_cast<float>(readNumericField(cloud2, *z_field, i));
            if (intensity_field)
            {
                p.intensity = static_cast<float>(readNumericField(cloud2, *intensity_field, i));
            }
            else if (reflectivity_field)
            {
                p.intensity = static_cast<float>(readNumericField(cloud2, *reflectivity_field, i));
            }
            else
            {
                p.intensity = 0.0f;
            }
            p.ring = ring_field
                ? static_cast<std::uint16_t>(readNumericField(cloud2, *ring_field, i))
                : 0;
            p.scan_id = scan_id_field
                ? static_cast<std::uint32_t>(readNumericField(cloud2, *scan_id_field, i))
                : 0;
            cloud_input_->push_back(p);
        }

        std::cout << "[DataPreprocess] Loaded " << cloud_input_->size()
                  << " points from " << params.pcd_path << std::endl;
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP
