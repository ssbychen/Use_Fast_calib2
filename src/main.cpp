/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"
#include <cstdlib>

namespace
{
void printUsage(const char* program)
{
    std::cerr << "Usage: " << program
              << " --pcd <path> --image <path> --output <path> [--config <path>]"
              << std::endl;
}
}

int main(int argc, char **argv) 
{
    std::string config_path;
    std::string cli_pcd_path;
    std::string cli_image_path;
    std::string cli_output_path;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& option) -> const char*
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for " << option << std::endl;
                printUsage(argv[0]);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--config")
        {
            config_path = require_value(arg);
        }
        else if (arg == "--pcd")
        {
            cli_pcd_path = require_value(arg);
        }
        else if (arg == "--image")
        {
            cli_image_path = require_value(arg);
        }
        else if (arg == "--output")
        {
            cli_output_path = require_value(arg);
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            printUsage(argv[0]);
            return 2;
        }
    }

    Params params = loadParameters(config_path);
    if (!cli_pcd_path.empty()) params.pcd_path = cli_pcd_path;
    if (!cli_image_path.empty()) params.image_path = cli_image_path;
    if (!cli_output_path.empty()) params.output_path = cli_output_path;

    if (params.pcd_path.empty() || params.image_path.empty() || params.output_path.empty())
    {
        std::cerr << "[Main] --pcd, --image and --output must be provided directly or via --config." << std::endl;
        printUsage(argv[0]);
        return 2;
    }

    std::string mounting_error;
    if (!validateLidarMountAxes(params.lidar_forward_axis, params.lidar_up_axis,
                                mounting_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid LiDAR mounting-axis configuration: "
                       << mounting_error);
      return 1;
    }

    std::string output_error;
    if (!ensureDirectoryTree(params.output_path, output_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid output directory: " << output_error);
      return 1;
    }

    DataPreprocessPtr dataPreprocessPtr(new DataPreprocess(params));

    cv::Mat img_input = dataPreprocessPtr->img_input_;
    pcl::PointCloud<Common::Point>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;
    if (img_input.empty())
    {
      ROS_ERROR_STREAM("[Main] Image is empty. Check image_path: " << params.image_path);
      return 1;
    }
    if (!cloud_input || cloud_input->empty())
    {
      ROS_ERROR_STREAM("[Main] Point cloud is empty. Check pcd_path: " << params.pcd_path);
      return 1;
    }
    if (dataPreprocessPtr->lidar_type_ == LiDARType::Unknown)
    {
      ROS_ERROR_STREAM("[Main] Unknown LiDAR type for PCD: " << params.pcd_path);
      return 1;
    }

    std::string camera_error;
    if (!validateCameraCalibrationForImage(params, img_input.cols, img_input.rows,
                                           camera_error))
    {
      ROS_ERROR_STREAM("[Main] Invalid camera calibration: " << camera_error);
      return 1;
    }

    QRDetectPtr qrDetectPtr(new QRDetect(params));
    LidarDetectPtr lidarDetectPtr(new LidarDetect(params));

    PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
    qr_center_cloud->reserve(4);
    qrDetectPtr->detect_qr(img_input, qr_center_cloud);
    if (qr_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " camera target centers, got " << qr_center_cloud->size() << ".");
      return 1;
    }

    PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
    lidar_center_cloud->reserve(4);

    switch (dataPreprocessPtr->lidar_type_)
    {
        case LiDARType::Solid:
            lidarDetectPtr->detect_solid_lidar(cloud_input, lidar_center_cloud);
            break;

        case LiDARType::Mech:
            lidarDetectPtr->detect_mech_lidar(cloud_input, lidar_center_cloud);
            break;

        default:
            std::cerr << BOLDYELLOW
                    << "[Main] Unknown LiDAR type."
                    << RESET << std::endl;
            break;
    }
    if (lidar_center_cloud->size() != TARGET_NUM_CIRCLES)
    {
      ROS_ERROR_STREAM("[Main] Expected " << TARGET_NUM_CIRCLES
                       << " LiDAR target centers, got " << lidar_center_cloud->size() << ".");
      return 1;
    }

    PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
    if (!sortPatternCenters(qr_center_cloud, qr_centers, "camera") ||
        !sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar",
                            params.lidar_forward_axis, params.lidar_up_axis))
    {
      ROS_ERROR("[Main] Failed to sort target centers. Check the LiDAR mounting-axis configuration.");
      return 1;
    }

    validateTargetGeometry(qr_centers, params.delta_width_circles, params.delta_height_circles, "QR");
    validateTargetGeometry(lidar_centers, params.delta_width_circles, params.delta_height_circles, "LiDAR");

    saveTargetHoleCenters(lidar_centers, qr_centers, params);

    Eigen::Matrix4f transformation;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
    svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
    aligned_lidar_centers->reserve(lidar_centers->size());
    alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

    double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
    if (rmse > 0) 
    {
      std::cout << BOLDYELLOW << "[Result] RMSE: " << BOLDRED << std::fixed << std::setprecision(4)
      << rmse << " m" << RESET << std::endl;
    }

    std::cout << BOLDYELLOW << "[Result] Single-scene calibration: extrinsic parameters T_cam_lidar = " << RESET << std::endl;
    std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

    saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);
    return 0;
}
