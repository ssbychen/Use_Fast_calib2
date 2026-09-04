#include "circle_center_extract_lib.hpp"

#include <Eigen/Dense>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{

struct CliOptions
{
  std::string input_pcd_path;
  std::string output_txt_path;
  int rows = 0;
  int cols = 0;
  double cluster_tolerance = 0.0;
  int min_cluster_size = 30;
};

struct PlaneFrame
{
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d axis_x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d axis_y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
};

struct CircleFit
{
  double x = 0.0;
  double y = 0.0;
  double radius = 0.0;
  double mean_abs_error = std::numeric_limits<double>::max();
  int support = 0;
  bool valid = false;
};

struct CircleCandidate
{
  CircleFit fit;
  pcl::PointXYZ plane_center;
  Eigen::Vector3d world_center = Eigen::Vector3d::Zero();
  int cluster_index = -1;
  size_t cluster_size = 0;
  double score = std::numeric_limits<double>::max();
};

void printUsage()
{
  std::cerr
      << "Usage: circle_center_extract <input.pcd> <output.txt> <rows> <cols> "
      << "<cluster_tolerance> [min_cluster_size]\n";
}

bool parseInt(const std::string& text, int& value)
{
  try
  {
    size_t consumed = 0;
    const int parsed = std::stoi(text, &consumed);
    if (consumed != text.size()) return false;
    value = parsed;
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool parseDouble(const std::string& text, double& value)
{
  try
  {
    size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size()) return false;
    value = parsed;
    return true;
  }
  catch (const std::exception&)
  {
    return false;
  }
}

bool parseCli(int argc, char** argv, CliOptions& options)
{
  if (argc < 6)
  {
    printUsage();
    return false;
  }

  options.input_pcd_path = argv[1];
  options.output_txt_path = argv[2];
  if (!parseInt(argv[3], options.rows) ||
      !parseInt(argv[4], options.cols) ||
      !parseDouble(argv[5], options.cluster_tolerance))
  {
    std::cerr << "Failed to parse rows/cols/cluster_tolerance." << std::endl;
    printUsage();
    return false;
  }

  if (argc >= 7 && !parseInt(argv[6], options.min_cluster_size))
  {
    std::cerr << "Failed to parse min_cluster_size." << std::endl;
    printUsage();
    return false;
  }

  if (options.rows <= 0 || options.cols <= 0)
  {
    std::cerr << "rows and cols must be positive." << std::endl;
    return false;
  }
  if (!std::isfinite(options.cluster_tolerance) || options.cluster_tolerance <= 0.0)
  {
    std::cerr << "cluster_tolerance must be a finite positive value." << std::endl;
    return false;
  }
  if (options.min_cluster_size < 8)
  {
    std::cerr << "min_cluster_size must be at least 8." << std::endl;
    return false;
  }
  return true;
}

bool fitPlaneFrame(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                   PlaneFrame& frame)
{
  if (!cloud || cloud->size() < 8) return false;

  Eigen::Vector4f centroid4f;
  pcl::compute3DCentroid(*cloud, centroid4f);
  frame.origin = centroid4f.head<3>().cast<double>();

  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*cloud, centroid4f, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
  if (solver.info() != Eigen::Success) return false;

  const Eigen::Vector3d normal = solver.eigenvectors().col(0).cast<double>().normalized();
  Eigen::Vector3d axis_x = solver.eigenvectors().col(2).cast<double>().normalized();
  Eigen::Vector3d axis_y = normal.cross(axis_x).normalized();
  if (axis_y.dot(solver.eigenvectors().col(1).cast<double>()) < 0.0) axis_y *= -1.0;
  axis_x = axis_y.cross(normal).normalized();

  if (!axis_x.allFinite() || !axis_y.allFinite() || !normal.allFinite()) return false;

  frame.normal = normal;
  frame.axis_x = axis_x;
  frame.axis_y = axis_y;
  return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                                                   const PlaneFrame& frame)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr projected(new pcl::PointCloud<pcl::PointXYZ>);
  projected->reserve(cloud->size());
  for (const auto& point : cloud->points)
  {
    const Eigen::Vector3d world(point.x, point.y, point.z);
    const Eigen::Vector3d d = world - frame.origin;

    pcl::PointXYZ projected_point;
    projected_point.x = static_cast<float>(d.dot(frame.axis_x));
    projected_point.y = static_cast<float>(d.dot(frame.axis_y));
    projected_point.z = 0.0f;
    projected->push_back(projected_point);
  }
  return projected;
}

Eigen::Vector3d planeToWorld(const PlaneFrame& frame, double x, double y)
{
  return frame.origin + frame.axis_x * x + frame.axis_y * y;
}

bool extractBoundaryCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& aligned_cloud,
                          double cluster_tolerance,
                          int min_cluster_size,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud)
{
  boundary_cloud->clear();
  if (!aligned_cloud || aligned_cloud->size() < static_cast<size_t>(min_cluster_size))
  {
    return false;
  }

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(aligned_cloud);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setInputCloud(aligned_cloud);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setRadiusSearch(std::max(0.01, cluster_tolerance * 1.5));
  normal_estimator.compute(*normals);
  if (normals->size() != aligned_cloud->size()) return false;

  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
  boundary_estimator.setInputCloud(aligned_cloud);
  boundary_estimator.setInputNormals(normals);
  boundary_estimator.setSearchMethod(tree);
  boundary_estimator.setRadiusSearch(std::max(0.01, cluster_tolerance * 1.8));
  boundary_estimator.setAngleThreshold(static_cast<float>(std::acos(-1.0) / 4.0));
  boundary_estimator.compute(boundaries);

  boundary_cloud->reserve(aligned_cloud->size());
  for (size_t i = 0; i < aligned_cloud->size() && i < boundaries.size(); ++i)
  {
    if (boundaries.points[i].boundary_point > 0)
    {
      boundary_cloud->push_back(aligned_cloud->points[i]);
    }
  }
  return boundary_cloud->size() >= static_cast<size_t>(min_cluster_size);
}

std::vector<pcl::PointIndices> clusterBoundaryCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& boundary_cloud,
                                                    double cluster_tolerance,
                                                    int min_cluster_size)
{
  std::vector<pcl::PointIndices> cluster_indices;
  if (!boundary_cloud || boundary_cloud->empty()) return cluster_indices;

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(boundary_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(cluster_tolerance);
  clustering.setMinClusterSize(min_cluster_size);
  clustering.setMaxClusterSize(static_cast<int>(boundary_cloud->size()));
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(boundary_cloud);
  clustering.extract(cluster_indices);
  return cluster_indices;
}

bool fitCircleRobust(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cluster, CircleFit& result)
{
  result = CircleFit{};
  if (!cluster || cluster->size() < 8) return false;

  double cx = 0.0;
  double cy = 0.0;
  for (const auto& point : cluster->points)
  {
    cx += static_cast<double>(point.x);
    cy += static_cast<double>(point.y);
  }
  cx /= static_cast<double>(cluster->size());
  cy /= static_cast<double>(cluster->size());

  std::vector<double> radii;
  radii.reserve(cluster->size());
  for (const auto& point : cluster->points)
  {
    const double dx = static_cast<double>(point.x) - cx;
    const double dy = static_cast<double>(point.y) - cy;
    radii.push_back(std::sqrt(dx * dx + dy * dy));
  }
  std::nth_element(radii.begin(), radii.begin() + radii.size() / 2, radii.end());
  double radius = radii[radii.size() / 2];
  if (!std::isfinite(radius) || radius <= 0.0) return false;

  const double huber_delta = std::max(0.002, radius * 0.2);
  for (int iteration = 0; iteration < 40; ++iteration)
  {
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();

    for (const auto& point : cluster->points)
    {
      const double dx = cx - static_cast<double>(point.x);
      const double dy = cy - static_cast<double>(point.y);
      const double distance = std::sqrt(dx * dx + dy * dy);
      if (distance < 1e-9) continue;

      const double residual = distance - radius;
      const double abs_residual = std::fabs(residual);
      const double weight = (abs_residual <= huber_delta) ? 1.0 : (huber_delta / abs_residual);

      const Eigen::Vector3d jacobian(dx / distance, dy / distance, -1.0);
      hessian += weight * jacobian * jacobian.transpose();
      gradient += weight * jacobian * residual;
    }

    const Eigen::Vector3d step = hessian.ldlt().solve(-gradient);
    if (!step.allFinite()) return false;

    cx += step.x();
    cy += step.y();
    radius += step.z();
    if (radius <= 0.0 || !std::isfinite(radius)) return false;
    if (step.norm() < 1e-7) break;
  }

  double error_sum = 0.0;
  int support = 0;
  for (const auto& point : cluster->points)
  {
    const double dx = static_cast<double>(point.x) - cx;
    const double dy = static_cast<double>(point.y) - cy;
    const double distance = std::sqrt(dx * dx + dy * dy);
    if (!std::isfinite(distance)) continue;
    error_sum += std::fabs(distance - radius);
    ++support;
  }
  if (support == 0) return false;

  result.x = cx;
  result.y = cy;
  result.radius = radius;
  result.mean_abs_error = error_sum / static_cast<double>(support);
  result.support = support;
  result.valid = std::isfinite(result.x) && std::isfinite(result.y) &&
                 std::isfinite(result.radius) && std::isfinite(result.mean_abs_error);
  return result.valid;
}

bool extractCircleCandidates(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& boundary_cloud,
                             const std::vector<pcl::PointIndices>& cluster_indices,
                             const PlaneFrame& frame,
                             std::vector<CircleCandidate>& candidates)
{
  candidates.clear();
  if (!boundary_cloud) return false;

  for (size_t cluster_index = 0; cluster_index < cluster_indices.size(); ++cluster_index)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
    cluster->reserve(cluster_indices[cluster_index].indices.size());
    for (int point_index : cluster_indices[cluster_index].indices)
    {
      cluster->push_back(boundary_cloud->points[static_cast<size_t>(point_index)]);
    }

    CircleFit fit;
    if (!fitCircleRobust(cluster, fit)) continue;
    if (fit.radius <= 0.0 || !std::isfinite(fit.radius)) continue;
    if ((fit.mean_abs_error / fit.radius) > 0.35) continue;

    CircleCandidate candidate;
    candidate.fit = fit;
    candidate.plane_center.x = static_cast<float>(fit.x);
    candidate.plane_center.y = static_cast<float>(fit.y);
    candidate.plane_center.z = 0.0f;
    candidate.world_center = planeToWorld(frame, fit.x, fit.y);
    candidate.cluster_index = static_cast<int>(cluster_index);
    candidate.cluster_size = cluster->size();
    candidates.push_back(candidate);
  }

  if (candidates.empty()) return false;

  std::vector<double> radii;
  radii.reserve(candidates.size());
  for (const auto& candidate : candidates) radii.push_back(candidate.fit.radius);
  std::nth_element(radii.begin(), radii.begin() + radii.size() / 2, radii.end());
  const double median_radius = radii[radii.size() / 2];
  if (!std::isfinite(median_radius) || median_radius <= 0.0) return false;

  for (auto& candidate : candidates)
  {
    const double normalized_error = candidate.fit.mean_abs_error / median_radius;
    const double radius_penalty =
        std::fabs(std::log(std::max(candidate.fit.radius, 1e-9) / median_radius));
    candidate.score = normalized_error + radius_penalty;
  }
  return true;
}

bool writeResults(const std::string& output_path,
                  int rows,
                  int cols,
                  const std::vector<CircleCandidate>& ordered_candidates)
{
  std::ofstream output(output_path.c_str());
  if (!output.is_open())
  {
    std::cerr << "Failed to open output file: " << output_path << std::endl;
    return false;
  }

  output << std::fixed << std::setprecision(9);
  output << "# row col center_x center_y center_z radius mean_abs_error\n";
  for (int row = 0; row < rows; ++row)
  {
    for (int col = 0; col < cols; ++col)
    {
      const CircleCandidate& candidate = ordered_candidates[static_cast<size_t>(row * cols + col)];
      output << row << ' '
             << col << ' '
             << candidate.world_center.x() << ' '
             << candidate.world_center.y() << ' '
             << candidate.world_center.z() << ' '
             << candidate.fit.radius << ' '
             << candidate.fit.mean_abs_error << '\n';
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv)
{
  CliOptions options;
  if (!parseCli(argc, argv, options)) return 1;

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile(options.input_pcd_path, *input_cloud) != 0)
  {
    std::cerr << "Failed to load input PCD: " << options.input_pcd_path << std::endl;
    return 1;
  }
  if (input_cloud->empty())
  {
    std::cerr << "Input cloud is empty: " << options.input_pcd_path << std::endl;
    return 1;
  }

  PlaneFrame frame;
  if (!fitPlaneFrame(input_cloud, frame))
  {
    std::cerr << "Failed to estimate a board plane from the cropped cloud." << std::endl;
    return 1;
  }

  const pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud = projectToPlane(input_cloud, frame);
  pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (!extractBoundaryCloud(aligned_cloud,
                            options.cluster_tolerance,
                            options.min_cluster_size,
                            boundary_cloud))
  {
    std::cerr << "Failed to extract enough boundary points from the board cloud." << std::endl;
    return 1;
  }

  const std::vector<pcl::PointIndices> cluster_indices =
      clusterBoundaryCloud(boundary_cloud,
                           options.cluster_tolerance,
                           options.min_cluster_size);
  if (cluster_indices.empty())
  {
    std::cerr << "No boundary clusters were found." << std::endl;
    return 1;
  }

  std::vector<CircleCandidate> candidates;
  if (!extractCircleCandidates(boundary_cloud, cluster_indices, frame, candidates))
  {
    std::cerr << "Failed to fit any circular boundary clusters." << std::endl;
    return 1;
  }

  const int expected_count = options.rows * options.cols;
  if (static_cast<int>(candidates.size()) < expected_count)
  {
    std::cerr << "Only " << candidates.size() << " circular clusters passed fitting, but "
              << expected_count << " are required for a "
              << options.rows << "x" << options.cols << " board." << std::endl;
    return 1;
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const CircleCandidate& a, const CircleCandidate& b) {
              if (std::fabs(a.score - b.score) > 1e-12) return a.score < b.score;
              if (std::fabs(a.fit.mean_abs_error - b.fit.mean_abs_error) > 1e-12) {
                return a.fit.mean_abs_error < b.fit.mean_abs_error;
              }
              return a.cluster_size > b.cluster_size;
            });

  std::vector<CircleCandidate> unique_candidates;
  unique_candidates.reserve(candidates.size());
  for (const auto& candidate : candidates)
  {
    bool duplicate = false;
    for (const auto& existing : unique_candidates)
    {
      const double dx = candidate.plane_center.x - existing.plane_center.x;
      const double dy = candidate.plane_center.y - existing.plane_center.y;
      const double min_distance = 0.6 * std::max(candidate.fit.radius, existing.fit.radius);
      if ((dx * dx + dy * dy) <= (min_distance * min_distance))
      {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) unique_candidates.push_back(candidate);
  }
  candidates.swap(unique_candidates);

  if (static_cast<int>(candidates.size()) < expected_count)
  {
    std::cerr << "Only " << candidates.size()
              << " unique circular clusters remained after de-duplication, but "
              << expected_count << " are required." << std::endl;
    return 1;
  }
  candidates.resize(static_cast<size_t>(expected_count));

  pcl::PointCloud<pcl::PointXYZ>::Ptr plane_centers(new pcl::PointCloud<pcl::PointXYZ>);
  plane_centers->reserve(candidates.size());
  for (const auto& candidate : candidates) plane_centers->push_back(candidate.plane_center);

  std::vector<int> ordered_indices;
  std::string order_error;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_plane_centers(new pcl::PointCloud<pcl::PointXYZ>);
  if (!circle_center_extract::sortGridCloud(plane_centers,
                                            options.rows,
                                            options.cols,
                                            ordered_plane_centers,
                                            &ordered_indices,
                                            &order_error))
  {
    std::cerr << "Failed to order fitted circles into a grid: " << order_error << std::endl;
    return 1;
  }

  std::vector<CircleCandidate> ordered_candidates;
  ordered_candidates.reserve(candidates.size());
  for (int index : ordered_indices)
  {
    ordered_candidates.push_back(candidates[static_cast<size_t>(index)]);
  }

  if (!writeResults(options.output_txt_path, options.rows, options.cols, ordered_candidates))
  {
    return 1;
  }

  std::cout << "Saved " << ordered_candidates.size()
            << " ordered circle centers to " << options.output_txt_path << std::endl;
  return 0;
}
