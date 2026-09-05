#include <gtest/gtest.h>

#include <array>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "common_lib.h"
#include "circle_center_extract_lib.hpp"

namespace
{
pcl::PointXYZ makePoint(const Eigen::Vector3f& p)
{
  pcl::PointXYZ out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

Eigen::Vector3f toOptical(const Eigen::Vector3f& native,
                         const Eigen::Vector3f& forward,
                         const Eigen::Vector3f& left,
                         const Eigen::Vector3f& up)
{
  return Eigen::Vector3f(-native.dot(left),
                         -native.dot(up),
                          native.dot(forward));
}
}  // namespace

TEST(LidarMountAxes, ResolvesAll24RightHandedCombinations)
{
  const std::vector<std::string> axes = {"+x", "-x", "+y", "-y", "+z", "-z"};
  const std::array<Eigen::Vector3f, 4> body_points = {{
      Eigen::Vector3f(3.0f,  0.25f,  0.20f),
      Eigen::Vector3f(3.0f, -0.25f, -0.20f),
      Eigen::Vector3f(3.0f,  0.25f, -0.20f),
      Eigen::Vector3f(3.0f, -0.25f,  0.20f)}};

  int valid_combinations = 0;
  for (const auto& forward_name : axes)
  {
    for (const auto& up_name : axes)
    {
      Eigen::Vector3f forward;
      Eigen::Vector3f left;
      Eigen::Vector3f up;
      std::string normalized_forward;
      std::string normalized_left;
      std::string normalized_up;
      std::string error;
      if (!resolveLidarMountAxes(forward_name, up_name,
                                 forward, left, up,
                                 normalized_forward, normalized_left, normalized_up,
                                 error))
      {
        continue;
      }

      ++valid_combinations;
      EXPECT_NEAR(forward.dot(up), 0.0f, 1e-6f);
      EXPECT_NEAR((forward.cross(left) - up).norm(), 0.0f, 1e-6f);

      pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& body : body_points)
      {
        const Eigen::Vector3f native =
            forward * body.x() + left * body.y() + up * body.z();
        lidar_points->push_back(makePoint(native));
        camera_points->push_back(makePoint(Eigen::Vector3f(-body.y(), -body.z(), body.x())));
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_camera(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_lidar(new pcl::PointCloud<pcl::PointXYZ>);
      ASSERT_TRUE(sortPatternCenters(camera_points, sorted_camera, "camera"));
      ASSERT_TRUE(sortPatternCenters(lidar_points, sorted_lidar, "lidar",
                                     forward_name, up_name));

      ASSERT_EQ(sorted_camera->size(), sorted_lidar->size());
      for (std::size_t i = 0; i < sorted_lidar->size(); ++i)
      {
        const auto& p = sorted_lidar->points[i];
        const Eigen::Vector3f optical =
            toOptical(Eigen::Vector3f(p.x, p.y, p.z), forward, left, up);
        EXPECT_NEAR(optical.x(), sorted_camera->points[i].x, 1e-5f);
        EXPECT_NEAR(optical.y(), sorted_camera->points[i].y, 1e-5f);
        EXPECT_NEAR(optical.z(), sorted_camera->points[i].z, 1e-5f);
      }
    }
  }

  EXPECT_EQ(valid_combinations, 24);
}

TEST(LidarMountAxes, RejectsInvalidOrParallelAxes)
{
  std::string error;
  EXPECT_FALSE(validateLidarMountAxes("+x", "-x", error));
  EXPECT_FALSE(validateLidarMountAxes("front", "+z", error));
  EXPECT_FALSE(validateLidarMountAxes("+x", "", error));
}

TEST(CameraCalibration, RequiresMatchingResolution)
{
  Params params{};
  params.camera_width = 2448;
  params.camera_height = 2048;
  params.fx = 2364.0;
  params.fy = 2368.0;
  params.cx = 1211.0;
  params.cy = 1040.0;
  params.k1 = -0.05;
  params.k2 = 0.12;
  params.p1 = 0.0;
  params.p2 = 0.0;

  std::string error;
  EXPECT_TRUE(validateCameraCalibrationForImage(params, 2448, 2048, error));
  EXPECT_FALSE(validateCameraCalibrationForImage(params, 1224, 1024, error));

  params.camera_width = 0;
  EXPECT_FALSE(validateCameraCalibrationForImage(params, 2448, 2048, error));
}

TEST(TargetLayout, GeneratesCircleHoleBoardObjectPoints)
{
  Params params{};
  params.target_type = Params::TargetType::CircleHoleBoard;
  params.hole_rows = 2;
  params.hole_cols = 3;
  params.hole_spacing_x = 0.2;
  params.hole_spacing_y = 0.1;

  const auto object_points = generateCircleHoleBoardObjectPoints(params);
  ASSERT_EQ(object_points.size(), 6u);
  EXPECT_EQ(expectedTargetCount(params), 6);
  EXPECT_FLOAT_EQ(object_points.front().x, -0.2f);
  EXPECT_FLOAT_EQ(object_points.front().y, 0.05f);
  EXPECT_FLOAT_EQ(object_points.back().x, 0.2f);
  EXPECT_FLOAT_EQ(object_points.back().y, -0.05f);
}

TEST(TargetSorting, SortsCircleHoleGridForCameraAndLidar)
{
  const int rows = 2;
  const int cols = 3;
  const double spacing_x = 0.2;
  const double spacing_y = 0.1;
  const std::string forward_name = "+x";
  const std::string up_name = "+z";

  Eigen::Vector3f forward;
  Eigen::Vector3f left;
  Eigen::Vector3f up;
  std::string normalized_forward;
  std::string normalized_left;
  std::string normalized_up;
  std::string error;
  ASSERT_TRUE(resolveLidarMountAxes(forward_name, up_name,
                                    forward, left, up,
                                    normalized_forward, normalized_left,
                                    normalized_up, error));

  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points(new pcl::PointCloud<pcl::PointXYZ>);
  const std::vector<Eigen::Vector3f> ordered_body_points = {
      {3.0f,  0.2f, -0.05f},
      {3.0f,  0.0f, -0.05f},
      {3.0f, -0.2f, -0.05f},
      {3.0f,  0.2f,  0.05f},
      {3.0f,  0.0f,  0.05f},
      {3.0f, -0.2f,  0.05f}};
  const std::vector<int> shuffled = {4, 1, 5, 0, 3, 2};

  for (int idx : shuffled)
  {
    const Eigen::Vector3f& body = ordered_body_points[idx];
    const Eigen::Vector3f native =
        forward * body.x() + left * body.y() + up * body.z();
    lidar_points->push_back(makePoint(native));
    camera_points->push_back(makePoint(Eigen::Vector3f(-body.y(), -body.z(), body.x())));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_camera(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sorted_lidar(new pcl::PointCloud<pcl::PointXYZ>);
  ASSERT_TRUE(sortGridPatternCenters(camera_points, sorted_camera,
                                     rows, cols, spacing_x, spacing_y, "camera"));
  ASSERT_TRUE(sortGridPatternCenters(lidar_points, sorted_lidar,
                                     rows, cols, spacing_x, spacing_y,
                                     "lidar", forward_name, up_name));

  ASSERT_EQ(sorted_camera->size(), sorted_lidar->size());
  for (std::size_t i = 0; i < sorted_lidar->size(); ++i)
  {
    const auto& p = sorted_lidar->points[i];
    const Eigen::Vector3f optical =
        toOptical(Eigen::Vector3f(p.x, p.y, p.z), forward, left, up);
    EXPECT_NEAR(optical.x(), sorted_camera->points[i].x, 1e-5f);
    EXPECT_NEAR(optical.y(), sorted_camera->points[i].y, 1e-5f);
    EXPECT_NEAR(optical.z(), sorted_camera->points[i].z, 1e-5f);
  }
}

TEST(TargetSorting, SortsStandaloneExtractionGridWithoutSpacingHints)
{
  const int rows = 3;
  const int cols = 4;
  const double angle = -0.25;
  const double cos_a = std::cos(angle);
  const double sin_a = std::sin(angle);
  const Eigen::Vector2d translation(1.2, -0.4);

  std::vector<Eigen::Vector2d> ordered_points;
  std::vector<std::pair<int, int>> ordered_labels;
  ordered_points.reserve(rows * cols);
  ordered_labels.reserve(rows * cols);
  for (int row = 0; row < rows; ++row)
  {
    for (int col = 0; col < cols; ++col)
    {
      const Eigen::Vector2d base(0.18 * static_cast<double>(col),
                                 0.11 * static_cast<double>(row));
      const Eigen::Vector2d rotated(cos_a * base.x() - sin_a * base.y(),
                                    sin_a * base.x() + cos_a * base.y());
      ordered_points.push_back(rotated + translation);
      ordered_labels.emplace_back(row, col);
    }
  }

  const std::vector<int> shuffled = {7, 2, 10, 0, 5, 9, 1, 11, 3, 8, 4, 6};
  std::vector<Eigen::Vector2d> shuffled_points;
  std::vector<std::pair<int, int>> shuffled_labels;
  shuffled_points.reserve(shuffled.size());
  shuffled_labels.reserve(shuffled.size());
  for (int index : shuffled)
  {
    shuffled_points.push_back(ordered_points[static_cast<size_t>(index)]);
    shuffled_labels.push_back(ordered_labels[static_cast<size_t>(index)]);
  }

  std::vector<int> ordered_indices;
  std::string error;
  ASSERT_TRUE(circle_center_extract::sortGridIndices(shuffled_points,
                                                     rows,
                                                     cols,
                                                     ordered_indices,
                                                     &error))
      << error;

  ASSERT_EQ(ordered_indices.size(), ordered_points.size());
  std::vector<int> row_sequence;
  std::vector<int> col_sequence;
  row_sequence.reserve(rows);
  col_sequence.reserve(cols);
  for (int row = 0; row < rows; ++row)
  {
    row_sequence.push_back(
        shuffled_labels[static_cast<size_t>(ordered_indices[static_cast<size_t>(row * cols)])].first);
    for (int col = 0; col < cols; ++col)
    {
      const auto& label =
          shuffled_labels[static_cast<size_t>(ordered_indices[static_cast<size_t>(row * cols + col)])];
      EXPECT_EQ(label.first, row_sequence.back());
      if (row == 0) col_sequence.push_back(label.second);
      else EXPECT_EQ(label.second, col_sequence[static_cast<size_t>(col)]);
    }
  }

  EXPECT_EQ(std::abs(row_sequence[1] - row_sequence[0]), 1);
  EXPECT_EQ(std::abs(row_sequence[2] - row_sequence[1]), 1);
  EXPECT_EQ(std::abs(col_sequence[1] - col_sequence[0]), 1);
  EXPECT_EQ(std::abs(col_sequence[2] - col_sequence[1]), 1);
  EXPECT_EQ(std::abs(col_sequence[3] - col_sequence[2]), 1);
}

TEST(TargetSorting, SortsStandaloneExtractionGridWithUnequalSpacingHints)
{
  const int rows = 3;
  const int cols = 4;
  const double row_spacing = 0.09;
  const double col_spacing = 0.16;
  const double angle = 0.31;
  const double cos_a = std::cos(angle);
  const double sin_a = std::sin(angle);
  const Eigen::Vector2d translation(-0.8, 0.3);

  std::vector<Eigen::Vector2d> ordered_points;
  std::vector<std::pair<int, int>> ordered_labels;
  ordered_points.reserve(rows * cols);
  ordered_labels.reserve(rows * cols);
  for (int row = 0; row < rows; ++row)
  {
    for (int col = 0; col < cols; ++col)
    {
      const Eigen::Vector2d base(col_spacing * static_cast<double>(col),
                                 row_spacing * static_cast<double>(row));
      const Eigen::Vector2d rotated(cos_a * base.x() - sin_a * base.y(),
                                    sin_a * base.x() + cos_a * base.y());
      ordered_points.push_back(rotated + translation);
      ordered_labels.emplace_back(row, col);
    }
  }

  const std::vector<int> shuffled = {8, 2, 11, 0, 6, 9, 1, 10, 3, 5, 4, 7};
  std::vector<Eigen::Vector2d> shuffled_points;
  std::vector<std::pair<int, int>> shuffled_labels;
  for (int index : shuffled)
  {
    shuffled_points.push_back(ordered_points[static_cast<size_t>(index)]);
    shuffled_labels.push_back(ordered_labels[static_cast<size_t>(index)]);
  }

  std::vector<int> ordered_indices;
  std::string error;
  ASSERT_TRUE(circle_center_extract::sortGridIndices(shuffled_points,
                                                     rows,
                                                     cols,
                                                     row_spacing,
                                                     col_spacing,
                                                     ordered_indices,
                                                     &error))
      << error;

  ASSERT_EQ(ordered_indices.size(), shuffled_points.size());
  std::vector<int> row_sequence;
  std::vector<int> col_sequence;
  row_sequence.reserve(rows);
  col_sequence.reserve(cols);
  for (int row = 0; row < rows; ++row)
  {
    row_sequence.push_back(
        shuffled_labels[static_cast<size_t>(ordered_indices[static_cast<size_t>(row * cols)])].first);
    for (int col = 0; col < cols; ++col)
    {
      const auto& label =
          shuffled_labels[static_cast<size_t>(ordered_indices[static_cast<size_t>(row * cols + col)])];
      EXPECT_EQ(label.first, row_sequence.back());
      if (row == 0) col_sequence.push_back(label.second);
      else EXPECT_EQ(label.second, col_sequence[static_cast<size_t>(col)]);
    }
  }
  EXPECT_EQ(std::abs(row_sequence[1] - row_sequence[0]), 1);
  EXPECT_EQ(std::abs(row_sequence[2] - row_sequence[1]), 1);
  EXPECT_EQ(std::abs(col_sequence[1] - col_sequence[0]), 1);
  EXPECT_EQ(std::abs(col_sequence[2] - col_sequence[1]), 1);
  EXPECT_EQ(std::abs(col_sequence[3] - col_sequence[2]), 1);
}

TEST(TargetSorting, RejectsInconsistentSpacingHints)
{
  const int rows = 2;
  const int cols = 3;
  const std::vector<Eigen::Vector2d> points = {
      {0.0, 0.0}, {0.10, 0.0}, {0.20, 0.0},
      {0.0, 0.05}, {0.10, 0.05}, {0.20, 0.05}};

  std::vector<int> ordered_indices;
  std::string error;
  EXPECT_FALSE(circle_center_extract::sortGridIndices(points,
                                                      rows,
                                                      cols,
                                                      0.5,
                                                      0.5,
                                                      ordered_indices,
                                                      &error));
  EXPECT_FALSE(error.empty());
}

TEST(OutputDirectory, CreatesMissingParents)
{
  const std::string root =
      "/tmp/fast_calib_common_lib_test_" + std::to_string(static_cast<long long>(::getpid()));
  const std::string first = root + "/first";
  const std::string nested = first + "/second";

  std::string error;
  ASSERT_TRUE(ensureDirectoryTree(nested, error)) << error;

  struct stat status;
  ASSERT_EQ(::stat(nested.c_str(), &status), 0);
  EXPECT_TRUE(S_ISDIR(status.st_mode));

  EXPECT_EQ(::rmdir(nested.c_str()), 0);
  EXPECT_EQ(::rmdir(first.c_str()), 0);
  EXPECT_EQ(::rmdir(root.c_str()), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
