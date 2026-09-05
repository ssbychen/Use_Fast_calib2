#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace circle_center_extract
{

struct GridProjection
{
  double row_value = 0.0;
  double col_value = 0.0;
  int index = -1;
};

inline double spanSquared(const std::vector<GridProjection>& projections,
                          int rows,
                          int cols)
{
  double score = 0.0;
  for (int row = 0; row < rows; ++row)
  {
    const int row_begin = row * cols;
    const int row_end = row_begin + cols;
    auto minmax_row = std::minmax_element(
        projections.begin() + row_begin, projections.begin() + row_end,
        [](const GridProjection& a, const GridProjection& b) {
          return a.row_value < b.row_value;
        });
    const double row_span = minmax_row.second->row_value - minmax_row.first->row_value;
    score += row_span * row_span;

    auto minmax_col = std::minmax_element(
        projections.begin() + row_begin, projections.begin() + row_end,
        [](const GridProjection& a, const GridProjection& b) {
          return a.col_value < b.col_value;
        });
    const double col_span = minmax_col.second->col_value - minmax_col.first->col_value;
    score -= col_span * col_span;
  }

  for (int col = 0; col < cols; ++col)
  {
    double min_col = std::numeric_limits<double>::max();
    double max_col = std::numeric_limits<double>::lowest();
    for (int row = 0; row < rows; ++row)
    {
      const double col_value = projections[row * cols + col].col_value;
      min_col = std::min(min_col, col_value);
      max_col = std::max(max_col, col_value);
    }
    const double span = max_col - min_col;
    score += span * span;
  }
  return score;
}

inline std::vector<GridProjection> buildOrderedProjections(const std::vector<Eigen::Vector2d>& points,
                                                           const Eigen::Vector2d& row_axis,
                                                           const Eigen::Vector2d& col_axis)
{
  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : points) centroid += p;
  centroid /= static_cast<double>(points.size());

  std::vector<GridProjection> projections;
  projections.reserve(points.size());
  for (int i = 0; i < static_cast<int>(points.size()); ++i)
  {
    const Eigen::Vector2d d = points[static_cast<size_t>(i)] - centroid;
    projections.push_back({d.dot(row_axis), d.dot(col_axis), i});
  }
  return projections;
}

inline bool lexicographicallyLess(const std::vector<int>& lhs,
                                  const std::vector<int>& rhs,
                                  const std::vector<Eigen::Vector2d>& points)
{
  if (rhs.empty()) return true;
  for (size_t i = 0; i < lhs.size() && i < rhs.size(); ++i)
  {
    const Eigen::Vector2d& a = points[static_cast<size_t>(lhs[i])];
    const Eigen::Vector2d& b = points[static_cast<size_t>(rhs[i])];
    if (std::fabs(a.x() - b.x()) > 1e-9) return a.x() < b.x();
    if (std::fabs(a.y() - b.y()) > 1e-9) return a.y() < b.y();
  }
  return lhs.size() < rhs.size();
}

inline bool assignQuantizedGridBySpacing(const std::vector<GridProjection>& projections,
                                         int rows,
                                         int cols,
                                         double row_spacing,
                                         double col_spacing,
                                         std::vector<int>& ordered_indices,
                                         double& score)
{
  const int expected = rows * cols;
  ordered_indices.assign(static_cast<size_t>(expected), -1);
  score = std::numeric_limits<double>::max();
  if (row_spacing <= 0.0 || col_spacing <= 0.0) return false;

  double min_row = std::numeric_limits<double>::max();
  double min_col = std::numeric_limits<double>::max();
  for (const auto& p : projections)
  {
    min_row = std::min(min_row, p.row_value);
    min_col = std::min(min_col, p.col_value);
  }

  double local_score = 0.0;
  for (const auto& p : projections)
  {
    const double row_value = (p.row_value - min_row) / row_spacing;
    const double col_value = (p.col_value - min_col) / col_spacing;
    const int row = static_cast<int>(std::llround(row_value));
    const int col = static_cast<int>(std::llround(col_value));
    if (row < 0 || row >= rows || col < 0 || col >= cols) return false;

    const int slot = row * cols + col;
    if (ordered_indices[static_cast<size_t>(slot)] >= 0) return false;
    ordered_indices[static_cast<size_t>(slot)] = p.index;

    const double row_residual = row_value - static_cast<double>(row);
    const double col_residual = col_value - static_cast<double>(col);
    local_score += row_residual * row_residual + col_residual * col_residual;
  }

  for (int idx : ordered_indices)
  {
    if (idx < 0) return false;
  }
  score = local_score;
  return true;
}

inline bool sortGridIndicesLegacy(const std::vector<Eigen::Vector2d>& points,
                                  int rows,
                                  int cols,
                                  std::vector<int>& ordered_indices,
                                  std::string* error = nullptr)
{
  ordered_indices.clear();
  const int expected = rows * cols;
  if (rows <= 0 || cols <= 0 || static_cast<int>(points.size()) != expected)
  {
    if (error) *error = "point count does not match rows*cols";
    return false;
  }

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : points) centroid += p;
  centroid /= static_cast<double>(points.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& p : points)
  {
    const Eigen::Vector2d d = p - centroid;
    covariance += d * d.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success)
  {
    if (error) *error = "failed to solve PCA";
    return false;
  }

  const Eigen::Vector2d axis_major = solver.eigenvectors().col(1).normalized();
  const Eigen::Vector2d axis_minor = solver.eigenvectors().col(0).normalized();
  const std::array<std::pair<Eigen::Vector2d, Eigen::Vector2d>, 2> assignments = {{
      {axis_minor, axis_major},
      {axis_major, axis_minor}}};

  double best_score = std::numeric_limits<double>::max();
  std::vector<int> best_indices;

  for (const auto& assignment : assignments)
  {
    for (const int row_sign : {-1, 1})
    {
      for (const int col_sign : {-1, 1})
      {
        std::vector<GridProjection> projections =
            buildOrderedProjections(points,
                                    static_cast<double>(row_sign) * assignment.first,
                                    static_cast<double>(col_sign) * assignment.second);

        std::sort(projections.begin(), projections.end(),
                  [](const GridProjection& a, const GridProjection& b) {
                    if (std::fabs(a.row_value - b.row_value) > 1e-9) {
                      return a.row_value < b.row_value;
                    }
                    return a.col_value < b.col_value;
                  });

        std::vector<int> current_indices;
        current_indices.reserve(points.size());
        for (int row = 0; row < rows; ++row)
        {
          auto row_begin = projections.begin() + row * cols;
          auto row_end = row_begin + cols;
          std::sort(row_begin, row_end,
                    [](const GridProjection& a, const GridProjection& b) {
                      return a.col_value < b.col_value;
                    });
          for (auto it = row_begin; it != row_end; ++it)
          {
            current_indices.push_back(it->index);
          }
        }

        const double score = spanSquared(projections, rows, cols);
        if (score + 1e-12 < best_score ||
            (std::fabs(score - best_score) <= 1e-12 &&
             lexicographicallyLess(current_indices, best_indices, points)))
        {
          best_score = score;
          best_indices.swap(current_indices);
        }
      }
    }
  }

  if (best_indices.size() != points.size())
  {
    if (error) *error = "failed to assign ordered grid indices";
    return false;
  }

  ordered_indices.swap(best_indices);
  return true;
}

inline bool sortGridIndices(const std::vector<Eigen::Vector2d>& points,
                            int rows,
                            int cols,
                            double row_spacing,
                            double col_spacing,
                            std::vector<int>& ordered_indices,
                            std::string* error = nullptr)
{
  if (!std::isfinite(row_spacing) || !std::isfinite(col_spacing) ||
      row_spacing <= 0.0 || col_spacing <= 0.0)
  {
    return sortGridIndicesLegacy(points, rows, cols, ordered_indices, error);
  }

  ordered_indices.clear();
  const int expected = rows * cols;
  if (rows <= 0 || cols <= 0 || static_cast<int>(points.size()) != expected)
  {
    if (error) *error = "point count does not match rows*cols";
    return false;
  }

  Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
  for (const auto& p : points) centroid += p;
  centroid /= static_cast<double>(points.size());

  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& p : points)
  {
    const Eigen::Vector2d d = p - centroid;
    covariance += d * d.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success)
  {
    if (error) *error = "failed to solve PCA";
    return false;
  }

  const Eigen::Vector2d axis_major = solver.eigenvectors().col(1).normalized();
  const Eigen::Vector2d axis_minor = solver.eigenvectors().col(0).normalized();
  const std::array<std::pair<Eigen::Vector2d, Eigen::Vector2d>, 2> assignments = {{
      {axis_minor, axis_major},
      {axis_major, axis_minor}}};

  double best_score = std::numeric_limits<double>::max();
  std::vector<int> best_indices;

  for (const auto& assignment : assignments)
  {
    for (const int row_sign : {-1, 1})
    {
      for (const int col_sign : {-1, 1})
      {
        const std::vector<GridProjection> projections =
            buildOrderedProjections(points,
                                    static_cast<double>(row_sign) * assignment.first,
                                    static_cast<double>(col_sign) * assignment.second);

        std::vector<int> current_indices;
        double score = std::numeric_limits<double>::max();
        if (!assignQuantizedGridBySpacing(projections,
                                          rows,
                                          cols,
                                          row_spacing,
                                          col_spacing,
                                          current_indices,
                                          score))
        {
          continue;
        }

        if (score + 1e-12 < best_score ||
            (std::fabs(score - best_score) <= 1e-12 &&
             lexicographicallyLess(current_indices, best_indices, points)))
        {
          best_score = score;
          best_indices.swap(current_indices);
        }
      }
    }
  }

  if (best_indices.size() == points.size())
  {
    ordered_indices.swap(best_indices);
    return true;
  }

  if (error) *error = "failed to assign ordered grid indices with provided spacings";
  return false;
}

inline bool sortGridIndices(const std::vector<Eigen::Vector2d>& points,
                            int rows,
                            int cols,
                            std::vector<int>& ordered_indices,
                            std::string* error = nullptr)
{
  return sortGridIndicesLegacy(points, rows, cols, ordered_indices, error);
}

inline bool sortGridCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          int rows,
                          int cols,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_cloud,
                          std::vector<int>* ordered_indices = nullptr,
                          std::string* error = nullptr)
{
  if (!cloud || !ordered_cloud)
  {
    if (error) *error = "cloud is null";
    return false;
  }

  std::vector<Eigen::Vector2d> points;
  points.reserve(cloud->size());
  for (const auto& point : cloud->points)
  {
    points.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
  }

  std::vector<int> indices;
  if (!sortGridIndices(points, rows, cols, indices, error)) return false;

  ordered_cloud->clear();
  ordered_cloud->reserve(cloud->size());
  for (int index : indices)
  {
    ordered_cloud->push_back(cloud->points[static_cast<size_t>(index)]);
  }
  if (ordered_indices) *ordered_indices = indices;
  return true;
}

}  // namespace circle_center_extract
