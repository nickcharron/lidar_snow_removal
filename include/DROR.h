#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief class for crop box filter
 */
template <typename T>
class DROR
{
public:
  /**
   * @brief default constructor
   */
  DROR()
  {
    kd_tree_.reset(new pcl::KdTreeFLANN<T>());
  }

  /**
   * @brief default destructor
   */
  ~DROR() = default;

  /**
   * @brief Method for setting radius multiplier
   * @param radius_multiplier
   */
  void SetRadiusMultiplier(double radius_multiplier) { radius_multiplier_ = radius_multiplier; }

  /**
   * @brief Method for retrieving radius multiplier
   * @return radius_multiplier default = 3. This should be greater than 1
   */
  double GetRadiusMultiplier() { return radius_multiplier_; }

  /**
   * @brief Method for setting azimuth angle
   * @param azimuth_angle default = 0.04 degees
   */
  void SetAzimuthAngle(double azimuth_angle_deg) { azimuth_angle__rad_ = azimuth_angle_deg * M_PI / 180; }

  /**
   * @brief Method for getting azimuth angle in degrees
   * @return azimuth_angle in degrees
   */
  double GetAzimuthAngle() { return azimuth_angle__rad_ * 180 / M_PI; }

  /**
   * @brief Method for setting minimum number of neighbors which includes the
   * point that is being filtered (3 means 2 neighbors)
   * @param min_neighbors default = 3
   */
  void SetMinNeighbors(double min_neighbors) { min_neighbors_ = min_neighbors; }

  /**
   * @brief Method for retrieving minimum number of neighbors
   * @return min_neighbors
   */
  double GetMinNeighbors() { return min_neighbors_; }

  /**
   * @brief Method for setting the minimum search radius
   * @param min_search_radius default = 0.04
   */
  void SetMinSearchRadius(double min_search_radius) { min_search_radius_ = min_search_radius; }

  /**
   * @brief Method for retrieving the minimum search radius
   * @return min_search_radius
   */
  double GetMinSearchRadius() { return min_search_radius_; }

  /**
   * @brief Method for applying the filter
   * @param input_cloud cloud to be filtered
   * @param filtered_cloud cloud to save to
   */

  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud);

private:
  double radius_multiplier_{3};
  double azimuth_angle__rad_{0.04 * M_PI / 180};
  double min_neighbors_{3};
  double min_search_radius_{0.04};

  typename pcl::KdTreeFLANN<T>::Ptr kd_tree_;
};

template <typename T>
void DROR<T>::Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
                     typename pcl::PointCloud<T> &filtered_cloud)
{
  if (input_cloud->empty())
  {
    filtered_cloud.clear();
    return;
  }

  // 1. Pre-calculate the constant part of the dynamic radius formula
  // Formula: radius = multiplier * 2 * range * sin(azimuth)
  const double radius_const = radius_multiplier_ * 2.0 * std::sin(azimuth_angle__rad_);

  // 2. Initialize KD-Tree
  kd_tree_->setInputCloud(input_cloud);

  const int cloud_size = static_cast<int>(input_cloud->size());

  // Mask to track which points to keep (0 = discard, 1 = keep)
  // Using vector<int> because vector<bool> is not thread-safe for writes
  std::vector<int> keep_mask(cloud_size, 0);

// 3. Parallel loop using OpenMP
#pragma omp parallel
  {
    // Thread-local buffers to prevent constant re-allocation on the heap
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

#pragma omp for
    for (int i = 0; i < cloud_size; ++i)
    {
      const T &point = input_cloud->points[i];

      // Optimization: Use x*x + y*y instead of pow()
      float range_i = std::sqrt(point.x * point.x + point.y * point.y);

      float search_radius_dynamic = static_cast<float>(radius_const * range_i);

      if (search_radius_dynamic < min_search_radius_)
      {
        search_radius_dynamic = static_cast<float>(min_search_radius_);
      }

      // Perform radius search
      // We use the thread-local vectors here
      int neighbors = kd_tree_->radiusSearch(point, search_radius_dynamic,
                                             pointIdxRadiusSearch,
                                             pointRadiusSquaredDistance);

      if (neighbors >= min_neighbors_)
      {
        keep_mask[i] = 1;
      }
    }
  }

  filtered_cloud.clear();
  // Pre-allocate memory to avoid multiple reallocations during push_back
  filtered_cloud.reserve(cloud_size / 2);

  for (int i = 0; i < cloud_size; ++i)
  {
    if (keep_mask[i])
    {
      filtered_cloud.push_back(input_cloud->points[i]);
    }
  }
}