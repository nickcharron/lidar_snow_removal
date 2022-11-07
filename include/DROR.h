#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief class for crop box filter
 */
class DROR
{
public:
  /**
   * @brief default constructor
   */
  DROR() = default;

  /**
   * @brief default destructor
   */
  ~DROR() = default;

  /**
   * @brief Method for setting radius multiplier
   * @param radius_multiplier
   */
  void SetRadiusMultiplier(double radius_multiplier);

  /**
   * @brief Method for retrieving radius multiplier
   * @return radius_multiplier default = 3. This should be greater than 1
   */
  double GetRadiusMultiplier();

  /**
   * @brief Method for setting azimuth angle
   * @param azimuth_angle default = 0.04 degees
   */
  void SetAzimuthAngle(double azimuth_angle);

  /**
   * @brief Method for getting azimuth angle in degrees
   * @return azimuth_angle in degrees
   */
  double GetAzimuthAngle();

  /**
   * @brief Method for setting minimum number of neighbors which includes the
   * point that is being filtered (3 means 2 neighbors)
   * @param min_neighbors default = 3
   */
  void SetMinNeighbors(double min_neighbors);

  /**
   * @brief Method for retrieving minimum number of neighbors
   * @return min_neighbors
   */
  double GetMinNeighbors();

  /**
   * @brief Method for setting the minimum search radius
   * @param min_search_radius default = 0.04
   */
  void SetMinSearchRadius(double min_search_radius);

  /**
   * @brief Method for retrieving the minimum search radius
   * @return min_search_radius
   */
  double GetMinSearchRadius();

  /**
   * @brief Method for applying the filter
   * @param input_cloud cloud to be filtered
   * @param filtered_cloud cloud to save to
   */

  template <typename T>
  void Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
              typename pcl::PointCloud<T> &filtered_cloud);

private:
  double radius_multiplier_{3};
  double azimuth_angle__rad_{0.04 * M_PI / 180};
  double min_neighbors_{3};
  double min_search_radius_{0.04};
};

template <typename T>
void DROR::Filter(typename pcl::PointCloud<T>::Ptr &input_cloud,
                  typename pcl::PointCloud<T> &filtered_cloud)
{
  using KdTreePtr = typename pcl::KdTreeFLANN<T>::Ptr;
  //    Clear points in output cloud
  filtered_cloud.clear();

  // init. kd search tree
  KdTreePtr kd_tree_(new pcl::KdTreeFLANN<T>());
  kd_tree_->setInputCloud(input_cloud);

  // Go over all the points and check which doesn't have enough neighbors
  // perform filtering
  for (typename pcl::PointCloud<T>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it)
  {
    float x_i = it->x;
    float y_i = it->y;
    float range_i = sqrt(pow(x_i, 2) + pow(y_i, 2));
    float search_radius_dynamic = radius_multiplier_ * 2 * range_i * sin(azimuth_angle__rad_);

    if (search_radius_dynamic < min_search_radius_)
    {
      search_radius_dynamic = min_search_radius_;
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors =
        kd_tree_->radiusSearch(*it, search_radius_dynamic, pointIdxRadiusSearch,
                               pointRadiusSquaredDistance);

    if (neighbors >= min_neighbors_)
    {
      filtered_cloud.push_back(*it);
    }
  }
}