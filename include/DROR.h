// #pragma once

// pcl headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

/**
 * @brief class for crop box filter
 */
class DROR {
public:
  /**
   * @brief constructor
   */
  DROR();

  /**
   * @brief destructor
   */
  ~DROR();

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
  * @return azimuth_angle
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
  void Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
              pcl::PointCloud<pcl::PointXYZI>& filtered_cloud);

private:
  double radius_multiplier_, azimuth_angle_, min_neighbors_,
         min_search_radius_;
};
