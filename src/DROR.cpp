#include "DROR.h"

typedef pcl::KdTreeFLANN<pcl::PointXYZI> KdTree;
typedef KdTree::Ptr KdTreePtr;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

DROR::DROR(){
  radius_multiplier_=3;
  azimuth_angle_=0.04;
  min_neighbors_=3;
  min_search_radius_=0.04;
}

DROR::~DROR(){
  //
}

void DROR::SetRadiusMultiplier(double radius_multiplier) {
  radius_multiplier_ = radius_multiplier;
}

double DROR::GetRadiusMultiplier() {
  return radius_multiplier_;
}

void DROR::SetAzimuthAngle(double azimuth_angle) {
  azimuth_angle_ = azimuth_angle;
}

double DROR::GetAzimuthAngle() {
  return azimuth_angle_;
}

void DROR::SetMinNeighbors(double min_neighbors) {
  min_neighbors_ = min_neighbors;
}

double DROR::GetMinNeighbors() {
  return min_neighbors_;
}

void DROR::SetMinSearchRadius(double min_search_radius) {
  min_search_radius_ = min_search_radius;
}

double DROR::GetMinSearchRadius() {
  return min_search_radius_;
}

void DROR::Filter(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                  pcl::PointCloud<pcl::PointXYZI>& filtered_cloud) {
  // Clear points in output cloud
  filtered_cloud.clear();

  // init. kd search tree
  KdTreePtr kd_tree_(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kd_tree_->setInputCloud(input_cloud);

  // Go over all the points and check which doesn't have enough neighbors
  // perform filtering
  for (pcl::PointCloud<pcl::PointXYZI>::iterator it = input_cloud->begin();
       it != input_cloud->end(); ++it) {
    float x_i = it->x;
    float y_i = it->y;
    float range_i = sqrt(pow(x_i, 2) + pow(y_i, 2));
    float search_radius_dynamic =
        radius_multiplier_ * azimuth_angle_ * 3.14159265359 / 180 * range_i;

    if (search_radius_dynamic < min_search_radius_) {
      search_radius_dynamic = min_search_radius_;
    }

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    int neighbors =
        kd_tree_->radiusSearch(*it, search_radius_dynamic, pointIdxRadiusSearch,
                               pointRadiusSquaredDistance);

    if (neighbors >= min_neighbors_) { filtered_cloud.push_back(*it); }
  }
}
