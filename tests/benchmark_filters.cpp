#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/time.h>
#include <random>
#include <iostream>
#include "DROR.h"

TEST(FilterBenchmark, CompareProcessingTimes)
{
    // Simulate a Velodyne VLP-32C
    // VLP-32C generates ~600,000 points per second. At 10Hz, this is ~60,000 points per scan.
    const int num_points = 60000;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    cloud->reserve(num_points);

    std::default_random_engine generator(42); // Fixed seed for reproducibility
    std::uniform_real_distribution<float> dist_xy(-20.0f, 20.0f);
    std::uniform_real_distribution<float> dist_z(-3.0f, 3.0f);
    std::uniform_real_distribution<float> dist_intensity(0.0f, 255.0f);

    for (int i = 0; i < num_points; ++i)
    {
        pcl::PointXYZI pt;
        pt.x = dist_xy(generator);
        pt.y = dist_xy(generator);
        pt.z = dist_z(generator);
        pt.intensity = dist_intensity(generator);
        cloud->push_back(pt);
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_out_dror;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_ror(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out_sor(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::console::TicToc tt;

    // 1. DROR Filter
    DROR<pcl::PointXYZI> dror;
    dror.SetRadiusMultiplier(3.0);
    dror.SetAzimuthAngle(0.04);
    dror.SetMinNeighbors(3);
    dror.SetMinSearchRadius(0.04);

    tt.tic();
    dror.Filter(cloud, cloud_out_dror);
    double time_dror = tt.toc();
    std::cout << "[          ] DROR processing time:     " << time_dror << " ms (" << cloud_out_dror.size() << " points kept)\n";

    // 2. PCL ROR Filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(3);

    tt.tic();
    ror.filter(*cloud_out_ror);
    double time_ror = tt.toc();
    std::cout << "[          ] PCL ROR processing time:  " << time_ror << " ms (" << cloud_out_ror->size() << " points kept)\n";

    // 3. PCL SOR Filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);

    tt.tic();
    sor.filter(*cloud_out_sor);
    double time_sor = tt.toc();
    std::cout << "[          ] PCL SOR processing time:  " << time_sor << " ms (" << cloud_out_sor->size() << " points kept)\n";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}