/* Copyright (c) 2017, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: remove_ground.hpp
 * Desc: Header file for ground filter related to anm_lidar_matcher.
 * Auth: Benjamin Skikos <bapskiko@uwaterloo.ca>
 *       Leonid Koppel <lkoppel@uwaterloo.ca>
 *       Pranav Ganti <pganti@uwaterloo.ca>
 *
 * ############################################################################
*/
#ifndef ANM_LIDAR_MATCHER_REMOVE_GROUND_HPP
#define ANM_LIDAR_MATCHER_REMOVE_GROUND_HPP

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <wave/matching/ground_segmentation_params.hpp>

namespace anm_lidar_matcher {

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/** Keeps only non-ground points
 *
 * Uses wave::GroundSegmentation, which is based on Chen, Tongtong, et al.
 * "Gaussian-process-based real-time ground segmentation for autonomous land
 * vehicles." (https://link.springer.com/article/10.1007/s10846-013-9889-4)
 *
 * @param input   input point cloud
 * @param params  settings for GroundSegmentation
 * @param output  output point cloud, only non-ground points
 */
void removeGround(const wave::GroundSegmentationParams &params,
                  const PointCloud::ConstPtr &input,
                  const PointCloud::Ptr &output);

}  // namespace anm_lidar_matcher

#endif  // ANM_LIDAR_MATCHER_REMOVE_GROUND_HPP
