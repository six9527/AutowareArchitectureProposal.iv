/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <chrono>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "pointcloud_preprocessor/RANSACGroundFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor
{
struct PlaneBasis
{
  Eigen::Vector3d e_x;
  Eigen::Vector3d e_y;
  Eigen::Vector3d e_z;
};

struct RGB
{
  double r = 0.0;
  double g = 0.0;
  double b = 0.0;
};

class RANSACGroundFilterNodelet : public pointcloud_preprocessor::Filter
{
  using PointType = pcl::PointXYZ;

protected:
  std::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::RANSACGroundFilterConfig>> srv_;

  void filter(
    const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

  void subscribe() override;

  void unsubscribe() override;

  bool child_init(ros::NodeHandle & nh, bool & has_service) override;

  void config_callback(pointcloud_preprocessor::RANSACGroundFilterConfig & config, uint32_t level);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  ros::Publisher debug_pose_array_pub_;
  ros::Publisher debug_ground_cloud_pub_;

  std::string base_frame_ = "base_link";
  int max_iterations_ = 0;
  int min_inliers_ = 0;
  int min_points_ = 0;
  double outlier_threshold_ = 0.1;
  double plane_slope_threshold_ = 10.0;
  double height_threshold_ = 0.1;
  double voxel_size_x_ = 0.1;
  double voxel_size_y_ = 0.1;
  double voxel_size_z_ = 0.1;
  bool debug_ = false;
  bool is_initilized_debug_message_ = false;
  Eigen::Vector3d unit_vec_ = Eigen::Vector3d::UnitZ();

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */
  bool transformPointCloud(
    const std::string & in_target_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud_ptr,
    const sensor_msgs::PointCloud2::Ptr & out_cloud_ptr);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
   * in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
   * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
   */
  void extractPointsIndices(
    const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
    pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr);

  Eigen::Affine3d getPlaneAffine(
    const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal);

  void applyRANSAC(
    const pcl::PointCloud<PointType>::Ptr & input,
    pcl::PointIndices::Ptr & output_inliers,
    pcl::ModelCoefficients::Ptr & output_coefficients);

  void publishDebugMessage(
    const geometry_msgs::PoseArray & debug_pose_array,
    const pcl::PointCloud<PointType> & ground_cloud, const std_msgs::Header & header);

  void setDebugPublisher();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RANSACGroundFilterNodelet();
};
}  // namespace pointcloud_preprocessor
