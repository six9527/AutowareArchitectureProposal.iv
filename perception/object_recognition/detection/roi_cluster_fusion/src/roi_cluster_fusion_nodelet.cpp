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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <roi_cluster_fusion/roi_cluster_fusion_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <map>

#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace roi_cluster_fusion
{
Debuger::Debuger(const ros::NodeHandle & nh, const ros::NodeHandle & pnh, const int camera_num)
: nh_(nh), pnh_(pnh)
{
  image_transport_ = std::make_shared<image_transport::ImageTransport>(pnh_);
  image_buffers_.resize(camera_num);
  for (int id = 0; id < camera_num; ++id) {
    image_subs_.push_back(image_transport_->subscribe(
      "input/image_raw" + std::to_string(id), 1,
      boost::bind(&Debuger::imageCallback, this, _1, id)));
    image_pubs_.push_back(image_transport_->advertise("output/image_raw" + std::to_string(id), 1));
    image_buffers_.at(id).set_capacity(5);
  }
}

void Debuger::imageCallback(const sensor_msgs::ImageConstPtr & input_image_msg, const int id){
  image_buffers_.at(id).push_front(input_image_msg);
}

void Debuger::showImage(
  const int id, const ros::Time & time,
  const std::vector<sensor_msgs::RegionOfInterest> & image_rois,
  const std::vector<sensor_msgs::RegionOfInterest> & pointcloud_rois,
  const std::vector<Eigen::Vector2d> & points)
{
  // cv::namedWindow("ROI" + std::to_string(id), CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED);
  const boost::circular_buffer<sensor_msgs::ImageConstPtr> & image_buffer = image_buffers_.at(id);
  const image_transport::Publisher &image_pub = image_pubs_.at(id);
  for (size_t i = 0; i < image_buffer.size(); ++i) {
    if (image_buffer.at(i)->header.stamp == time) {
      cv_bridge::CvImagePtr cv_ptr;
       cv_ptr = cv_bridge::toCvCopy(image_buffer.at(i), image_buffer.at(i)->encoding);

      for(const auto & point:points){
        cv::circle(
          cv_ptr->image, cv::Point((int)point.x(), (int)point.y()), 2,
          cv::Scalar(255, 255, 255), 3, 4);
      }

      for (const auto & image_roi : image_rois) {
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset),
          cv::Point(image_roi.x_offset, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(image_roi.x_offset, image_roi.y_offset + image_roi.height),
          cv::Point(image_roi.x_offset + image_roi.width, image_roi.y_offset + image_roi.height),
          cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
      }
      for (const auto & pointcloud_roi : pointcloud_rois) {
        cv::line(
          cv_ptr->image, cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset),
          cv::Point(pointcloud_roi.x_offset + pointcloud_roi.width, pointcloud_roi.y_offset),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image, cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset),
          cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image,
          cv::Point(pointcloud_roi.x_offset + pointcloud_roi.width, pointcloud_roi.y_offset),
          cv::Point(
            pointcloud_roi.x_offset + pointcloud_roi.width,
            pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        cv::line(
          cv_ptr->image,
          cv::Point(pointcloud_roi.x_offset, pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Point(
            pointcloud_roi.x_offset + pointcloud_roi.width,
            pointcloud_roi.y_offset + pointcloud_roi.height),
          cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
      }
      image_pub.publish(cv_ptr->toImageMsg());
      // cv::imshow("ROI" + std::to_string(id), cv_ptr->image);
      // cv::waitKey(2);

      break;
    }
  }
}

RoiClusterFusionNodelet::RoiClusterFusionNodelet() {}

void RoiClusterFusionNodelet::onInit()
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<bool>("use_iou_x", use_iou_x_, true);
  private_nh_.param<bool>("use_iou_y", use_iou_y_, false);
  private_nh_.param<bool>("use_iou", use_iou_, false);
  private_nh_.param<bool>("use_cluster_semantic_type", use_cluster_semantic_type_, false);
  private_nh_.param<double>("iou_threshold", iou_threshold_, 0.1);
  private_nh_.param<int>("rois_number", rois_number_, 1);
  if (rois_number_ < 1) {
    ROS_WARN("minimum roi_num is 1. current roi_num is %d", rois_number_);
    rois_number_ = 1;
  }
  if (8 < rois_number_) {
    ROS_WARN("maximum roi_num is 8. current roi_num is %d", rois_number_);
    rois_number_ = 8;
  }

  tf_listener_ptr_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
  cluster_sub_.subscribe(private_nh_, "input/clusters", 1);
  for (int id = 0; id < rois_number_; ++id) {
    v_camera_info_sub_.push_back(
      std::make_shared<ros::Subscriber>(private_nh_.subscribe<sensor_msgs::CameraInfo>(
        "input/camera_info" + std::to_string(id), 1,
        boost::bind(&RoiClusterFusionNodelet::cameraInfoCallback, this, _1, id))));
  }
  v_roi_sub_.resize(rois_number_);
  for (int id = 0; id < (int)v_roi_sub_.size(); ++id) {
    v_roi_sub_.at(id) = std::make_shared<
      message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray>>();
    v_roi_sub_.at(id)->subscribe(private_nh_, "input/rois" + std::to_string(id), 1);
  }
  // add dummy callback to enable passthrough filter
  v_roi_sub_.at(0)->registerCallback(bind(&RoiClusterFusionNodelet::dummyCallback, this, _1));
  switch (rois_number_) {
    case 1:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), passthrough_, passthrough_, passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 2:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), passthrough_,
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 3:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        passthrough_, passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 4:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), passthrough_, passthrough_, passthrough_, passthrough_);
      break;
    case 5:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), passthrough_, passthrough_, passthrough_);
      break;
    case 6:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), passthrough_, passthrough_);
      break;
    case 7:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6), passthrough_);
      break;
    case 8:
      sync_ptr_ = std::make_shared<Sync>(
        SyncPolicy(10), cluster_sub_, *v_roi_sub_.at(0), *v_roi_sub_.at(1), *v_roi_sub_.at(2),
        *v_roi_sub_.at(3), *v_roi_sub_.at(4), *v_roi_sub_.at(5), *v_roi_sub_.at(6),
        *v_roi_sub_.at(7));
      break;
    default:
      return;
  }
  // sync_ptr_->registerCallback(boost::bind(&RoiClusterFusionNodelet::fusionCallback, this, _1, _2, _3, _4, _5, _6, _7,
  // _8, _9));
  sync_ptr_->registerCallback(std::bind(
    &RoiClusterFusionNodelet::fusionCallback, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7, std::placeholders::_8, std::placeholders::_9));
  labeled_cluster_pub_ =
    private_nh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output/labeled_clusters", 10);

  bool debug_mode;
  private_nh_.param<bool>("debug_mode", debug_mode, false);
  if (debug_mode) debuger_ = std::make_shared<Debuger>(nh_, private_nh_, rois_number_);
}

void RoiClusterFusionNodelet::cameraInfoCallback(
  const sensor_msgs::CameraInfoConstPtr & input_camera_info_msg, const int id)
{
  m_camera_info_[id] = *input_camera_info_msg;
}

void RoiClusterFusionNodelet::fusionCallback(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_cluster_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi0_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi1_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi2_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi3_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi4_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi5_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi6_msg,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr & input_roi7_msg)
{
  // Guard
  if (labeled_cluster_pub_.getNumSubscribers() < 1) return;

  // build output msg
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;
  output_msg = *input_cluster_msg;

  // reset cluster semantic type
  if (!use_cluster_semantic_type_) {
    for (auto & feature_object : output_msg.feature_objects) {
      feature_object.object.semantic.type = autoware_perception_msgs::Semantic::UNKNOWN;
      feature_object.object.semantic.confidence = 0.0;
    }
  }

  // check camera info
  for (int id = 0; id < (int)v_roi_sub_.size(); ++id) {
    // debug variable
    std::vector<sensor_msgs::RegionOfInterest> debug_image_rois;
    std::vector<sensor_msgs::RegionOfInterest> debug_pointcloud_rois;
    std::vector<Eigen::Vector2d> debug_image_points;

    // cannot find camera info
    if (m_camera_info_.find(id) == m_camera_info_.end()) {
      ROS_WARN("no camera info. id is %d", id);
      continue;
    }

    // projection matrix
    Eigen::Matrix4d projection;
    projection << m_camera_info_.at(id).P.at(0), m_camera_info_.at(id).P.at(1),
      m_camera_info_.at(id).P.at(2), m_camera_info_.at(id).P.at(3), m_camera_info_.at(id).P.at(4),
      m_camera_info_.at(id).P.at(5), m_camera_info_.at(id).P.at(6), m_camera_info_.at(id).P.at(7),
      m_camera_info_.at(id).P.at(8), m_camera_info_.at(id).P.at(9), m_camera_info_.at(id).P.at(10),
      m_camera_info_.at(id).P.at(11);

    // get transform from cluster frame id to camera optical frame id
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
        /*target*/ m_camera_info_.at(id).header.frame_id,
        /*src*/ input_cluster_msg->header.frame_id, ros::Time(0));
    } catch (tf2::TransformException & ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    // build cluster roi
    std::map<size_t, sensor_msgs::RegionOfInterest> m_cluster_roi;
    for (size_t i = 0; i < input_cluster_msg->feature_objects.size(); ++i) {
      if (input_cluster_msg->feature_objects.at(i).feature.cluster.data.empty()) continue;

      sensor_msgs::PointCloud2 transformed_cluster;
      tf2::doTransform(
        input_cluster_msg->feature_objects.at(i).feature.cluster, transformed_cluster,
        transform_stamped);

      // for reduce calc cost
      // Eigen::Vector3d centroid_point;
      // centroid_point << 0, 0, 0;
      // for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"), iter_y(transformed_cluster,
      // "y"), iter_z(transformed_cluster, "z");
      //      iter_x != iter_x.end();
      //      ++iter_x, ++iter_y, ++iter_z)
      // {
      //     centroid_point += Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
      // }
      // centroid_point *= 1.0 / (double)input_cluster_msg->feature_objects.at(i).feature.cluster.data.size();
      //     if (centroid_point.z() <= 0.0)
      //         continue;

      std::vector<Eigen::Vector2d> projected_points;
      projected_points.reserve(transformed_cluster.data.size());
      int min_x(m_camera_info_.at(id).width), min_y(m_camera_info_.at(id).height), max_x(0),
        max_y(0);
      for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
           iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
           iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (*iter_z <= 0.0) continue;
        Eigen::Vector4d projected_point = projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
        Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
          projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
        if (
          0 <= (int)normalized_projected_point.x() &&
          (int)normalized_projected_point.x() <= (int)m_camera_info_.at(id).width - 1 &&
          0 <= (int)normalized_projected_point.y() &&
          (int)normalized_projected_point.y() <= (int)m_camera_info_.at(id).height - 1) {
          min_x = std::min((int)normalized_projected_point.x(), min_x);
          min_y = std::min((int)normalized_projected_point.y(), min_y);
          max_x = std::max((int)normalized_projected_point.x(), max_x);
          max_y = std::max((int)normalized_projected_point.y(), max_y);
          projected_points.push_back(normalized_projected_point);
          debug_image_points.push_back(normalized_projected_point);
        }
      }
      if (projected_points.empty()) continue;

      sensor_msgs::RegionOfInterest roi;
      // roi.do_rectify = m_camera_info_.at(id).do_rectify;
      roi.x_offset = min_x;
      roi.y_offset = min_y;
      roi.width = max_x - min_x;
      roi.height = max_y - min_y;
      m_cluster_roi.insert(std::make_pair(i, roi));
      debug_pointcloud_rois.push_back(roi);
    }

    // calc iou
    autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr input_roi_msg;
    if (id == 0)
      input_roi_msg = input_roi0_msg;
    else if (id == 1)
      input_roi_msg = input_roi1_msg;
    else if (id == 2)
      input_roi_msg = input_roi2_msg;
    else if (id == 3)
      input_roi_msg = input_roi3_msg;
    else if (id == 4)
      input_roi_msg = input_roi4_msg;
    else if (id == 5)
      input_roi_msg = input_roi5_msg;
    else if (id == 6)
      input_roi_msg = input_roi6_msg;
    else if (id == 7)
      input_roi_msg = input_roi7_msg;
    for (size_t i = 0; i < input_roi_msg->feature_objects.size(); ++i) {
      int index;
      double max_iou = 0.0;
      for (auto m_cluster_roi_itr = m_cluster_roi.begin(); m_cluster_roi_itr != m_cluster_roi.end();
           ++m_cluster_roi_itr) {
        double iou(0.0), iou_x(0.0), iou_y(0.0);
        if (use_iou_)
          iou =
            calcIoU(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        if (use_iou_x_)
          iou_x =
            calcIoUX(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        if (use_iou_y_)
          iou_y =
            calcIoUY(m_cluster_roi_itr->second, input_roi_msg->feature_objects.at(i).feature.roi);
        if (max_iou < iou + iou_x + iou_y) {
          index = m_cluster_roi_itr->first;
          max_iou = iou + iou_x + iou_y;
        }
      }
      if (
        iou_threshold_ < max_iou &&
        output_msg.feature_objects.at(index).object.semantic.confidence <=
          input_roi_msg->feature_objects.at(i).object.semantic.confidence)
        output_msg.feature_objects.at(index).object.semantic =
          input_roi_msg->feature_objects.at(i).object.semantic;
      debug_image_rois.push_back(input_roi_msg->feature_objects.at(i).feature.roi);
    }

    debuger_->showImage(
      id, input_roi_msg->header.stamp, debug_image_rois, debug_pointcloud_rois, debug_image_points);
  }
  // publish output msg
  labeled_cluster_pub_.publish(output_msg);
}

double RoiClusterFusionNodelet::calcIoU(
  const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = (double)roi_1.width * (double)roi_1.height;
  s_2 = (double)roi_2.width * (double)roi_2.height;

  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x) * (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) return 0.0;
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUX(
  const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = (double)roi_1.width;
  s_2 = (double)roi_2.width;
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_x - overlap_min_x);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) return 0.0;
  return overlap_s / (s_1 + s_2 - overlap_s);
}
double RoiClusterFusionNodelet::calcIoUY(
  const sensor_msgs::RegionOfInterest & roi_1, const sensor_msgs::RegionOfInterest & roi_2)
{
  double s_1, s_2;
  s_1 = (double)roi_1.height;
  s_2 = (double)roi_2.height;
  double overlap_s;
  double overlap_max_x, overlap_max_y, overlap_min_x, overlap_min_y;
  overlap_min_x = roi_1.x_offset < roi_2.x_offset ? roi_2.x_offset : roi_1.x_offset;
  overlap_min_y = roi_1.y_offset < roi_2.y_offset ? roi_2.y_offset : roi_1.y_offset;
  overlap_max_x = roi_1.x_offset + roi_1.width < roi_2.x_offset + roi_2.width
                    ? roi_1.x_offset + roi_1.width
                    : roi_2.x_offset + roi_2.width;
  overlap_max_y = roi_1.y_offset + roi_1.height < roi_2.y_offset + roi_2.height
                    ? roi_1.y_offset + roi_1.height
                    : roi_2.y_offset + roi_2.height;
  overlap_s = (overlap_max_y - overlap_min_y);
  if (overlap_max_x < overlap_min_x || overlap_max_y < overlap_min_y) return 0.0;
  return overlap_s / (s_1 + s_2 - overlap_s);
}
}  // namespace roi_cluster_fusion

PLUGINLIB_EXPORT_CLASS(roi_cluster_fusion::RoiClusterFusionNodelet, nodelet::Nodelet)
