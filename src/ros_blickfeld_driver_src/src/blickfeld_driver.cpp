/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Implementation for class BlickfeldDriver
 */
#include <std_msgs/Header.h>

#include "blickfeld_driver/blickfeld_driver.h"
#include "blickfeld_driver_core/blickfeld_driver_point_cloud_parser.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"
#include "blickfeld_driver_core/blickfeld_driver_utils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>  
namespace {
template <typename T>
void setMessageTime(bool use_lidar_timestamp, time_t device_time, T &ros_message) {
  if (use_lidar_timestamp == true) {
    ros_message.header.stamp.fromNSec(device_time);
  } else {
    ros_message.header.stamp = ros::Time::now();
  }
}
}  // namespace

namespace blickfeld {
namespace ros_interop {

BlickfeldDriver::BlickfeldDriver(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
    : node_handle_(node_handle), private_node_handle_(private_node_handle) {}

BlickfeldDriver::~BlickfeldDriver() { BlickfeldDriverCore::stop(); }

bool BlickfeldDriver::init() {
  ROS_INFO("Init Blickfeld ROS driver ...");

  std::string host = "";
  if (private_node_handle_.getParam("host", host) == false) {
    ROS_ERROR("Could not read parameter 'host'");
    return false;
  }

  const DeviceAlgorithmOptions device_algorithm_options = createDeviceAlgorithmOptions();
  const ImageOptions image_options = createImageOptions();
  const ImuOptions imu_options = createImuOptions();
  const PointCloudOptions point_cloud_options = createPointCloudOptions();

  point_cloud_publisher_ = private_node_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud_out", 4);

  if (image_options.ambient_image == true) {
    ambient_image_publisher_ = private_node_handle_.advertise<sensor_msgs::Image>("ambient_image_out", 1);
  }
  if (image_options.intensity_image == true) {
    intensity_image_publisher_ = private_node_handle_.advertise<sensor_msgs::Image>("intensity_image_out", 1);
  }
  if (image_options.range_image == true) {
    range_image_publisher_ = private_node_handle_.advertise<sensor_msgs::Image>("range_image_out", 1);
  }

  if (imu_options.imu_stream == true) {
    imu_publisher_ = private_node_handle_.advertise<sensor_msgs::Imu>("imu_out", 4);
  }

  diagnostics_publisher_ = private_node_handle_.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic_out", 4);

  scan_pattern_service_ =
      private_node_handle_.advertiseService("set_scan_pattern", &BlickfeldDriver::setScanPatternCallback, this);
  imu_static_tf_service_ = private_node_handle_.advertiseService("publish_imu_static_tf",
                                                                 &BlickfeldDriver::publishImuStaticTFCallback, this);

  BlickfeldDriverCore::init(host, point_cloud_options, device_algorithm_options, image_options, imu_options);
  BlickfeldDriverCore::start();

  return true;
}
//TODO:publish pointcloud
void BlickfeldDriver::publishPointCloud(sensor_msgs::PointCloud2Ptr point_cloud, time_t device_time) {
  setMessageTime(use_lidar_timestamp_, device_time, *point_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*point_cloud, *laser_cloud_out);

  Eigen::Matrix3d rotation_matrix1 = Eigen::Matrix3d::Identity();
  rotation_matrix1 << 0.503, 0.863, 0.00,
                      -0.863, 0.503, 0.00,
                      1.000, 1.000, 1.000;
  Eigen::Vector3d t1;
  t1 <<  -2.05, 1.389, 0.436;

 
  Eigen::Matrix4d a1;
  a1.setIdentity();
  a1.block<3,3>(0,0) = rotation_matrix1;
  a1.topRightCorner<3, 1>() = t1;

  Eigen::Matrix3d rotation_matrix2 = Eigen::Matrix3d::Identity();
  rotation_matrix2 << 0.999, 0.863, 0.00,
                      -0.027, 0.999, -0.031,
                      0.035, 0.032, 0.998;
  Eigen::Vector3d t2;
  t2 << -0.466, 0.352, -0.417;

  Eigen::Matrix4d a2;
  a2.setIdentity();
  a2.block<3,3>(0,0) = rotation_matrix2;
  a2.topRightCorner<3, 1>() = t2;

  Eigen::Matrix3d rotation_matrix3 = Eigen::Matrix3d::Identity();
  rotation_matrix3 << 0.999, -0.019, -0.003,
                      0.019, 0.999, 0.005,
                      0.003, -0.005, 0.999;
  Eigen::Vector3d t3;
  t3 << 0.073, -0.002, 0.057;

  Eigen::Matrix4d a3;
  a3.setIdentity();
  a3.block<3,3>(0,0) = rotation_matrix3;
  a3.topRightCorner<3, 1>() = t3;

  Eigen::Matrix4d a;
  a.setIdentity();
  a=a3*a2*a1;

  Eigen::Affine3d transOffset;
  transOffset.matrix()=a;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*laser_cloud_out, *point_cloud_out, transOffset);
  sensor_msgs::PointCloud2 tempCloud;
  pcl::toROSMsg(*point_cloud_out, tempCloud);
  point_cloud_publisher_.publish(tempCloud);
}

void BlickfeldDriver::publishStatus(diagnostic_msgs::DiagnosticStatusPtr status) {
  diagnostics_publisher_.publish(status);
}

void BlickfeldDriver::publishRangeImage(sensor_msgs::ImagePtr range_image, time_t device_time) {
  setMessageTime(use_lidar_timestamp_, device_time, *range_image);
  range_image_publisher_.publish(range_image);
}

void BlickfeldDriver::publishIntensityImage(sensor_msgs::ImagePtr intensity_image, time_t device_time) {
  setMessageTime(use_lidar_timestamp_, device_time, *intensity_image);
  intensity_image_publisher_.publish(intensity_image);
}

void BlickfeldDriver::publishAmbientImage(sensor_msgs::ImagePtr ambient_image, time_t device_time) {
  setMessageTime(use_lidar_timestamp_, device_time, *ambient_image);
  ambient_image_publisher_.publish(ambient_image);
}

void BlickfeldDriver::publishImu(sensor_msgs::ImuPtr imu, time_t device_time) {
  setMessageTime(use_lidar_timestamp_, device_time, *imu);
  imu_publisher_.publish(imu);
}

void BlickfeldDriver::publishImuStaticTF(TransformMsgStampedPtr transform_msg_stamped) {
  static_broadcaster_.sendTransform(*transform_msg_stamped);
}

void BlickfeldDriver::printLogMessages(const LogMessages &log_messages) {
  for (const auto &log_message : log_messages) {
    switch (log_message.first) {
      case (LogLevel::Critical): {
        ROS_ERROR_STREAM(log_message.second.str());
        break;
      }
      case (LogLevel::Warning): {
        ROS_WARN_STREAM(log_message.second.str());
        break;
      }
      case (LogLevel::Debug): {
        ROS_DEBUG_STREAM(log_message.second.str());
        break;
      }
    }
  }
}

bool BlickfeldDriver::setScanPatternCallback(blickfeld_driver::SetScanPatternRequest &service_request,
                                             blickfeld_driver::SetScanPatternResponse &service_response) {
  SetScanPatternResponse response = setScanPattern(reinterpret_cast<SetScanPatternRequest &>(service_request));
  service_response = reinterpret_cast<blickfeld_driver::SetScanPatternResponse &>(response);
  return true;
}

bool BlickfeldDriver::publishImuStaticTFCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &) {
  publishImuStaticTF(getStaticTF());
  return true;
}

PointCloudOptions BlickfeldDriver::createPointCloudOptions() {
  PointCloudOptions point_cloud_options;
  if (private_node_handle_.getParam("publish_intensities", point_cloud_options.intensities) == false) {
    ROS_WARN("Could not read parameter 'publish_intensities', using default: %s",
             point_cloud_options.intensities == true ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_ambient_light", point_cloud_options.ambient_light) == false) {
    ROS_WARN("Could not read parameter 'publish_ambient_light', using default: %s",
             point_cloud_options.ambient_light == true ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_explicit_range", point_cloud_options.range) == false) {
    ROS_WARN("Could not read parameter 'publish_explicit_range', using default: %s",
             point_cloud_options.range == true ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_no_return_points", point_cloud_options.no_return_points) == false) {
    ROS_WARN("Could not read parameter 'publish_no_return_points', using default: %s",
             point_cloud_options.no_return_points == true ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_point_time_offset", point_cloud_options.point_time_offset) == false) {
    ROS_WARN("Could not read parameter 'publish_point_time_offset', using default: %s",
             point_cloud_options.point_time_offset == true ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_point_id", point_cloud_options.point_id) == false) {
    ROS_WARN("Could not read parameter 'publish_point_id', using default: %s",
             point_cloud_options.point_id ? "true" : "false");
  }
  if (private_node_handle_.getParam("use_lidar_timestamp", use_lidar_timestamp_) == false) {
    ROS_INFO("Time stamp will be created from %s",
             (use_lidar_timestamp_ == true) ? "LiDAR timestamp" : "ROS timeserver");
  };
  if (point_cloud_options.no_return_points == true) {
    if (private_node_handle_.getParam("no_return_point_range", point_cloud_options.no_return_point_range) == false) {
      ROS_WARN("Could not read parameter 'no_return_point_range', using %f", point_cloud_options.no_return_point_range);
    }
  }
  if (private_node_handle_.getParam("lidar_frame_id", point_cloud_options.lidar_frame_id) == false) {
    ROS_WARN("Could not read parameter 'lidar_frame_id', using default: %s",
             point_cloud_options.lidar_frame_id.c_str());
  }
  std::string returns_publishing_options_str;
  if (private_node_handle_.getParam("returns_publishing_options", returns_publishing_options_str) == false) {
    ROS_WARN("Could not read parameter 'return_publishing_options_', using default: %s",
             driver_utilities::returnOptionsToString(point_cloud_options.return_options).c_str());
  } else {
    try {
      point_cloud_options.return_options = driver_utilities::returnOptionsFromString(returns_publishing_options_str);
    } catch (const std::invalid_argument &exe) {
      ROS_WARN("Could not parse parameter 'return_publishing_options_', using default: %s",
               driver_utilities::returnOptionsToString(point_cloud_options.return_options).c_str());
    }
  }
  return point_cloud_options;
}

ImageOptions BlickfeldDriver::createImageOptions() {
  ImageOptions image_options;
  if (private_node_handle_.getParam("publish_range_image", image_options.range_image) == false) {
    ROS_WARN("Could not read parameter 'publish_range_image', using default: %s",
             image_options.range_image ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_ambient_image", image_options.ambient_image) == false) {
    ROS_WARN("Could not read parameter 'publish_ambient_image', using default: %s",
             image_options.ambient_image ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_intensity_image", image_options.intensity_image) == false) {
    ROS_WARN("Could not read parameter 'publish_intensity_image', using default: %s",
             image_options.intensity_image ? "true" : "false");
  }

  std::string projection_type_str;
  if (private_node_handle_.getParam("projection_type", projection_type_str) == false) {
    ROS_WARN_STREAM("Could not read parameter 'projection_type', using default: "
                    << driver_utilities::projectionTypeToString(image_options.projection_type));
  }
  try {
    image_options.projection_type = driver_utilities::projectionTypeFromString(projection_type_str);
  } catch (const std::invalid_argument &exe) {
    ROS_WARN_STREAM(exe.what() << ". Could not parse parameter 'projection_type', using default: "
                               << driver_utilities::projectionTypeToString(image_options.projection_type));
  }

  return image_options;
}

DeviceAlgorithmOptions BlickfeldDriver::createDeviceAlgorithmOptions() {
  DeviceAlgorithmOptions device_algorithm_options;

  if (private_node_handle_.getParam("use_background_subtraction",
                                    device_algorithm_options.use_background_subtraction) == false) {
    ROS_WARN("Could not read parameter 'use_background_subtraction', using default: %s",
             device_algorithm_options.use_background_subtraction ? "true" : "false");
  }
  if (private_node_handle_.getParam("use_neighbor_filter", device_algorithm_options.use_neighbor_filter) == false) {
    ROS_WARN("Could not read parameter 'use_neighbor_filter', using default: %s",
             device_algorithm_options.use_neighbor_filter ? "true" : "false");
  }
  if (private_node_handle_.getParam("background_subtraction_num_initialization_frames",
                                    device_algorithm_options.background_subtraction_num_initialization_frames) ==
      false) {
    ROS_WARN("Could not read parameter 'background_subtraction_num_initialization_frames', using default: %d",
             device_algorithm_options.background_subtraction_num_initialization_frames);
  }
  if (private_node_handle_.getParam("background_subtraction_exponential_decay_rate",
                                    device_algorithm_options.background_subtraction_exponential_decay_rate) == false) {
    ROS_WARN("Could not read parameter 'background_subtraction_exponential_decay_rate', using default: %f",
             device_algorithm_options.background_subtraction_exponential_decay_rate);
  }

  return device_algorithm_options;
}

ImuOptions BlickfeldDriver::createImuOptions() {
  ImuOptions imu_options;

  if (private_node_handle_.getParam("lidar_frame_id", imu_options.lidar_frame_id) == false) {
    ROS_WARN("Could not read parameter 'lidar_frame_id', using default: %s", imu_options.lidar_frame_id.c_str());
  }
  if (private_node_handle_.getParam("publish_imu", imu_options.imu_stream) == false) {
    ROS_WARN("Could not read parameter 'publish_imu', using default: %s", imu_options.imu_stream ? "true" : "false");
  }
  if (private_node_handle_.getParam("publish_imu_static_tf", imu_options.imu_static_tf) == false) {
    ROS_WARN("Could not read parameter 'publish_imu_static_tf', using default: %s",
             imu_options.imu_static_tf ? "true" : "false");
  }

  return imu_options;
}

}  // namespace ros_interop
}  // namespace blickfeld
