/**
 * @file
 * @copyright Copyright (C) 2020, Blickfeld GmbH
 * All rights reserved.
 *
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree.
 *
 * Header for Blickfeld Driver Utilities
 */

#pragma once

#include <string>

#include <blickfeld/scanner.h>

#include "blickfeld_driver/blickfeld_driver_ros_types.h"
#include "blickfeld_driver_core/blickfeld_driver_types.h"

namespace blickfeld {
namespace ros_interop {
namespace driver_utilities {

/**
 * @brief getFlatPixelCoordinates computes the pixel coordinate of a point in a scan pattern
 *
 * @param[in] point_index
 * @param[in] scanline
 * @param[in] scan_pattern
 * @param[in] projection_type Angle_preserving or Scanline_preserving
 * @return flatten pixel coordinate (1D) or -1 when the pixel coordinate and the scan pattern does not match
 */
int getFlatPixelCoordinates(int point_index, const blickfeld::protocol::data::Scanline &scanline,
                            const ScanPatternT &scan_pattern, const ProjectionType &projection_type);

/**
 * @brief getNumberOfScanlines returns number of scanlines of a scan pattern
 *
 * @param[in] scan_pattern
 * @return number of scanlines
 */
size_t getNumberOfScanlines(const ScanPatternT &scan_pattern);

/**
 * @brief getImageSize
 *
 * @param[in] scan_pattern
 * @return size of the image
 */
cv::Size getImageSize(const ScanPatternT &scan_pattern);

/**
 * @brief cvMatToRosImageMessage is a helper to make a ros sensor message image from a cv::Mat
 *
 * @param[in] cv_mat_image
 * @param[in] encoding
 * @param[in] frame_id
 * @return ros image message
 */
SensorMsgImagePtr cvMatToRosImageMessage(const cv::Mat &cv_mat_image, const std::string &encoding,
                                         const std::string &frame_id);

/**
 * @brief projectionTypeFromString converts @p string to a ProjectionType, it would throw an exception in case of
 * undefined string
 *
 * @param[in] string input string
 * @return projection_type
 */
ProjectionType projectionTypeFromString(const std::string &string);

/**
 * @brief projectionTypeToString converts @p projection_type to a string, it would throw an exception in case of
 * undefined projection_type
 *
 * @param[in] projection_type
 * @return std::string
 */
std::string projectionTypeToString(const ProjectionType &projection_type);

/**
 * @brief returnOptionsFromString converts @p string to a ReturnOptions, it would throw an exception in case
 * of undefined string
 *
 * @param[in] string
 * @return return options
 */
ReturnOptions returnOptionsFromString(const std::string &string);

/**
 * @brief returnOptionsToString converts @p return_options to a string, it would throw an exception in case of
 * undefined projection_type
 *
 * @param[in] return_options
 * @return std::string
 */
std::string returnOptionsToString(const ReturnOptions &return_options);

/**
 * @brief publishImage checks the @p image_option and return true in case at least one of the image publishing options
 * is true
 *
 * @param[in] image_options
 * @return true in case at least one of the image publishing options is true
 */
bool publishImage(const ImageOptions &image_options);

}  // namespace driver_utilities
}  // namespace ros_interop
}  // namespace blickfeld
