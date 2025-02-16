/**
 * @file MultiSenseUtilities.hh
 *
 * Copyright 2013-2025
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Significant history (date, user, job code, action):
 *   2025-01-15, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "MultiSenseTypes.hh"

namespace multisense
{

///
/// Make sure our Points and point clouds are packed for applications which might need to handle
/// the underlying raw data
///
#pragma pack(push, 1)

template<typename Color>
struct Point
{
    float x = 0;
    float y = 0;
    float z = 0;
    Color color;
};

template<>
struct Point<void>
{
    float x = 0;
    float y = 0;
    float z = 0;
};

template<typename Color = void>
struct PointCloud
{
    std::vector<Point<Color>> cloud;
};

#pragma pack(pop)

///
/// @brief Convert a status object to a user readable string
///
MULTISENSE_API std::string to_string(const Status &status);

///
/// @brief Write a image to a specific path on disk. The type of serialization is determined by the
///        input path
///
MULTISENSE_API bool write_image(const Image &image, const std::filesystem::path &path);

///
/// @brief Create a depth image from a image frame
///
/// @param depth_format Supported formats include MONO16 and FLOAT32. Note MONO16 will be quantized to millimeters)
/// @param invalid_value The value to set invalid depth measurements to. (i.e. points where disparity = 0)
///
MULTISENSE_API std::optional<Image> create_depth_image(const ImageFrame &frame,
                                                       const Image::PixelFormat &depth_format,
                                                       const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW,
                                                       int32_t invalid_value = 65535);

///
/// @brief Convert a YCbCr420 luma + chroma image into a BGR color image
///
MULTISENSE_API std::optional<Image> create_bgr_image(const Image &luma, const Image &chroma, const DataSource &output_source);

///
/// @brief Convert a YCbCr420 luma + chroma image into a BGR color image
///
MULTISENSE_API std::optional<Image> create_bgr(const ImageFrame &frame, const DataSource &output_source);

///
/// @brief Create a point cloud from a image frame and a color source.
///
template<typename Color>
MULTISENSE_API std::optional<PointCloud<Color>> create_color_pointcloud(const ImageFrame &frame,
                                                                        double max_range,
                                                                        const DataSource &color_source = DataSource::UNKNOWN,
                                                                        const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW)
{
    Image placeholder{};
    std::reference_wrapper<const Image> disparity = std::cref(placeholder);
    std::reference_wrapper<const Image> color = std::cref(placeholder);

    size_t color_step = 0;
    double color_disparity_scale = 0.0;

    if constexpr (std::is_same_v<Color, void>)
    {
        if (!frame.has_image(disparity_source))
        {
            return std::nullopt;
        }

        disparity = frame.get_image(disparity_source);

        if (disparity.get().format != Image::PixelFormat::MONO16 ||
            disparity.get().width < 0 ||
            disparity.get().height < 0)
        {
            return std::nullopt;
        }
    }
    else
    {
        color_step = sizeof(Color);

        if (!frame.has_image(color_source) || !frame.has_image(disparity_source))
        {
            return std::nullopt;
        }

        disparity = frame.get_image(disparity_source);
        color = frame.get_image(color_source);

        if (disparity.get().format != Image::PixelFormat::MONO16 ||
            color.get().width != disparity.get().width ||
            color.get().height != disparity.get().height ||
            disparity.get().width < 0 ||
            disparity.get().height < 0)
        {
            return std::nullopt;
        }

        const double tx = frame.calibration.right.P[0][3] / frame.calibration.right.P[0][0];
        const double color_tx = color.get().calibration.P[0][3] / color.get().calibration.P[0][0];
        color_disparity_scale = color_tx / tx;
    }

    constexpr double scale = 1.0 / 16.0;

    const double squared_range = max_range * max_range;

    const double fx = disparity.get().calibration.P[0][0];
    const double fy = disparity.get().calibration.P[1][1];
    const double cx = disparity.get().calibration.P[0][2];
    const double cy = disparity.get().calibration.P[1][2];
    const double tx = frame.calibration.right.P[0][3] / frame.calibration.right.P[0][0];
    const double cx_prime = frame.calibration.right.P[0][2];

    const double fytx = fy * tx;
    const double fxtx = fx * tx;

    const double fycxtx = fy * cx * tx;
    const double fxcytx = fx * cy * tx;
    const double fxfytx = fx * fy * tx;
    const double fycxcxprime = fy * (cx - cx_prime);

    PointCloud<Color> output;
    output.cloud.reserve(disparity.get().width * disparity.get().height);

    for (size_t h = 0 ; h < static_cast<size_t>(disparity.get().height) ; ++h)
    {
        for (size_t w = 0 ; w < static_cast<size_t>(disparity.get().width) ; ++w)
        {
            const size_t index = disparity.get().image_data_offset +
                                 (h * disparity.get().width * sizeof(uint16_t)) +
                                 (w * sizeof(uint16_t));

            const double d =
                static_cast<double>(*reinterpret_cast<const uint16_t*>(disparity.get().raw_data->data() + index)) * scale;

            if (d == 0.0)
            {
                continue;
            }

            const double inversebeta = 1.0 / (-fy * d + fycxcxprime);
            const double x = ((fytx * w) + (-fycxtx)) * inversebeta;
            const double y = ((fxtx * h) + (-fxcytx)) * inversebeta;
            const double z = fxfytx * inversebeta;

            if ((x*x + y*y + z*z) > squared_range)
            {
                continue;
            }

            if constexpr (std::is_same_v<Color, void>)
            {
                output.cloud.push_back(Point<Color>{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
            }
            else
            {
                //
                // Use the approximation that color_pixel_u = disp_u - (tx_color/ tx) * d
                //
                const size_t color_index = color.get().image_data_offset +
                                           (h * color.get().width * color_step) +
                                           static_cast<size_t>((static_cast<double>(w) - (color_disparity_scale * d))) * color_step;

                const Color color_pixel = *reinterpret_cast<const Color*>(color.get().raw_data->data() + color_index);

                output.cloud.push_back(Point<Color>{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z),
                                                    color_pixel});
            }
        }
    }

    return output;
}

MULTISENSE_API std::optional<PointCloud<void>> create_pointcloud(const ImageFrame &frame,
                                                                 double max_range,
                                                                 const DataSource &disparity_source = DataSource::LEFT_DISPARITY_RAW);

///
/// @brief Write a point cloud to a ASCII ply file
///
template <typename Color>
MULTISENSE_API bool write_pointcloud_ply(const PointCloud<Color> &point_cloud, const std::filesystem::path &path)
{
    std::stringstream ss;

    ss << "ply\n";
    ss << "format ascii 1.0\n";
    ss << "element vertex " << point_cloud.cloud.size() << "\n";
    ss << "property float x\n";
    ss << "property float y\n";
    ss << "property float z\n";

    if constexpr (std::is_same_v<Color, uint8_t>)
    {
        ss << "property uchar gray\n";
    }
    else if constexpr (std::is_same_v<Color, uint16_t>)
    {
        ss << "property ushort gray\n";
    }
    else if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
    {
        ss << "property uchar red\n";
        ss << "property uchar green\n";
        ss << "property uchar blue\n";
    }
    else if (!std::is_same_v<Color, void>)
    {
        throw std::runtime_error("Unsupported color type");
    }

    ss << "end_header\n";

    for (const auto &point : point_cloud.cloud)
    {
        if constexpr (std::is_same_v<Color, std::array<uint8_t, 3>>)
        {
            //
            // Our points are in BGR ordering, convert to RGB
            //
            ss << point.x << " " <<
                  point.y << " " <<
                  point.z << " " <<
                  static_cast<uint32_t>(point.color[2]) << " " <<
                  static_cast<uint32_t>(point.color[1]) << " " <<
                  static_cast<uint32_t>(point.color[0]) << "\n";
        }
        else if constexpr(std::is_same_v<Color, void>)
        {
            ss << point.x << " " << point.y << " " << point.z << "\n";
        }
        else
        {
            ss << point.x << " " << point.y << " " << point.z << " " << static_cast<uint32_t>(point.color) << "\n";
        }
    }

    std::ofstream ply(path.c_str());
    if (!ply.good())
    {
        return false;
    }

    ply << ss.str();

    return true;
}

}
