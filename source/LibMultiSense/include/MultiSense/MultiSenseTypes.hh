/**
 * @file LibMultiSense/MultiSenseTypes.hh
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace multisense
{

///
/// @brief Identifies which camera or data source the image is from
///
enum class DataSource
{
    UNKNOWN,
    ALL,
    LEFT_MONO_RAW,
    RIGHT_MONO_RAW,
    LEFT_MONO_COMPRESSED,
    RIGHT_MONO_COMPRESSED,
    LEFT_RECTIFIED_RAW,
    RIGHT_RECTIFIED_RAW,
    LEFT_RECTIFIED_COMPRESSED,
    RIGHT_RECTIFIED_COMPRESSED,
    LEFT_DISPARITY_RAW,
    LEFT_DISPARITY_COMPRESSED,
    AUX_COMPRESSED,
    AUX_RECTIFIED_COMPRESSED,
    AUX_LUMA_RAW,
    AUX_LUMA_RECTIFIED_RAW,
    AUX_CHROMA_RAW,
    AUX_CHROMA_RECTIFIED_RAW,
    COST_RAW
};

struct CameraCalibration
{
    ///
    /// @brief Distortion type
    ///
    enum class DistortionType
    {
        NONE,
        PLUMBOB,
        RATIONAL_POLYNOMIAL
    };

    ///
    /// @brief Unrectified camera projection matrix stored in row-major ordering
    ///
    std::array<std::array<float, 3>, 3> K = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

    ///
    /// @brief Rotation matrix which takes points in the unrectified camera frame and transform
    ///        them in to the rectified coordinate frame
    ///
    std::array<std::array<float, 3>, 3> R = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

    ///
    /// @brief Rectified projection matrix which takes points in the origin camera coordinate
    ///        frame and projects them into the current camera
    ///
    std::array<std::array<float, 4>, 3> P = {{{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}}};

    ///
    /// @brief The type of the distortion model used for the unrectified camera
    ///
    DistortionType distortion_type = DistortionType::NONE;

    ///
    /// @brief Coefficients for the distortion model
    ///
    std::vector<float> D = {};
};

struct StereoCalibration
{
    ///
    /// @brief Calibration information for the left camera
    ///
    CameraCalibration left;

    ///
    /// @brief Calibration information for the right camera
    ///
    CameraCalibration right;

    ///
    /// @brief Calibration information for the aux camera (optional 3rd center camera)
    ///
    CameraCalibration aux;
};

struct DeviceInfo
{
    struct PcbInfo
    {
        std::string name;
        uint32_t revision;
    };

    enum class HardwareRevision
    {
        UNKNOWN,
        SL,
        S7,
        S21,
        ST21,
        S27,
        S30,
        KS21,
        MONOCAM,
        KS21_SILVER,
        ST25,
        KS21i
    };

    enum class ImagerType
    {
        UNKNOWN,
        CMV2000_GREY,
        CMV2000_COLOR,
        CMV4000_GREY,
        CMV4000_COLOR,
        FLIR_TAU2,
        AR0234_GREY,
        AR0239_COLOR
    };

    enum class LightingType
    {
        NONE,
        INTERNAL,
        EXTERNAL,
        PATTERN_PROJECTOR
    };

    //
    // TODO(malvarado): Populate this with valid inputs
    //
    enum class LensType
    {
        UNKNOWN
    };

    std::string camera_name;
    std::string build_date;
    std::string serial_number;
    HardwareRevision hardware_revision;

    uint8_t number_of_pcbs;
    std::array<PcbInfo, 8>  pcb_info;

    std::string imager_name;
    ImagerType  imager_type;
    uint32_t imager_width;
    uint32_t imager_height;

    std::string lens_name;
    LensType lens_type;
    float nominal_stereo_baseline;    // meters
    float nominal_focal_lenght_m;     // meters
    float nominal_relative_aperature; // f-stop

    LightingType lighting_type;
    uint32_t number_of_lights;
};

///
/// @brief Represents a single image plus metadata
///
struct Image
{

    ///
    /// @brief Pixel formats
    ///
    enum class PixelFormat
    {
        UNKNOWN,
        MONO8,
        RGB8,
        MONO16
    };

    ///
    /// @brief A pointer to the raw image data sent from the camera
    ///
    std::shared_ptr<const std::vector<uint8_t>> raw_data = nullptr;

    ///
    /// @brief An offset into the raw_data pointer where the image data starts
    ///
    int64_t image_data_offset = 0;

    ///
    /// @brief The length of the image data after the image_data_offset has been applied
    ///
    size_t image_data_length = 0;

    ///
    /// @brief The format of the image data stored in the raw_data stored in the raw_data buffer
    ///
    PixelFormat format = PixelFormat::UNKNOWN;

    ///
    /// @brief Width of the image in pixels
    ///
    int width = -1;

    ///
    /// @brief Height of the image in pixels
    ///
    int height = -1;

    ///
    /// @brief The timestamp associated with the image based on the camera's clock. Starts at 0
    ///        on boot
    ///
    std::chrono::system_clock::time_point camera_timestamp{};

    ///
    /// @brief The timestamp associated with the image based using the camera's clock which is potentially PTP
    ///        synchronized with a remote PTP master
    ///
    std::chrono::system_clock::time_point ptp_timestamp{};

    ///
    /// @brief The camera data source which this image corresponds to
    ///
    DataSource source = DataSource::UNKNOWN;

    ///
    /// @brief The scaled calibration associated with the image
    ///
    CameraCalibration calibration;
};

///
/// @brief A frame containing multiple images (indexed by DataSource).
///
struct ImageFrame
{
    ///
    /// @brief Add an image to the frame, keyed by the image's DataSource.
    ///
    void add_image(const Image& image)
    {
        images[image.source] = image;
    }

    ///
    /// @brief Retrieve image by DataSource. Throws if not found.
    ///
    const Image& get_image(const DataSource &source) const
    {
        auto it = images.find(source);
        if (it == images.end())
        {
            throw std::runtime_error("No image found for requested DataSource");
        }
        return it->second;
    }

    ///
    /// @brief Check if we have an image for a given data source
    ///
    bool has_image(const DataSource &source) const
    {
        return (images.find(source) != images.end());
    }

    int64_t frame_id = 0;
    std::map<DataSource, Image> images;
    std::chrono::system_clock::time_point frame_time{};
    std::chrono::system_clock::time_point ptp_frame_time{};
};

}
