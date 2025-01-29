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

#ifdef HAVE_OPENCV
#include <opencv2/core/mat.hpp>
#endif


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
    std::optional<CameraCalibration> aux = std::nullopt;
};

///
/// @brief The Device information associated with the MultiSense. The DeviceInfo is used to determine what features
///        the MultiSense offers, and provides debug information to the Carnegie Robotics' team.
///        The DeviceInfo can only be set at the factory
///
struct DeviceInfo
{
    ///
    /// @brief Info for the PCBs contained in the unit
    ///
    struct PcbInfo
    {
        std::string name;
        uint32_t revision;
    };

    ///
    /// @brief MultiSense Hardware revisions
    ///
    enum class HardwareRevision
    {
        UNKNOWN,
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

    ///
    /// @brief Different imager types
    ///
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

    ///
    /// @brief MultiSense lighting types
    ///
    enum class LightingType
    {
        ///
        /// @brief No lights
        ///
        NONE,
        ///
        /// @brief Lights driven internally
        ///
        INTERNAL,
        ///
        /// @brief Drive lights via an external output
        ///
        EXTERNAL,
        ///
        /// @brief A pattern projector
        ///
        PATTERN_PROJECTOR
    };

    //
    // TODO(malvarado): Populate this with valid inputs
    //
    enum class LensType
    {
        UNKNOWN
    };

    ///
    /// @brief The name of the MultiSense variant
    ///
    std::string camera_name;

    ///
    /// @brief The date the MultiSense was manufactured
    ///
    std::string build_date;

    ///
    /// @brief The unique serial number of the MultiSense
    ///
    std::string serial_number;

    ///
    /// @brief The hardware revision of the MultiSense
    ///
    HardwareRevision hardware_revision;

    ///
    /// @brief The number of valid pcb_info objects
    ///
    uint8_t number_of_pcbs;

    ///
    /// @brief Information about each PCB
    ///
    std::array<PcbInfo, 8>  pcb_info;

    ///
    /// @brief The name of the imager used by the primary camera. For stereo cameras this is the
    ///        Left/Right stereo pair. For mono cameras this is the single imager
    ///
    std::string imager_name;

    ///
    /// @brief The type of the imager
    ///
    ImagerType  imager_type;

    ///
    /// @brief The native width of the primary imager
    ///
    uint32_t imager_width;

    ///
    /// @brief The native height of the primary imager
    ///
    uint32_t imager_height;

    ///
    /// @brief The name of the lens used for the primary camera For stereo cameras this is the
    ///        Left/Right stereo pair. For mono cameras this is the single camera
    ///
    std::string lens_name;

    ///
    /// @brief The type of the primary imager
    ///
    LensType lens_type;

    ///
    /// @brief The nominal stereo baseline in meters
    ///
    float nominal_stereo_baseline;

    ///
    /// @brief The nominal focal length for the primary lens in meters
    ///
    float nominal_focal_length;

    ///
    /// @brief The nominal relative aperture for the primary camera modules in f-stop
    ///
    float nominal_relative_aperature;

    ///
    /// @brief The type of lighting used in the MultiSense
    ///
    LightingType lighting_type;

    ///
    /// @brief The number of lights the MultiSense controls
    ///
    uint32_t number_of_lights;

    ///
    /// @brief Determine if the MultiSense has a Aux color camera based on the DeviceInfo
    ///
    constexpr bool has_aux_camera() const
    {
        switch (hardware_revision)
        {
            case HardwareRevision::S27:
            case HardwareRevision::S30:
            case HardwareRevision::MONOCAM:
            case HardwareRevision::KS21i:
                return true;
            default:
                return false;
        }

        return false;
    }
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

    ///
    /// @brief Transform a image into a cv::Mat object if the user wants to build OpenCV utilities
    ///        The cv::Mat returned here wraps the underlying image data pointer associated with
    ///        the Image object. If the input Image object goes out of scope while you are still using
    ///        the corresponding cv::Mat, you will need to `clone` the cv::Mat creating an internal copy
    ///        of all the data
    ///
#ifdef HAVE_OPENCV
    cv::Mat cv_mat() const;
#endif
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

    ///
    /// @brief The unique monotonically increasing ID for each frame populated by the MultiSense
    ///
    int64_t frame_id = 0;

    ///
    /// @brief The images assocated with each source in the frame
    ///
    std::map<DataSource, Image> images;

    ///
    /// @brief The scaled calibration for the entire camera
    ///
    StereoCalibration calibration;
    std::chrono::system_clock::time_point frame_time{};
    std::chrono::system_clock::time_point ptp_frame_time{};
};

///
/// @brief Complete configuration object for configuring the MultiSense
///
struct MultiSenseConfiguration
{
    ///
    /// @brief Stereo specific configuration
    ///
    struct StereoConfiguration
    {
        ///
        ///
        /// @brief This is used to filter low confidence stereo data before it is sent to the
        ///        host. Larger numbers indicate more aggressive filtering.
        ///        Valid range is [0, 1.0]
        ///
        float postfilter_strength = 0.85;
    };

    ///
    /// @brief Manual exposure specific configuration
    ///
    struct ManualExposureConfiguration
    {
        ///
        /// @brief The desired electrical and digital gain used to brighten the image.
        ///        Valid range is [1.6842, 16]
        ///
        float gain = 1.68;

        ///
        /// @brief The manual exposure time in microseconds
        ///        Valid range is [0, 33000]
        ///
        std::chrono::microseconds exposure_time{10000};
    };

    ///
    /// @brief Auto-exposure Region-of-Interest (ROI) specific configuration
    ///
    struct AutoExposureRoiConfiguration
    {
        ///
        /// @brief The x value of the top left corner of the ROI in the full resolution image. Note (0,0) is the top
        ///        left corner in the image coordinate frame.
        ///
        uint16_t top_left_x_position = 0;
        ///
        /// @brief The y value of the top left corner of the ROI in the full resolution image. Note (0,0) is the top
        ///        left corner in the image coordinate frame.
        ///
        uint16_t top_left_y_position = 0;
        ///
        /// @brief The width of the ROI in the full resolution image. A value of 0 disables the ROI
        ///
        uint16_t width = 0;
        ///
        /// @brief The height of the ROI in the full resolution image. A value of 0 disables the ROI
        ///
        uint16_t height = 0;
    };

    ///
    /// @brief Auto-exposure specific configuration
    ///
    struct AutoExposureConfiguration
    {
        ///
        /// @brief The max exposure time auto exposure algorithm can set in microseconds
        ///        Valid range is [0, 33000]
        ///
        std::chrono::microseconds max_exposure_time{10000};

        ///
        /// @brief The desired auto-exposure decay rate.
        ///         Valid range is [0, 20]
        ///
        uint32_t decay = 7;

        ///
        /// @brief The target intensity in the form of a ratio the auto exposure algorithm is trying to achieve.j
        ///        This ratio is multiplied by the max pixel value (255), to get the target auto-exposure pixel value.
        ///        The auto exposure algorithm tries to drive the target_threshold percentage of pixels below the target
        ///        intensity value
        ///
        float target_intensity = 0.5;

        ///
        /// @brief The ratio of pixels which must be equal or below the pixel value set by the target intensity setting.
        ///
        float target_threshold = 0.85;

        ///
        /// @brief The auto exposure algorithm adjusts both exposure and gain. This caps the gain the auto exposure
        ///        algorithm can use
        ///
        float max_gain = 2.0;

        ///
        /// @brief The auto exposure region-of-interest used to restrict the portion of the image which the
        ///        auto exposure algorithm is run on
        ///
        AutoExposureRoiConfiguration roi;
    };

    ///
    /// @brief Manual white balance specific configuration
    ///
    struct ManualWhiteBalanceConfiguration
    {
        ///
        /// @brief The manual red white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float red = 1.0;

        ///
        /// @brief The manual blue white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float blue = 1.0;
    };

    ///
    /// @brief Auto white balance specific configuration
    ///
    struct AutoWhiteBalanceConfiguration
    {
        ///
        /// @brief The decay rate used for auto-white-balance
        ///        Valid range [0, 20]
        ///
        uint32_t decay = 3;

        ///
        /// @brief The auto white balance threshold
        ///        Valid range [0.0, 1.0]
        ///
        float threshold = 0.5;
    };

    ///
    /// @brief Image specific configuration
    ///
    struct ImageConfiguration
    {
        ///
        /// @brief Set the gamma correction for the image.
        ///        Valid range [1.0, 2,2]
        ///
        float gamma = 2.2;

        ///
        /// @brief Enable HDR. Note this is not supported by the default MultiSense firmware. Please contact
        ///        support team (https://carnegierobotics.com/submitaticket) if you are interested in this feature
        ///
        bool hdr_enabled = false;

        ///
        /// @brief Enable or disable auto exposure
        ///
        bool auto_exposure_enabled = true;

        ///
        /// @brief The exposure config to use if auto exposure is disabled
        ///
        ManualExposureConfiguration manual_exposure;

        ///
        /// @brief The exposure config to use if auto exposure is enabled
        ///
        AutoExposureConfiguration auto_exposure;

        ///
        /// @brief Enable or disable auto white balance
        ///
        bool auto_white_balance_enabled = true;

        ///
        /// @brief The white balance parameters to use if auto white balance is disabled
        ///
        ManualWhiteBalanceConfiguration manual_white_balance;

        ///
        /// @brief The white balance parameters to use if auto white balance is enabled
        ///
        AutoWhiteBalanceConfiguration auto_white_balance;
    };

    ///
    /// @brief Image specific configuration for the Aux imager
    ///
    struct AuxConfiguration
    {
        ///
        /// @brief Image configuration for the Aux imager
        ///
        ImageConfiguration image_config;

        ///
        /// @brief Enable sharpening
        ///
        bool sharpening_enabled = false;

        ///
        /// @brief The percentage of the aux image to sharpen
        ///        Valid range is [0, 100]
        ///
        float sharpening_percentage = 50.0;

        ///
        ///
        ///  @brief The maximum difference in pixels that sharpening is
        ///         is allowed to change between neighboring pixels. This is useful for clamping
        ///         the sharpening percentage, while still maintaining a large gain.
        ///
        uint8_t sharpening_limit = 100;
    };

    ///
    /// @brief Predefined disparity pixel search windows. Larger values allows the camera to see objects
    ///        closer to the camera
    ///
    enum class MaxDisparities
    {
        ///
        /// @brief 64 pixels
        ///
        D64,
        ///
        /// @brief 128 pixels
        ///
        D128,
        ///
        /// @brief 256 pixels
        ///
        D256
    };

    ///
    /// @brief Configuration for time-based controls
    ///
    struct TimeConfiguration
    {
        ///
        /// @brief Enable PTP sync on the camera
        ///
        bool ptp_enabled = false;
    };

    ///
    /// @brief The MultiSense operating width
    ///
    uint32_t width = 960;

    ///
    /// @brief The MultiSense operating height
    ///
    uint32_t height = 600;

    ///
    /// @brief The max number of pixels the MultiSense searches when computing the disparity output
    ///
    MaxDisparities disparities = MaxDisparities::D256;

    ///
    /// @brief The target framerate the MultiSense should operate at
    ///
    float frames_per_second = 10;

    ///
    /// @brief The stereo configuration to use
    ///
    StereoConfiguration stereo_config;

    ///
    /// @brief The image configuration to use for the main stereo pair
    ///
    ImageConfiguration image_config;

    ///
    /// @brief The image configuration to use for the aux camera if present
    ///
    std::optional<AuxConfiguration> aux_config = std::nullopt;

    ///
    /// @brief Configuration for the MultiSense time-sync options
    ///
    TimeConfiguration time_config;
};

///
/// @brief Consolidated status information which can be queried on demand from the MultiSense
///
struct MultiSenseStatus
{
    struct PtpStatus
    {
        ///
        /// @brief Status of the grandmaster clock. true if synchronized to a non-local grandmaster OR if
        ///        a non-local grandmaster was present any time during the current boot
        ///
        bool grandmaster_present = false;

        ///
        /// @brief The id of the current grandmaster clock
        ///
        uint8_t grandmaster_id = 0;

        ///
        /// @brief Offset between the camera's PTP Hardware Clock and the grandmaster clock
        ///
        std::chrono::nanoseconds grandmaster_offset{0};

        ///
        /// @brief The estimate delay of the PTP synchronization messages from the grandmaster
        ///
        std::chrono::nanoseconds path_delay{0};

        ///
        /// @brief The number of network hops from the grandmaster to the camera's clock
        ///
        uint16_t steps_from_local_to_grandmaster = 0;
    };

    struct CameraStatus
    {
        ///
        /// @brief True if the cameras are operating and currently streaming data
        ///
        bool cameras_ok = false;

        ///
        /// @brief True if the onboard processing pipeline is ok and currently processing images
        ///
        bool processing_pipeline_ok = false;
    };

    struct TemperatureStatus
    {
        ///
        /// @brief Temperature of the FPGA  in Celsius
        ///
        float fpga_temperature_C = 0.0f;

        ///
        /// @brief Temperature of the left imager in Celsius
        ///
        float left_imager_temperature_C = 0.0f;

        ///
        /// @brief Temperature of the right imager in Celsius
        ///
        float right_imager_temperature_C = 0.0f;

        ///
        /// @brief Temperature of the internal switching power supply in Celsius
        ///
        float power_supply_temperature_C = 0.0f;
    };

    struct PowerStatus
    {
        ///
        /// @brief The current input voltage in volts
        ///
        float input_voltage = 0.0f;

        ///
        /// @brief The current input current in Amperes
        ///
        float input_current = 0.0f;

        ///
        /// @brief The current power draw of the FPGA in Watts
        ///
        float fpga_power = 0.0f;
    };

    struct ClientNetworkStatus
    {
        ///
        /// @brief The total number of dropped messages on the client side
        ///
        size_t dropped_messages = 0;

        ///
        /// @brief The total number of valid messages received from the client
        ///
        size_t received_messages = 0;
    };

    struct TimeStatus
    {
        ///
        /// @brief The camera's system time when the status message request was received
        ///
        std::chrono::nanoseconds camera_time{0};

        ///
        /// @brief The time of the host machine running the client when the status request was sent
        ///
        std::chrono::nanoseconds client_host_time{0};

        ///
        /// @brief The estimated network delay between when the status request was sent, and when the
        ///        status request was received. This is computed by measuring the time between when the client
        ///        machine sent the status request, and received the status response
        ///
        std::chrono::nanoseconds network_delay{0};
    };

    ///
    /// @brief summary of the current MultiSense state. True if the MultiSense is operating properly
    ///
    bool system_ok = false;

    PtpStatus ptp;

    CameraStatus camera;

    TemperatureStatus temperature;

    PowerStatus power;

    ClientNetworkStatus client_network;

    TimeStatus timee;
};

}
