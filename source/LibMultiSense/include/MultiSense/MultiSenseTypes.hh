/**
 * @file MultiSenseTypes.hh
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

#include <array>
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

#if !defined(MULTISENSE_API)
#if defined (_MSC_VER)
#if defined (MultiSense_EXPORTS)
#define MULTISENSE_API __declspec(dllexport)
#else
#define MULTISENSE_API __declspec(dllimport)
#endif
#else
#define MULTISENSE_API
#endif
#endif


namespace multisense
{

using TimeT = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;

enum class Status : uint8_t
{
    UNKNOWN,
    OK,
    TIMEOUT,
    INTERNAL_ERROR,
    FAILED,
    UNSUPPORTED,
    EXCEPTION,
    UNINITIALIZED,
    INCOMPLETE_APPLICATION
};

///
/// @brief Identifies which camera or data source the image is from
///
enum class DataSource : uint16_t
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
    COST_RAW,
    IMU
};

struct MULTISENSE_API CameraCalibration
{
    ///
    /// @brief Distortion type
    ///
    enum class DistortionType : uint8_t
    {
        NONE,
        PLUMBOB,
        RATIONAL_POLYNOMIAL
    };

    ///
    /// @brief Unrectified camera projection matrix stored in row-major ordering
    ///
    std::array<std::array<float, 3>, 3> K{{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief Rotation matrix which takes points in the unrectified camera frame and transform
    ///        them in to the rectified coordinate frame
    ///
    std::array<std::array<float, 3>, 3> R{{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief Rectified projection matrix which takes points in the origin camera coordinate
    ///        frame and projects them into the current camera
    ///
    std::array<std::array<float, 4>, 3> P{{{0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}}};

    ///
    /// @brief The type of the distortion model used for the unrectified camera
    ///
    DistortionType distortion_type = DistortionType::NONE;

    ///
    /// @brief Coefficients for the distortion model
    ///
    std::vector<float> D = {};
};

struct MULTISENSE_API StereoCalibration
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
/// @brief Represents a single image plus metadata
///
struct MULTISENSE_API Image
{
    ///
    /// @brief Pixel formats
    ///
    enum class PixelFormat : uint8_t
    {
        UNKNOWN,
        MONO8,
        RGB8,
        MONO16,
        FLOAT32
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
    TimeT camera_timestamp{};

    ///
    /// @brief The timestamp associated with the image based using the camera's clock which is potentially PTP
    ///        synchronized with a remote PTP master
    ///
    TimeT ptp_timestamp{};

    ///
    /// @brief The camera data source which this image corresponds to
    ///
    DataSource source = DataSource::UNKNOWN;

    ///
    /// @brief The scaled calibration associated with the image
    ///
    CameraCalibration calibration;

    ///
    /// @brief Get a pixel at a certain width/height location in the image.
    ///        NOTE: This check is slower since it checks to make sure the request is safe. It
    ///        should not be called repeatedly at high frequency
    ///
    template <typename T>
    std::optional<T> at(int w, int h) const
    {
        if (w < 0 || h < 0 || w >= width || h >= height || raw_data == nullptr || format == PixelFormat::UNKNOWN)
        {
            return std::nullopt;
        }

        if ((sizeof(T) == 8 && format == PixelFormat::MONO8) ||
            (sizeof(T) == 16 && format == PixelFormat::MONO16) ||
            (sizeof(T) == 24 && format == PixelFormat::RGB8) ||
            (sizeof(T) == 32 && format == PixelFormat::FLOAT32))
        {
            const size_t offset = sizeof(T) * ((width * h) + w);

            return *reinterpret_cast<const T*>(raw_data->data() + image_data_offset + offset);
        }

        return std::nullopt;
    }

#ifdef HAVE_OPENCV
    ///
    /// @brief Transform a image into a cv::Mat object if the user wants to build OpenCV utilities
    ///        The cv::Mat returned here wraps the underlying image data pointer associated with
    ///        the Image object. If the input Image object goes out of scope while you are still using
    ///        the corresponding cv::Mat, you will need to `clone` the cv::Mat creating an internal copy
    ///        of all the data
    ///
    cv::Mat cv_mat() const;
#endif
};

///
/// @brief A frame containing multiple images (indexed by DataSource).
///
struct MULTISENSE_API ImageFrame
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

    ///
    /// @brief The MultiSeense timestamp associated with the frame
    ///
    TimeT frame_time{};

    ///
    /// @brief The MultiSeense ptp timestamp associated with the frame
    ///
    TimeT ptp_frame_time{};
};

///
/// @brief A single IMU sample from the camera
///
struct MULTISENSE_API ImuSample
{
    ///
    /// @brief A generic measurement for a 3-axis IMU
    ///
    struct MULTISENSE_API Measurement
    {
        ///
        /// @brief Measurement on the x-axis of the sensor
        ///
        float x = 0.0f;

        ///
        /// @brief Measurement on the y-axis of the sensor
        ///
        float y = 0.0f;

        ///
        /// @brief Measurement on the z-axis of the sensor
        ///
        float z = 0.0f;
    };

    ///
    /// @brief The acceleration in units of Gs. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has separate accelerometer and gyroscope chips)
    ///
    std::optional<Measurement> accelerometer{};
    ///
    /// @brief The rotational velocity in degrees-per-second. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has separate accelerometer and gyroscope chips)
    ///
    std::optional<Measurement> gyroscope{};
    ///
    /// @brief The measured magnetic field in milligauss. Depending on the IMU configuration of the sensor,
    ///        this may be invalid (i.e. the MultiSense has a separate magnetometer chip)
    ///
    std::optional<Measurement> magnetometer{};

    ///
    /// @brief The MultiSeense timestamp associated with the frame
    ///
    TimeT sample_time{};

    ///
    /// @brief The  MultiSeense ptp timestamp associated with the frame
    ///
    TimeT ptp_sample_time{};
};

///
/// @brief A collection of IMU samples from the camera
///
struct MULTISENSE_API ImuFrame
{
    ///
    /// @brief A batched collection of IMU samples
    ///
    std::vector<ImuSample> samples;
};


///
/// @brief Complete configuration object for configuring the MultiSense. Can be updated during camera operation
///
struct MULTISENSE_API MultiSenseConfiguration
{
    ///
    /// @brief Stereo specific configuration
    ///
    struct MULTISENSE_API StereoConfiguration
    {
        ///
        ///
        /// @brief This is used to filter low confidence stereo data before it is sent to the
        ///        host. Larger numbers indicate more aggressive filtering.
        ///        Valid range is [0, 1.0]
        ///
        float postfilter_strength = 0.85f;
    };

    ///
    /// @brief Manual exposure specific configuration
    ///
    struct MULTISENSE_API ManualExposureConfiguration
    {
        ///
        /// @brief The desired electrical and digital gain used to brighten the image.
        ///        Valid range is [1.6842, 16]
        ///
        float gain = 1.68f;

        ///
        /// @brief The manual exposure time in microseconds
        ///        Valid range is [0, 33000]
        ///
        std::chrono::microseconds exposure_time{10000};
    };

    ///
    /// @brief Auto-exposure Region-of-Interest (ROI) specific configuration
    ///
    struct MULTISENSE_API AutoExposureRoiConfiguration
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
    struct MULTISENSE_API AutoExposureConfiguration
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
        float target_intensity = 0.5f;

        ///
        /// @brief The ratio of pixels which must be equal or below the pixel value set by the target intensity setting.
        ///
        float target_threshold = 0.85f;

        ///
        /// @brief The auto exposure algorithm adjusts both exposure and gain. This caps the gain the auto exposure
        ///        algorithm can use
        ///
        float max_gain = 2.0f;

        ///
        /// @brief The auto exposure region-of-interest used to restrict the portion of the image which the
        ///        auto exposure algorithm is run on
        ///
        AutoExposureRoiConfiguration roi;
    };

    ///
    /// @brief Manual white balance specific configuration
    ///
    struct MULTISENSE_API ManualWhiteBalanceConfiguration
    {
        ///
        /// @brief The manual red white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float red = 1.0f;

        ///
        /// @brief The manual blue white-balance setting
        ///        Valid range is [0.25, 4]
        ///
        float blue = 1.0f;
    };

    ///
    /// @brief Auto white balance specific configuration
    ///
    struct MULTISENSE_API AutoWhiteBalanceConfiguration
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
        float threshold = 0.5f;
    };

    ///
    /// @brief Image specific configuration
    ///
    struct MULTISENSE_API ImageConfiguration
    {
        ///
        /// @brief Set the gamma correction for the image.
        ///        Valid range [1.0, 2,2]
        ///
        float gamma = 2.2f;

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
    struct MULTISENSE_API AuxConfiguration
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
        float sharpening_percentage = 50.0f;

        ///
        ///
        ///  @brief The maximum difference in pixels that sharpening is
        ///         is allowed to change between neighboring pixels. This is useful for clamping
        ///         the sharpening percentage, while still maintaining a large gain.
        ///
        uint8_t sharpening_limit = 100;
    };

    ///
    /// @brief Operating resolutions for the MultiSense
    ///
    enum class OperatingResolution : uint8_t
    {
        ///
        /// @brief default value to handle any unknown values
        ///
        UNSUPPORTED,
        ///
        /// @brief Operates at the native resolution of the camera's imager sensor
        ///
        FULL_RESOLUTION,
        ///
        /// @brief Operates at the 1/4 of resolution of the camera's imager sensor. 1/2 of the native width and 1/2
        ///        of the native height
        ///
        QUARTER_RESOLUTION
    };

    ///
    /// @brief Predefined disparity pixel search windows. Larger values allows the camera to see objects
    ///        closer to the camera
    ///
    enum class MaxDisparities : uint8_t
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
    struct MULTISENSE_API TimeConfiguration
    {
        ///
        /// @brief Enable PTP sync on the camera
        ///
        bool ptp_enabled = false;
    };

    ///
    /// @brief Configuration for transmitting packets from the MultiSense to the host
    ///
    struct MULTISENSE_API NetworkTransmissionConfiguration
    {
        ///
        /// @brief Add a small delay between the transmission of each packet to hopefully interact
        ///        better with slower client machines, or more fragile networks
        ///
        bool packet_delay_enabled = false;
    };

    ///
    /// @brief Configuration for the IMU sensor
    ///
    struct MULTISENSE_API ImuConfiguration
    {
        ///
        /// @brief Configuration for a specific IMU operating mode. There are separate modes for each of the
        ///        IMU sensors (i.e. accelerometer, gyroscope, magnetometer)
        ///
        struct MULTISENSE_API OperatingMode
        {
            ///
            /// @brief The name of the specific IMU source corresponding to the ImuSource
            ///
            std::string name{};

            ///
            /// @brief Enable the current source
            ///
            bool enabled = false;

            //
            // TODO (malvarado): Pass value rather than index
            //

            ///
            /// @brief The index of the specific IMU rate configuration specified in ImuSource rate
            ///        table
            ///
            uint32_t rate_index = 0;

            ///
            /// @brief The index of the specific IMU range configuration specified in ImuSource range
            ///        table
            ///
            uint32_t range_index = 0;
        };

        ///
        /// @brief The number of IMU samples which should be included in a IMU frame
        ///
        uint32_t samples_per_frame = 0;

        ///
        /// @brief The current IMU modes
        ///
        std::vector<OperatingMode> modes{};
    };

    ///
    /// @brief Lighting configuration for the camera
    /// TODO (malvarado) Handle serialization explicitly since optionals of optionals are not supported
    ///
    struct MULTISENSE_API LightingConfiguration
    {
        ///
        /// @brief Lighting config for lights integrated into the MultiSense
        ///
        struct MULTISENSE_API InternalConfig
        {
            ///
            /// @brief Lighting brightness ranging from 0 to 100.0. A value of 0 will turn off the LEDs
            ///
            float intensity = 0.0f;

            ///
            /// @brief Enable flashing of the light
            ///
            bool flash = false;
        };

        ///
        /// @brief Lighting config for lights driven by GPIO outputs from the MultiSense
        ///
        struct MULTISENSE_API ExternalConfig
        {
            ///
            /// @brief Different flash modes for the camera
            ///
            enum class FlashMode : uint8_t
            {
                NONE,
                SYNC_WITH_MAIN_STEREO,
                SYNC_WITH_AUX
            };

            ///
            /// @brief Lighting brightness ranging from 0 to 100.0. A value of 0 will turn off the LEDs
            ///
            float intensity = 0.0f;

            ///
            /// @brief Configure flash mode
            ///
            FlashMode flash = FlashMode::NONE;

            ///
            /// @brief The number of pulses of the light per single exposure.
            ///        This is used to trigger the light or output signal multiple times after a
            ///        single exposure. For values greater than 1, pulses will occur between the
            ///        exposures, not during. This can be used to leverage human persistence of
            ///        vision to make the light appear as though it is not flashing
            ///
            uint32_t pulses_per_exposure = 1;

            ///
            /// @brief The time it takes for the light to reach full brightness. This will be used to
            ///        ensure the light is at full brightness when the image is exposed
            ///
            std::chrono::microseconds startup_time{0};
        };

        ///
        /// @brief The internal lighting config. Will be nullopt if the camera does not
        ///        support internal lighting controls
        ///
        std::optional<InternalConfig> internal = std::nullopt;

        ///
        /// @brief The external lighting config. Will be nullopt if the camera does not
        ///        support internal lighting controls
        ///
        std::optional<ExternalConfig> external = std::nullopt;
    };

    ///
    /// @brief The operating resolution of the MultiSense
    ///
    OperatingResolution resolution = OperatingResolution::QUARTER_RESOLUTION;

    ///
    /// @brief The max number of pixels the MultiSense searches when computing the disparity output
    ///
    MaxDisparities disparities = MaxDisparities::D256;

    ///
    /// @brief The target framerate the MultiSense should operate at
    ///
    float frames_per_second = 10.0f;

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

    ///
    /// @brief Configuration to control network transmission settings
    ///
    NetworkTransmissionConfiguration network_config;

    ///
    /// @brief The imu configuration to use for the camera. Will be invalid if sensor does not contain an IMU
    ///
    std::optional<ImuConfiguration> imu_config = std::nullopt;

    ///
    /// @brief The lighting configuration for the camera. If invalid, the camera does not support lighting
    ///        configuration
    ///
    LightingConfiguration lighting_config{};
};

///
/// @brief Consolidated status information which can be queried on demand from the MultiSense. Will change
///        during camera operation
///
struct MULTISENSE_API MultiSenseStatus
{
    struct MULTISENSE_API PtpStatus
    {
        ///
        /// @brief Status of the grandmaster clock. true if synchronized to a non-local grandmaster OR if
        ///        a non-local grandmaster was present any time during the current boot
        ///
        bool grandmaster_present = false;

        ///
        /// @brief The id of the current grandmaster clock (8 bytes, 0xXXXXXX.XXXX.XXXXXX)
        ///
        std::array<uint8_t, 8> grandmaster_id{0, 0, 0, 0, 0, 0, 0, 0};

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

    struct MULTISENSE_API CameraStatus
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

    struct MULTISENSE_API TemperatureStatus
    {
        ///
        /// @brief Temperature of the FPGA  in Celsius
        ///
        float fpga_temperature = 0.0f;

        ///
        /// @brief Temperature of the left imager in Celsius
        ///
        float left_imager_temperature = 0.0f;

        ///
        /// @brief Temperature of the right imager in Celsius
        ///
        float right_imager_temperature = 0.0f;

        ///
        /// @brief Temperature of the internal switching power supply in Celsius
        ///
        float power_supply_temperature = 0.0f;
    };

    struct MULTISENSE_API PowerStatus
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

    struct MULTISENSE_API ClientNetworkStatus
    {
        ///
        /// @brief The total number of valid messages received from the client
        ///
        size_t received_messages = 0;

        ///
        /// @brief The total number of dropped messages on the client side
        ///
        size_t dropped_messages = 0;

        ///
        /// @brief The total number of invalid packets received on the client side
        ///
        size_t invalid_packets = 0;
    };

    struct MULTISENSE_API TimeStatus
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

        ///
        /// @brief Compute the time offset which can manually be added to all camera timestamps to
        ///        change the camera timestamps from the reference clock of the camera into reference clock of the host
        ///
        std::chrono::nanoseconds offset_to_host() const
        {
            return (client_host_time + network_delay) - camera_time;
        }

        ///
        /// @brief Apply the time offset to a camera timestamps to camera timestamp from the reference clock of
        ///        the camera into reference clock of the host
        ///
        TimeT apply_offset_to_host(const TimeT &input_camera_time) const
        {
            return input_camera_time + std::chrono::duration_cast<std::chrono::system_clock::duration>(offset_to_host());
        }
    };

    ///
    /// @brief Summary of the current MultiSense state. True if the MultiSense is operating properly
    ///
    bool system_ok = false;

    ///
    /// @brief The current ptp status. Only valid if ptp is enabled
    ///
    std::optional<PtpStatus> ptp;

    ///
    /// @brief The current camera status
    ///
    CameraStatus camera;

    ///
    /// @brief The current temperature status
    ///
    TemperatureStatus temperature;

    ///
    /// @brief The current power status
    ///
    PowerStatus power;

    ///
    /// @brief The current client network statistics
    ///
    ClientNetworkStatus client_network;

    ///
    /// @brief The current timing status information
    ///
    TimeStatus time;
};

///
/// @brief Static status info for the MultiSense. Will not change during camera operation
///
struct MULTISENSE_API MultiSenseInfo
{
    ///
    /// @brief The network configuration for the MultiSense
    ///
    struct MULTISENSE_API NetworkInfo
    {
        ///
        /// @brief The ip address of the camera (i.e. X.X.X.X)
        ///
        std::string ip_address = "10.66.171.21";

        ///
        /// @brief The gateway of the camera (i.e. X.X.X.X)
        ///
        std::string gateway = "10.66.171.1";

        ///
        /// @brief The netmask of the camera (i.e. X.X.X.X)
        ///
        std::string netmask = "255.255.255.0";
    };

    ///
    /// @brief The Device information associated with the MultiSense. The DeviceInfo is used to determine what features
    ///        the MultiSense offers, and provides debug information to the Carnegie Robotics' team.
    ///        The DeviceInfo can only be set at the factory
    ///
    struct MULTISENSE_API DeviceInfo
    {
        ///
        /// @brief Info for the PCBs contained in the unit
        ///
        struct MULTISENSE_API PcbInfo
        {
            std::string name;
            uint32_t revision;
        };

        ///
        /// @brief MultiSense Hardware revisions
        ///
        enum class HardwareRevision : uint8_t
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
        enum class ImagerType : uint8_t
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
        enum class LightingType : uint8_t
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
        enum class LensType : uint8_t
        {
            UNKNOWN
        };

        ///
        /// @brief The name of the MultiSense variant
        ///
        std::string camera_name{};

        ///
        /// @brief The date the MultiSense was manufactured
        ///
        std::string build_date{};

        ///
        /// @brief The unique serial number of the MultiSense
        ///
        std::string serial_number{};

        ///
        /// @brief The hardware revision of the MultiSense
        ///
        HardwareRevision hardware_revision{};

        ///
        /// @brief Information about each PCB
        ///
        std::vector<PcbInfo> pcb_info{};

        ///
        /// @brief The name of the imager used by the primary camera. For stereo cameras this is the
        ///        Left/Right stereo pair. For mono cameras this is the single imager
        ///
        std::string imager_name{};

        ///
        /// @brief The type of the imager
        ///
        ImagerType imager_type{};

        ///
        /// @brief The native width of the primary imager
        ///
        uint32_t imager_width = 0;

        ///
        /// @brief The native height of the primary imager
        ///
        uint32_t imager_height = 0;

        ///
        /// @brief The name of the lens used for the primary camera For stereo cameras this is the
        ///        Left/Right stereo pair. For mono cameras this is the single camera
        ///
        std::string lens_name{};

        ///
        /// @brief The type of the primary imager
        ///
        LensType lens_type{};

        ///
        /// @brief The nominal stereo baseline in meters
        ///
        float nominal_stereo_baseline = 0.0f;

        ///
        /// @brief The nominal focal length for the primary lens in meters
        ///
        float nominal_focal_length = 0.0f;

        ///
        /// @brief The nominal relative aperture for the primary camera modules in f-stop
        ///
        float nominal_relative_aperture = 0.0f;

        ///
        /// @brief The type of lighting used in the MultiSense
        ///
        LightingType lighting_type{};

        ///
        /// @brief The number of lights the MultiSense controls
        ///
        uint32_t number_of_lights = 0;

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
        }
    };

    ///
    /// @brief Convenience wrapper for a version number
    ///        See https://semver.org/
    ///
    struct MULTISENSE_API Version
    {
        ///
        /// @brief Major version number
        ///
        uint32_t major = 0;

        ///
        /// @brief Minor version number
        ///
        uint32_t minor = 0;

        ///
        /// @brief Patch version number
        ///
        uint32_t patch = 0;

        ///
        /// @brief Convert a Version info to string for convenience
        ///
        std::string to_string() const
        {
            return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
        }

        ///
        /// @brief Convenience operator for comparing versions
        ///
        bool operator<(const Version& other) const
        {
            return major < other.major ||
                   (major == other.major && minor < other.minor) ||
                   (major == other.major && minor == other.minor && patch < other.patch);
        }
    };

    ///
    /// @brief Version information for the MultiSense
    ///
    struct MULTISENSE_API SensorVersion
    {
        ///
        /// @brief The date the firmware running on the camera was built
        ///
        std::string firmware_build_date{};

        ///
        /// @brief The version of the firmware running on the camera
        ///
        MultiSenseInfo::Version firmware_version{};

        ///
        /// @brief ID for the version of hardware
        ///
        uint64_t hardware_version = 0;

        ///
        /// @brief Unique ID for the hardware
        ///
        uint64_t hardware_magic = 0;

        ///
        /// @brief Unique ID for the fpga
        ///
        uint64_t fpga_dna = 0;
    };

    ///
    /// @brief A valid operating mode for the MultiSense
    ///
    struct MULTISENSE_API SupportedOperatingMode
    {
        ///
        /// @brief Supported operating resolution
        ///
        MultiSenseConfiguration::OperatingResolution resolution = MultiSenseConfiguration::OperatingResolution::QUARTER_RESOLUTION;

        ///
        /// @brief Supported operating disparity
        ///
        MultiSenseConfiguration::MaxDisparities disparities = MultiSenseConfiguration::MaxDisparities::D256;

        ///
        /// @brief Data sources supported at that mode
        ///
        std::vector<DataSource> supported_sources{};
    };

    ///
    /// @brief Info about the available IMu configurations
    ///
    struct MULTISENSE_API ImuSource
    {
        ///
        /// @brief A sample rate, and what impact it has on bandwidth
        ///
        struct MULTISENSE_API Rate
        {
            ///
            /// @brief The sample rate for the sensor in Hz
            ///
            float sample_rate = 0.0f;

            ///
            /// @brief The bandwith cutoff for a given IMU mode in Hz
            ///
            float bandwith_cutoff = 0.0f;
        };

        ///
        /// @brief The range for each sensor along with the corresponding sampling resolution
        ///
        struct MULTISENSE_API Range
        {
            ///
            /// @brief The max value the sensor can return. This value is +/- units for the given source
            ///
            float range = 0.0f;

            ///
            /// @brief The min resolution the sensor can return. In units per LSB
            ///
            float resolution = 0.0f;
        };

        ///
        /// @brief The name of the IMU chip on the camera
        ///
        std::string name{};

        ///
        /// @brief The name of the IMU device for the source
        /// TODO (malvarado) make this an enum (accel, gyro, mag) Update sample message too
        ///
        std::string device{};

        ///
        /// @brief The name of the units type it broadcasts
        ///
        /// TODO (malvarado) only use standard units (g's, degrees, milligaus)
        ///
        std::string units{};

        ///
        /// @brief The available rates supported by this operating mode
        ///
        std::vector<Rate> rates;

        ///
        /// @brief The available ranges supported by this operating mode
        ///
        std::vector<Range> ranges;
    };

    ///
    /// @brief Device info
    ///
    DeviceInfo device;

    ///
    /// @brief Sensor Version info
    ///
    SensorVersion version;

    ///
    /// @brief Supported operating modes
    ///
    std::vector<SupportedOperatingMode> operating_modes;

    ///
    /// @brief Supported operating modes for the IMU sensors (accelerometer, gyroscope, magnetometer). Will
    ///        be invalid if an IMU is not present
    ///
    std::optional<std::vector<ImuSource>> imu;

    ///
    /// @brief The network configuration of the MultiSense
    ///
    NetworkInfo network;
};

}
