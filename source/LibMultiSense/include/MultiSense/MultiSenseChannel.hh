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

#include <functional>
#include <optional>

#include "MultiSenseTypes.hh"

namespace multisense
{

///
/// @brief Identifiers for the different Channel implementations. Used for
///        selecting a implementation at runtime
///
enum class ChannelImplementation
{
    LEGACY
};

///
/// @brief Certain implementations may use a fixed set of internal buffers to mannage
///        incoming camera data. For those implementations specify configurations for
///        both small and large buffers
///
struct ReceiveBufferConfiguration
{
    size_t num_small_buffers = 0;
    size_t small_buffer_size = 0;
    size_t num_large_buffers = 0;
    size_t large_buffer_size = 0;
};

struct ChannelConfig
{
    std::string ip_address = "10.66.171.21";
    int16_t mtu = 1500;
    std::optional<std::chrono::milliseconds> receive_timeout = std::chrono::milliseconds(500);
    uint16_t command_port = 9001;
    std::optional<std::string> interface = std::nullopt;
    ReceiveBufferConfiguration receive_buffer_configuration{100, 1500, 16, 1920*1200*3};
};

class Channel {
public:

    ///
    /// @brief Factory create function which allows for switching between different channel
    ///        implementations
    ///
    static std::unique_ptr<Channel> create(const ChannelConfig &config,
                                           const ChannelImplementation &impl = ChannelImplementation::LEGACY);

    Channel() = default;

    ///
    /// Non-copyable
    ///
    Channel(const Channel&) = delete;
    Channel& operator=(const Channel&) = delete;

    ///
    /// Movable
    ///
    Channel(Channel&&) noexcept = default;
    Channel& operator=(Channel&&) noexcept = default;

    virtual ~Channel() = default;

    ///
    /// @brief Start a collection of data sources streaming from the camera.
    ///
    virtual bool start_streams(const std::vector<DataSource> &sources) = 0;

    ///
    /// @brief Stop specific data sources from streaming from the camera. An empty
    ///        collection of sources will stop all sources
    ///
    virtual bool stop_streams(const std::vector<DataSource> &sources) = 0;

    ///
    /// @brief Setup user callback that will be invoked whenever a new frame is received.
    ///
    virtual void add_image_frame_callback(std::function<void(const ImageFrame&)> callback) = 0;

    ///
    /// @brief Initialize the connection to the camera
    ///
    virtual bool connect(const ChannelConfig &config) = 0;

    ///
    /// @brief Disconnect from the camera
    ///
    virtual void disconnect() = 0;

    ///
    /// @brief A blocking call that waits for one image frame from the camera.
    ///
    /// If you’ve set a receive timeout (via ChannelConfig), it will block until that timeout expires;
    /// otherwise, it blocks indefinitely until data arrives.
    ///
    /// @return The newly received ImageFrame, or std::nullopt if timed out (and you used a timeout).
    ///
    virtual std::optional<ImageFrame> get_next_image_frame() = 0;

    ///
    /// @brief Get the current MultiSense configuration
    ///
    virtual MultiSenseConfiguration get_configuration() = 0;

    ///
    /// @brief Get set the current MultiSense configuration
    ///
    virtual bool set_configuration(const MultiSenseConfiguration &config) = 0;

    ///
    /// @brief Get the current stereo calibration. The output calibration will correspond to the full-resolution
    ///        operating mode of the camera
    ///
    virtual StereoCalibration get_calibration() = 0;

    ///
    /// @brief Set the current stereo calibration. The calibration is expected to be or the full-resolution
    ///        operating mode of the camera
    ///
    virtual bool set_calibration(const StereoCalibration &calibration) = 0;

    ///
    /// @brief Get the device info associated with the camera
    ///
    virtual DeviceInfo get_device_info() = 0;

    ///
    /// @brief Set the camera's device info. This setting is protected via a key since invalid values in the
    ///        device info can result in internal camera failures
    ///
    virtual bool set_device_info(const DeviceInfo &device_info, const std::string &key) = 0;
};

}
