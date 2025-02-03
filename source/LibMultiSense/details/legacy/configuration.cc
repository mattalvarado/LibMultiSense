/**
 * @file configuration.cc
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
 *   2025-01-18, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include "details/legacy/configuration.hh"
#include "details/legacy/utilities.hh"

namespace multisense {
namespace legacy {

MultiSenseConfiguration convert(const crl::multisense::details::wire::CamConfig &config,
                                const std::optional<crl::multisense::details::wire::AuxCamConfig> &aux_config,
                                const std::optional<crl::multisense::details::wire::ImuConfig> &imu_config,
                                bool ptp_enabled)
{
    using namespace crl::multisense::details;

    using ms_config = MultiSenseConfiguration;

    ms_config::StereoConfiguration stereo{config.stereoPostFilterStrength};

    ms_config::ManualExposureConfiguration manual_exposure{config.gain,
                                                           std::chrono::microseconds{config.exposure}};

    ms_config::AutoExposureRoiConfiguration auto_exposure_roi{config.autoExposureRoiX,
                                                              config.autoExposureRoiY,
                                                              config.autoExposureRoiWidth,
                                                              config.autoExposureRoiHeight};

    ms_config::AutoExposureConfiguration auto_exposure{std::chrono::microseconds{config.autoExposureMax},
                                                       config.autoExposureDecay,
                                                       config.autoExposureTargetIntensity,
                                                       config.autoExposureThresh,
                                                       config.gainMax,
                                                       std::move(auto_exposure_roi)};

    ms_config::ManualWhiteBalanceConfiguration manual_white_balance{config.whiteBalanceRed, config.whiteBalanceBlue};
    ms_config::AutoWhiteBalanceConfiguration auto_white_balance{config.autoWhiteBalanceDecay,
                                                                config.autoWhiteBalanceThresh};

    ms_config::ImageConfiguration image{config.gamma,
                                        config.hdrEnabled,
                                        (config.autoExposure != 0),
                                        std::move(manual_exposure),
                                        std::move(auto_exposure),
                                        (config.autoWhiteBalance != 0),
                                        std::move(manual_white_balance),
                                        std::move(auto_white_balance)};

    return MultiSenseConfiguration{config.width,
                                   config.height,
                                   get_disparities(config.disparities),
                                   config.framesPerSecond,
                                   std::move(stereo),
                                   std::move(image),
                                   (aux_config ? std::make_optional(convert(aux_config.value())) : std::nullopt),
                                   ms_config::TimeConfiguration{ptp_enabled},
                                   imu_config ? std::make_optional(convert(imu_config.value())) : std::nullopt};
}

MultiSenseConfiguration::AuxConfiguration convert(const crl::multisense::details::wire::AuxCamConfig &config)
{
    using namespace crl::multisense::details;
    using ms_config = MultiSenseConfiguration;

    ms_config::ManualExposureConfiguration manual_exposure{config.gain,
                                                           std::chrono::microseconds{config.exposure}};

    ms_config::AutoExposureRoiConfiguration auto_exposure_roi{config.autoExposureRoiX,
                                                              config.autoExposureRoiY,
                                                              config.autoExposureRoiWidth,
                                                              config.autoExposureRoiHeight};

    ms_config::AutoExposureConfiguration auto_exposure{std::chrono::microseconds{config.autoExposureMax},
                                                       config.autoExposureDecay,
                                                       config.autoExposureTargetIntensity,
                                                       config.autoExposureThresh,
                                                       config.gainMax,
                                                       std::move(auto_exposure_roi)};

    ms_config::ManualWhiteBalanceConfiguration manual_white_balance{config.whiteBalanceRed, config.whiteBalanceBlue};
    ms_config::AutoWhiteBalanceConfiguration auto_white_balance{config.autoWhiteBalanceDecay,
                                                                config.autoWhiteBalanceThresh};

    ms_config::ImageConfiguration image{config.gamma,
                                        config.hdrEnabled,
                                        (config.autoExposure != 0),
                                        std::move(manual_exposure),
                                        std::move(auto_exposure),
                                        (config.autoWhiteBalance != 0),
                                        std::move(manual_white_balance),
                                        std::move(auto_white_balance)};

    return ms_config::AuxConfiguration{std::move(image),
                                       config.sharpeningEnable,
                                       config.sharpeningPercentage,
                                       config.sharpeningLimit};
}

template <>
crl::multisense::details::wire::CamSetResolution convert<crl::multisense::details::wire::CamSetResolution>(const MultiSenseConfiguration &config)
{
    using namespace crl::multisense::details;

    int32_t disparities = 256;

    switch(config.disparities)
    {
        case MultiSenseConfiguration::MaxDisparities::D64: {disparities = 64; break;}
        case MultiSenseConfiguration::MaxDisparities::D128: {disparities = 128; break;}
        case MultiSenseConfiguration::MaxDisparities::D256: {disparities = 256; break;}
    }

    wire::CamSetResolution output{config.width, config.height, disparities};

    return output;
}

template <>
crl::multisense::details::wire::CamControl convert<crl::multisense::details::wire::CamControl>(const MultiSenseConfiguration &config)
{
    using namespace crl::multisense::details;

    wire::CamControl output;

    output.framesPerSecond = config.frames_per_second;
    output.gain = config.image_config.manual_exposure.gain;
    output.exposure = config.image_config.manual_exposure.exposure_time.count();

    output.autoExposure = config.image_config.auto_exposure_enabled;
    output.autoExposureMax = config.image_config.auto_exposure.max_exposure_time.count();
    output.autoExposureDecay = config.image_config.auto_exposure.decay;
    output.autoExposureThresh = config.image_config.auto_exposure.target_threshold;
    output.autoExposureTargetIntensity = config.image_config.auto_exposure.target_intensity;
    output.gainMax = config.image_config.auto_exposure.max_gain;

    output.autoExposureRoiX = config.image_config.auto_exposure.roi.top_left_x_position;
    output.autoExposureRoiY = config.image_config.auto_exposure.roi.top_left_y_position;
    output.autoExposureRoiWidth = config.image_config.auto_exposure.roi.width;
    output.autoExposureRoiHeight = config.image_config.auto_exposure.roi.height;

    output.whiteBalanceRed = config.image_config.manual_white_balance.red;
    output.whiteBalanceBlue = config.image_config.manual_white_balance.blue;

    output.autoWhiteBalance = config.image_config.auto_white_balance_enabled;
    output.autoWhiteBalanceDecay = config.image_config.auto_white_balance.decay;
    output.autoWhiteBalanceThresh  = config.image_config.auto_white_balance.threshold;

    output.stereoPostFilterStrength = config.stereo_config.postfilter_strength;

    output.hdrEnabled = config.image_config.hdr_enabled;
    output.gamma = config.image_config.gamma;

    return output;
}

crl::multisense::details::wire::AuxCamControl convert(const MultiSenseConfiguration::AuxConfiguration &config)
{
    using namespace crl::multisense::details;

    wire::AuxCamControl output;

    output.gain = config.image_config.manual_exposure.gain;
    output.exposure = config.image_config.manual_exposure.exposure_time.count();

    output.autoExposure = config.image_config.auto_exposure_enabled;
    output.autoExposureMax = config.image_config.auto_exposure.max_exposure_time.count();
    output.autoExposureDecay = config.image_config.auto_exposure.decay;
    output.autoExposureThresh = config.image_config.auto_exposure.target_threshold;
    output.autoExposureTargetIntensity = config.image_config.auto_exposure.target_intensity;
    output.gainMax = config.image_config.auto_exposure.max_gain;

    output.autoExposureRoiX = config.image_config.auto_exposure.roi.top_left_x_position;
    output.autoExposureRoiY = config.image_config.auto_exposure.roi.top_left_y_position;
    output.autoExposureRoiWidth = config.image_config.auto_exposure.roi.width;
    output.autoExposureRoiHeight = config.image_config.auto_exposure.roi.height;

    output.whiteBalanceRed = config.image_config.manual_white_balance.red;
    output.whiteBalanceBlue = config.image_config.manual_white_balance.blue;

    output.autoWhiteBalance = config.image_config.auto_white_balance_enabled;
    output.autoWhiteBalanceDecay = config.image_config.auto_white_balance.decay;
    output.autoWhiteBalanceThresh  = config.image_config.auto_white_balance.threshold;

    output.hdrEnabled = config.image_config.hdr_enabled;
    output.gamma = config.image_config.gamma;
    output.sharpeningEnable = config.sharpening_enabled;
    output.sharpeningPercentage = config.sharpening_percentage;
    output.sharpeningLimit = config.sharpening_limit;

    //
    // Currently unsupported values
    //
    output.cameraProfile = 0;

    return output;
}

crl::multisense::details::wire::SysSetPtp convert(const MultiSenseConfiguration::TimeConfiguration &config)
{
    using namespace crl::multisense::details;

    wire::SysSetPtp output;
    output.enable = config.ptp_enabled ? 1 : 0;

    return output;
}

MultiSenseConfiguration::ImuConfiguration convert(const crl::multisense::details::wire::ImuConfig &imu)
{
    using namespace crl::multisense::details;
    using ImuConfiguration = MultiSenseConfiguration::ImuConfiguration;

    std::vector<ImuConfiguration::OperatingMode> modes;
    for (const auto &element : imu.configs)
    {
        modes.emplace_back(ImuConfiguration::OperatingMode{element.name,
                                                           static_cast<bool>(element.flags & wire::imu::Config::FLAGS_ENABLED),
                                                           element.rateTableIndex,
                                                           element.rangeTableIndex});
    }

    return ImuConfiguration{imu.samplesPerMessage, std::move(modes)};
}

crl::multisense::details::wire::ImuConfig convert(const MultiSenseConfiguration::ImuConfiguration &imu,
                                                  uint32_t max_samples_per_message)
{
    using namespace crl::multisense::details;

    wire::ImuConfig output;

    output.samplesPerMessage = std::min(max_samples_per_message, imu.samples_per_frame);

    std::vector<wire::imu::Config> configs;
    for (const auto &mode : imu.modes)
    {
        wire::imu::Config config;
        config.name = mode.name;
        config.flags = mode.enabled ? wire::imu::Config::FLAGS_ENABLED : 0;
        config.rateTableIndex = mode.rate_index;
        config.rangeTableIndex = mode.range_index;

        configs.emplace_back(std::move(config));
    }

    output.storeSettingsInFlash = false;
    output.configs= std::move(configs);

    return output;
}

}
}
