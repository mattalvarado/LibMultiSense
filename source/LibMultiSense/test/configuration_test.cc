/**
 * @file configuration_test.hh
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
 *   2025-01-26, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/configuration.hh>

using namespace multisense::legacy;

multisense::MultiSenseConfiguration create_valid_config()
{
    using namespace multisense;
    using namespace std::chrono_literals;

    MultiSenseConfiguration::StereoConfiguration stereo_config{0.8};

    MultiSenseConfiguration::ManualExposureConfiguration manual_exposure{2.0, 500us};

    MultiSenseConfiguration::AutoExposureConfiguration auto_exposure{600us,
                                                                     6,
                                                                     0.6,
                                                                     0.95,
                                                                     3.0,
                                                                     {10, 20, 400, 300}};

    MultiSenseConfiguration::ManualWhiteBalanceConfiguration manual_white_balance{2.2, 3.3};

    MultiSenseConfiguration::AutoWhiteBalanceConfiguration auto_white_balance{4, 0.7};

    MultiSenseConfiguration::ImageConfiguration image_config{1.2,
                                                             true,
                                                             false,
                                                             manual_exposure,
                                                             auto_exposure,
                                                             true,
                                                             manual_white_balance,
                                                             auto_white_balance};

    MultiSenseConfiguration::AuxConfiguration aux_config{image_config, true, 10.0, 80};

    MultiSenseConfiguration::TimeConfig time{true};

    return MultiSenseConfiguration{1920,
                                   1200,
                                   MultiSenseConfiguration::MaxDisparities::D256,
                                   11.0,
                                   stereo_config,
                                   image_config,
                                   aux_config,
                                   time};
}

crl::multisense::details::wire::CamConfig create_valid_wire_config()
{
    using namespace crl::multisense::details;

    wire::CamConfig config;
    config.width = 1920;
    config.height = 1200;
    config.disparities = 256;
    config.stereoPostFilterStrength = 0.8;
    config.gain = 2.0;
    config.exposure = 500;

    config.autoExposure = 0;
    config.autoExposureMax = 600;
    config.autoExposureDecay = 6;
    config.autoExposureTargetIntensity = 0.6;
    config.autoExposureThresh = 0.95;
    config.gainMax = 3.0;
    config.autoExposureRoiX = 10;
    config.autoExposureRoiY = 20;
    config.autoExposureRoiWidth = 400;
    config.autoExposureRoiHeight = 300;

    config.autoWhiteBalance = 1;
    config.whiteBalanceRed = 2.2;
    config.whiteBalanceBlue = 3.3;
    config.autoWhiteBalanceDecay = 4;
    config.autoWhiteBalanceThresh = 0.7;

    config.gamma = 1.2;
    config.hdrEnabled = true;

    return config;
}

crl::multisense::details::wire::AuxCamConfig create_valid_wire_aux_config()
{
    using namespace crl::multisense::details;

    wire::AuxCamConfig config;
    config.gain = 2.0;
    config.exposure = 500;

    config.autoExposure = 0;
    config.autoExposureMax = 600;
    config.autoExposureDecay = 6;
    config.autoExposureTargetIntensity = 0.6;
    config.autoExposureThresh = 0.95;
    config.gainMax = 3.0;
    config.autoExposureRoiX = 10;
    config.autoExposureRoiY = 20;
    config.autoExposureRoiWidth = 400;
    config.autoExposureRoiHeight = 300;

    config.autoWhiteBalance = 1;
    config.whiteBalanceRed = 2.2;
    config.whiteBalanceBlue = 3.3;
    config.autoWhiteBalanceDecay = 4;
    config.autoWhiteBalanceThresh = 0.7;

    config.sharpeningEnable = true;
    config.sharpeningPercentage = 10.0;
    config.sharpeningLimit = 80;

    return config;
}

void check_equal(const multisense::MultiSenseConfiguration &config,
                 const crl::multisense::details::wire::CamSetResolution &res)
{
    using namespace multisense;

    ASSERT_EQ(config.width, res.width);
    ASSERT_EQ(config.height, res.height);

    switch(config.disparities)
    {
        case MultiSenseConfiguration::MaxDisparities::D64: {ASSERT_EQ(64, res.disparities); break;}
        case MultiSenseConfiguration::MaxDisparities::D128: {ASSERT_EQ(128, res.disparities); break;}
        case MultiSenseConfiguration::MaxDisparities::D256: {ASSERT_EQ(256, res.disparities); break;}
    }
}

template <typename ConfigT, typename ControlT>
void check_equal(const ConfigT &config,
                 const ControlT &control)
{
    ASSERT_FLOAT_EQ(config.manual_exposure.gain, control.gain);
    ASSERT_EQ(config.manual_exposure.exposure_time.count(), control.exposure);

    ASSERT_EQ(config.auto_exposure_enabled, control.autoExposure);
    ASSERT_EQ(config.auto_exposure.max_exposure_time.count(), control.autoExposureMax);
    ASSERT_EQ(config.auto_exposure.decay, control.autoExposureDecay);
    ASSERT_FLOAT_EQ(config.auto_exposure.target_threshold, control.autoExposureThresh);
    ASSERT_FLOAT_EQ(config.auto_exposure.target_intensity, control.autoExposureTargetIntensity);
    ASSERT_FLOAT_EQ(config.auto_exposure.max_gain, control.gainMax);

    ASSERT_EQ(config.auto_exposure.roi.top_left_x_position, control.autoExposureRoiX);
    ASSERT_EQ(config.auto_exposure.roi.top_left_y_position, control.autoExposureRoiY);
    ASSERT_EQ(config.auto_exposure.roi.width, control.autoExposureRoiWidth);
    ASSERT_EQ(config.auto_exposure.roi.height, control.autoExposureRoiHeight);

    ASSERT_FLOAT_EQ(config.manual_white_balance.red, control.whiteBalanceRed);
    ASSERT_FLOAT_EQ(config.manual_white_balance.blue, control.whiteBalanceBlue);

    ASSERT_EQ(config.auto_white_balance_enabled, control.autoWhiteBalance);
    ASSERT_EQ(config.auto_white_balance.decay, control.autoWhiteBalanceDecay);
    ASSERT_EQ(config.auto_white_balance.threshold, control.autoWhiteBalanceThresh);

    ASSERT_EQ(config.hdr_enabled, control.hdrEnabled);
    ASSERT_EQ(config.gamma, control.gamma);
}

void check_equal(const multisense::MultiSenseConfiguration &config,
                 const crl::multisense::details::wire::CamControl &control)
{
    using namespace multisense;

    ASSERT_FLOAT_EQ(config.frames_per_second, control.framesPerSecond);

    check_equal(config.image_config, control);

    ASSERT_FLOAT_EQ(config.stereo_config.postfilter_strength, control.stereoPostFilterStrength);
}

void check_equal(const multisense::MultiSenseConfiguration &config,
                 const crl::multisense::details::wire::AuxCamControl &control)
{
    using namespace multisense;

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    check_equal(config.aux_config->image_config, control);

    ASSERT_EQ(config.aux_config->sharpening_enabled, control.sharpeningEnable);
    ASSERT_FLOAT_EQ(config.aux_config->sharpening_percentage, control.sharpeningPercentage);
    ASSERT_FLOAT_EQ(config.aux_config->sharpening_limit, control.sharpeningLimit);
}

void check_equal(const multisense::MultiSenseConfiguration &config,
                 const crl::multisense::details::wire::CamConfig &wire_config)
{
    using namespace multisense;

    ASSERT_FLOAT_EQ(config.stereo_config.postfilter_strength, wire_config.stereoPostFilterStrength);
    ASSERT_EQ(config.width, wire_config.width);
    ASSERT_EQ(config.height, wire_config.height);

    switch(config.disparities)
    {
        case MultiSenseConfiguration::MaxDisparities::D64: {ASSERT_EQ(64, wire_config.disparities); break;}
        case MultiSenseConfiguration::MaxDisparities::D128: {ASSERT_EQ(128, wire_config.disparities); break;}
        case MultiSenseConfiguration::MaxDisparities::D256: {ASSERT_EQ(256, wire_config.disparities); break;}
    }

    ASSERT_EQ(config.frames_per_second, wire_config.framesPerSecond);

    check_equal(config.image_config, wire_config);
}

void check_equal(const multisense::MultiSenseConfiguration::AuxConfiguration &config,
                 const crl::multisense::details::wire::AuxCamConfig &wire_config)
{
    check_equal(config.image_config, wire_config);

    ASSERT_EQ(config.sharpening_enabled, wire_config.sharpeningEnable);
    ASSERT_FLOAT_EQ(config.sharpening_percentage, wire_config.sharpeningPercentage);
    ASSERT_FLOAT_EQ(config.sharpening_limit, wire_config.sharpeningLimit);
}

TEST(convert, cam_resolution)
{
    using namespace crl::multisense::details;

    const auto config = create_valid_config();

    const auto wire_resolution = convert<wire::CamSetResolution>(config);

    check_equal(config, wire_resolution);
}

TEST(convert, cam_control)
{
    using namespace crl::multisense::details;

    const auto config = create_valid_config();

    const auto cam_control = convert<wire::CamControl>(config);

    check_equal(config, cam_control);
}

TEST(convert, aux_cam_control)
{
    using namespace crl::multisense::details;

    const auto config = create_valid_config();

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    const auto cam_control = convert(config.aux_config.value());

    check_equal(config, cam_control);
}


TEST(convert, cam_config)
{
    const auto wire_config = create_valid_wire_config();
    const auto wire_aux_config = create_valid_wire_aux_config();

    const auto config = convert(wire_config, wire_aux_config, false);

    ASSERT_TRUE(static_cast<bool>(config.aux_config));

    check_equal(config, wire_config);
    check_equal(config.aux_config.value(), wire_aux_config);
}

TEST(convert, cam_config_invalid_aux)
{
    const auto wire_config = create_valid_wire_config();

    const auto config = convert(wire_config, std::nullopt, false);

    ASSERT_FALSE(static_cast<bool>(config.aux_config));

    check_equal(config, wire_config);
}
