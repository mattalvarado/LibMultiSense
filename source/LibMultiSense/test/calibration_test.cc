/**
 * @file calibration_test.hh
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
 *   2025-01-10, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/calibration.hh>

using namespace multisense::legacy;

crl::multisense::details::wire::CameraCalData create_valid_wire_cal(float fx, float fy, float cx, float cy)
{
    using namespace crl::multisense::details;

    wire::CameraCalData cal;
    memset(&cal.M[0][0], 0, sizeof(float) * 9);
    memset(&cal.R[0][0], 0, sizeof(float) * 9);
    memset(&cal.P[0][0], 0, sizeof(float) * 12);
    memset(&cal.D[0], 0, sizeof(float) * 8);

    cal.M[0][0] = fx;
    cal.M[0][2] = cx;
    cal.M[1][1] = fy;
    cal.M[1][2] = cy;
    cal.M[2][2] = 1.0;

    cal.R[0][0] = 1.0;
    cal.R[1][1] = 1.0;
    cal.R[2][2] = 1.0;

    cal.D[0] = 1.0;
    cal.D[1] = 0.1;
    cal.D[2] = 0.2;
    cal.D[3] = 2.0;
    cal.D[5] = 3.0;
    cal.D[6] = 3.0;
    cal.D[7] = 3.0;

    cal.P[0][0] = fx;
    cal.P[0][2] = cx;
    cal.P[1][1] = fy;
    cal.P[1][2] = cy;
    cal.P[2][2] = 1.0;

    return cal;
}

crl::multisense::details::wire::CameraCalData create_invalid_wire_cal()
{
    using namespace crl::multisense::details;

    wire::CameraCalData cal;
    memset(&cal.M[0][0], 0, sizeof(float) * 9);
    memset(&cal.R[0][0], 0, sizeof(float) * 9);
    memset(&cal.P[0][0], 0, sizeof(float) * 12);
    memset(&cal.D[0], 0, sizeof(float) * 8);

    return cal;
}

multisense::CameraCalibration create_valid_cal(float fx, float fy, float cx, float cy)
{
    multisense::CameraCalibration cal;

    cal.K = {{{fx, 0.0, cx}, {0.0, fy, cy}, {0.0, 0.0, 1.0}}};
    cal.R = {{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    cal.P = {{{fx, 0.0, cx, 0.0}, {0.0, fy, cy, 0.0}, {0.0, 0.0, 1.0, 0.0}}};

    cal.distortion_type = multisense::CameraCalibration::DistortionType::RATIONAL_POLYNOMIAL;
    cal.D = {1.0, 0.1, 0.2, 0.3, 0.4, 5.1, 6.2, 7.3};

    return cal;
}

multisense::CameraCalibration create_invalid_cal()
{
    multisense::CameraCalibration cal;
    memset(&cal.K[0][0], 0, sizeof(float) * 9);
    memset(&cal.R[0][0], 0, sizeof(float) * 9);
    memset(&cal.P[0][0], 0, sizeof(float) * 12);
    memset(&cal.D[0], 0, sizeof(float) * 8);

    return cal;
}

void check_equal(const crl::multisense::details::wire::CameraCalData &wire, const multisense::CameraCalibration &cal)
{
    for (size_t h = 0 ; h < 3 ; ++h)
    {
        for (size_t w = 0 ; w < 3 ; ++w)
        {
            ASSERT_DOUBLE_EQ(wire.M[h][w], cal.K[h][w]);
            ASSERT_DOUBLE_EQ(wire.R[h][w], cal.R[h][w]);
        }
    }

    for (size_t h = 0 ; h < 3 ; ++h)
    {
        for (size_t w = 0 ; w < 4 ; ++w)
        {
            ASSERT_DOUBLE_EQ(wire.P[h][w], cal.P[h][w]);
        }
    }

    for (size_t i = 0 ; i < cal.D.size() ; ++i)
    {
        ASSERT_DOUBLE_EQ(wire.D[i], cal.D[i]);
    }
}

TEST(is_valid, invalid)
{
    using namespace crl::multisense::details;

    wire::CameraCalData cal;
    ASSERT_FALSE(is_valid(cal));
}

TEST(is_valid, valid)
{
    using namespace crl::multisense::details;

    ASSERT_TRUE(is_valid(create_valid_wire_cal(800.0, 800.0, 400.0, 200.0)));
}

TEST(convert, cal_to_wire)
{
    const auto cal = create_valid_cal(800.0, 800.0, 400.0, 200.0);

    check_equal(convert(cal), cal);
}

TEST(convert, wire_to_cal)
{
    const auto cal = create_valid_wire_cal(800.0, 800.0, 400.0, 200.0);

    check_equal(cal, convert(cal));
}

TEST(convert, wire_to_stereo_valid_axu)
{
    using namespace crl::multisense::details;

    wire::SysCameraCalibration wire;
    wire.left = create_valid_wire_cal(800.0, 8001.0, 300.0, 200.0);
    wire.right = create_valid_wire_cal(801.0, 8001.0, 300.0, 200.0);
    wire.aux = create_valid_wire_cal(802.0, 8001.0, 300.0, 200.0);

    auto stereo = convert(wire);

    check_equal(wire.left, stereo.left);
    check_equal(wire.right, stereo.right);

    ASSERT_TRUE(static_cast<bool>(stereo.aux));
    check_equal(wire.aux, stereo.aux.value());
}

TEST(convert, wire_to_stereo_invalid_axu)
{
    using namespace crl::multisense::details;

    wire::SysCameraCalibration wire;
    wire.left = create_valid_wire_cal(800.0, 8001.0, 300.0, 200.0);
    wire.right = create_valid_wire_cal(801.0, 8001.0, 300.0, 200.0);
    wire.aux = create_invalid_wire_cal();

    auto stereo = convert(wire);

    check_equal(wire.left, stereo.left);
    check_equal(wire.right, stereo.right);

    ASSERT_FALSE(static_cast<bool>(stereo.aux));
}
