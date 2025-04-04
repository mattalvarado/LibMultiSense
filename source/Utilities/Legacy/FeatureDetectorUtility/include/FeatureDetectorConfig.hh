/**
 * @file LibMultiSense/FeatureDetectorMessage.hh
 *
 * Copyright 2024-2025
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
 *   2024-22-11, patrick.smith@carnegierobotics.com, IRAD, Created file.
 *   2025-14-01, hshibata@carnegierobotics.com, IRAD, options added
 **/


#ifndef __FEATURE_DETECTOR_CONFIG_H__
#define __FEATURE_DETECTOR_CONFIG_H__

#include <MultiSense/details/utility/Portability.hh>
#include <MultiSense/MultiSenseTypes.hh>

using namespace crl::multisense::details;

#pragma pack(push, 1)

struct FeatureDetectorConfigParams {

    static CRL_CONSTEXPR wire::VersionType VERSION    = 2;

    static CRL_CONSTEXPR uint32_t OPT_USE_OBSERVER = 2;
    static CRL_CONSTEXPR uint32_t OPT_HYBRID_FULLRES = 4;
    static CRL_CONSTEXPR uint32_t OPT_HYBRID_LEFT_OFF = 8;
    static CRL_CONSTEXPR uint32_t OPT_HYBRID_RIGHT_OFF = 16;
    static CRL_CONSTEXPR uint32_t OPT_OBSERVER_INCREMENTAL = 32;
    static CRL_CONSTEXPR uint32_t OPT_HYBRID_MODE = 64;
    static CRL_CONSTEXPR uint32_t OPT_AUTO_AFFINE_CAL = 128;

    //
    // The message version
    wire::VersionType version;

    //
    // The maximum number of features detected per image
    uint32_t numberOfFeatures;

    //
    // Enable/Disable feature grouping
    bool grouping;

    //
    // Enable motion detection
    // Currently this functions as enable/disable but could be used to specify
    // which octave motion detection is performed on.
    // Current Octave: 3
    uint32_t motion;

    // Feature detector's internal options
    uint32_t options;

    FeatureDetectorConfigParams ( ) :
      version(VERSION),
      numberOfFeatures(1500),
      grouping(true),
      motion(1),
      options(0)
    {    }
};

#pragma pack(pop)

#endif /* end of include guard: __FEATURE_DETECTOR_CONFIG_H__ */
