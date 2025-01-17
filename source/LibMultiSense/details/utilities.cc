/**
 * @file utilities.cc
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

#include <stdexcept>

#ifdef HAVE_OPENCV
#include <opencv2/imgcodecs.hpp>
#endif

#include "MultiSense/MultiSenseUtilities.hh"

namespace multisense
{
#ifdef HAVE_OPENCV
cv::Mat to_cv_mat(const Image &image)
{
    int cv_type = 0;
    switch(image.format)
    {
        case PixelFormat::MONO8: {cv_type = CV_8UC1; break;}
        case PixelFormat::RGB8: {cv_type = CV_8UC3; break;}
        case PixelFormat::MONO16: {cv_type = CV_16UC1; break;}
        default: {throw std::runtime_error("invalid pixel format");}
    }

    return cv::Mat{image.height,
                   image.width,
                   cv_type,
                   const_cast<uint8_t*>(image.raw_data->data() + image.image_data_offset)};
}
#endif

bool write_image(const Image &image, const std::filesystem::path &path)
{
#ifdef HAVE_OPENCV
    return cv::imwrite(path.string(), to_cv_mat(image));
#endif
    return false;
}
}
