/*
This file is part of the OpenIMPRESS project.

OpenIMPRESS is free software: you can redistribute it and/or modify
it under the terms of the Lesser GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OpenIMPRESS is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with OpenIMPRESS. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include <RGBDDevice.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

namespace oi { namespace client { namespace libfreenect2 {

class LibFreenect2DeviceInterface : public oi::core::rgbd::RGBDDeviceInterface {
public:
    LibFreenect2DeviceInterface(std::string serial, std::string pipeline_name, float minDepth, float maxDepth);
    int OpenDevice();
    int CloseDevice();
    int Cycle(oi::core::rgbd::RGBDDevice * streamer);
    
    int frame_width();
    int frame_height();
    int send_depth_stride();
    int raw_depth_stride();
    int raw_color_stride();
    
    float device_cx();
    float device_cy();
    float device_fx();
    float device_fy();
    float device_depth_scale();
    
    bool supports_audio();
    bool supports_body();
    bool supports_bidx();
    bool supports_hd();
    bool supports_depth();
    
    std::string device_guid();
    TJPF color_pixel_format();
    
private:
    ::libfreenect2::Freenect2 * freenect2;
    ::libfreenect2::Freenect2Device * dev = 0;
    ::libfreenect2::Registration* registration;
    ::libfreenect2::Frame * undistorted;
    ::libfreenect2::Frame * registered;
    ::libfreenect2::Freenect2Device::ColorCameraParams i_c;
    ::libfreenect2::Freenect2Device::IrCameraParams i_d;
    ::libfreenect2::SyncMultiFrameListener * listener;
    ::libfreenect2::FrameMap * frames;
    
    std::string serial;
    std::string pipeline_name;
    float minDepth;
    float maxDepth;
};

} } }
