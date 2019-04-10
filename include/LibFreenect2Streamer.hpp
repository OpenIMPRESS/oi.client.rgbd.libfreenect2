#pragma once
#include <asio.hpp>
#include <turbojpeg.h>
#include <cstdlib>
#include <signal.h>

#include "UDPConnector.hpp"
#include "RGBDStreamer.hpp"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


namespace oi { namespace core { namespace rgbd {

	class LibFreenect2Streamer : public RGBDStreamer {
	public:
		LibFreenect2Streamer(StreamerConfig cfg, oi::core::network::UDPBase * c);
		bool OpenDevice();
		bool CloseDevice();
		bool HandleData(oi::core::network::DataContainer * dc) ;
		int SendFrame();
	protected:
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
        
		std::string serial;
		std::string device_guid();
		TJPF color_pixel_format();
        
        bool supports_audio();
        bool supports_body();
        bool supports_bidx();
        bool supports_hd();

		libfreenect2::Freenect2 * freenect2;
		libfreenect2::Freenect2Device * dev = 0;
		libfreenect2::Registration* registration;
		libfreenect2::Frame * undistorted;
		libfreenect2::Frame * registered;
		libfreenect2::Freenect2Device::IrCameraParams i_d;
		libfreenect2::SyncMultiFrameListener * listener;
		libfreenect2::FrameMap * frames;
	};

} } }
