#pragma once
#include <stdint.h>
#include <asio.hpp>
#include <turbojpeg.h>
#include <chrono>
#include "UDPConnector.hpp"

namespace oi { namespace core { namespace rgbd {

	class StreamerConfig {
	public:
		void Parse(int argc, char *argv[]);

		bool useMatchMaking = false;
		std::string socketID = "kinect1";
		std::string remoteHost = "127.0.0.1";
		int remotePort = 10101; // overwrite if mm=true;
		int listenPort = 10102; // if mm=true, don't use this default value
		std::string deviceSerial = "";
		std::string pipeline = "cuda";
		std::string fileDump = "";
		float maxDepth = 8.0f;
	};


	struct record_state {
		std::string file;
		unsigned long total_frames;
		unsigned long current_frame;
		bool loop;
	};

	struct RGBD_HEADER_STRUCT {
		unsigned char msgType = 0x03;   // 00 // 0x03 = Depth; 0x04 = Color
		unsigned char deviceID = 0x00;  // 01
		unsigned char unused1 = 0x00;   // 02 => for consistent alignment
		unsigned char unused2 = 0x00;   // 03 => for consistent alignment
		uint32_t sequence = 0;          // 04-07
		unsigned short startRow = 0;    // 08-09
		unsigned short endRow = 0;      // 10-11
	};

	struct CONFIG_STRUCT {
		unsigned char msgType = 0x01;    // 00
		unsigned char deviceID = 0x00;   // 01
		unsigned char deviceType = 0x02; // 02
		unsigned char unused1 = 0x00;    // 03 => for consistent alignment
		unsigned short frameWidth = 0;   // 04-05
		unsigned short frameHeight = 0;  // 06-07
		unsigned short maxLines = 0;     // 08-09
		unsigned short unused2 = 0;      // 10-11 => for consistent alignment
		float Cx = 0.0f;                 // 12-15
		float Cy = 0.0f;                 // 16-19
		float Fx = 0.0f;                 // 20-23
		float Fy = 0.0f;                 // 24-27
		float DepthScale = 0.0f;         // 28-31
		char guid[33] = "00000000000000000000000000000000"; // 32-...
	};


	class RGBDStreamer {
	public:
		RGBDStreamer(StreamerConfig cfg, oi::core::network::UDPBase * c);
		void ConfigStream();
		std::chrono::milliseconds NOW();
		void HandleData();
		void Run();
		void Exit();
		virtual bool HandleData(oi::core::network::DataContainer * dc) = 0;
		virtual bool OpenDevice() = 0;
		virtual bool CloseDevice() = 0;
		virtual int SendFrame() = 0;
		int SendConfig();
	protected:
		int _SendFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata);

		virtual int frame_width() = 0;
		virtual int frame_height() = 0;
		virtual int raw_depth_stride() = 0;
		virtual int raw_color_stride() = 0;

		virtual float device_cx() = 0;
		virtual float device_cy() = 0;
		virtual float device_fx() = 0;
		virtual float device_fy() = 0;
		virtual float device_depth_scale() = 0;
		virtual std::string device_guid() = 0;
		virtual TJPF color_pixel_format() = 0;



		RGBD_HEADER_STRUCT rgbd_header;
		CONFIG_STRUCT stream_config;
		unsigned int headerSize = sizeof(rgbd_header);
		unsigned char * config_msg_buf;
		StreamerConfig config;
		bool stream_shutdown;

		const int JPEG_QUALITY = 40;
		const int MAX_UDP_PACKET_SIZE = 65506;

		unsigned int linesPerMessage;

		// Recording specific
		std::map<std::string, record_state> recordings;
		std::chrono::milliseconds prevFrame;

		// Logging
		unsigned int fpsCounter;
		unsigned int bytesCounter;
		std::chrono::milliseconds lastFpsAverage;
		const std::chrono::milliseconds interval{ 2000 };

	private:
		// Networking
		oi::core::network::UDPBase * client;
	};


} } }