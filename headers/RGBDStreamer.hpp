#pragma once
#include <stdint.h>
#include <asio.hpp>
#include <turbojpeg.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include "UDPConnector.hpp"

namespace oi { namespace core { namespace rgbd {

	static std::chrono::milliseconds NOW();

	typedef struct RGBD_HEADER_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char msgType = 0x03;   // 00 // 0x03 = Depth; 0x04 = Color
		unsigned char deviceID = 0x00;  // 01
		unsigned short delta_t = 0;     // 02-03 Delta milliseconds
		unsigned int sequence = 0;      // 04-07
		unsigned short startRow = 0;    // 08-09
		unsigned short endRow = 0;      // 10-11
	} RGBD_HEADER_STRUCT;


	typedef struct META_STRUCT { // DO NOT REARRANGE THESE!
		std::chrono::milliseconds timestamp; // 00-07
		unsigned long memory_pos; // 08-15
		unsigned int frame_nr; // 16-19
		unsigned int payload_size; // 20-23
	} META_STRUCT;

	typedef struct CONFIG_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char msgType = 0x01;    // 00
		unsigned char deviceID = 0x00;   // 01
		unsigned char deviceType = 0x02; // 02
		unsigned char unused1 = 0x00;    // 03    => for consistent alignment
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
	} CONFIG_STRUCT;


	class StreamerConfig {
	public:
		void Parse(int argc, char *argv[]);

		bool useMatchMaking = false; // true
		std::string socketID = "kinect1";
		std::string remoteHost = "127.0.0.1"; // "mm.openimpress.org" "127.0.0.1"
		int remotePort = 10101; // 10101 6312
		int listenPort = 10102; // 10102 0
		std::string deviceSerial = "038994245147";
		std::string pipeline = "cuda";
		std::string fileDump = "";
		float maxDepth = 8.0f;
		std::string recordPath = "";// "D:/RGBD/"; //
	};

	class FileMeta {
		friend class RecordState;
	public:
		std::string name(); // recording name
		CONFIG_STRUCT config();

		std::chrono::milliseconds startTime(); // unix timestamp at start
		std::chrono::milliseconds endTime(); // unix timestamp at end
		std::chrono::milliseconds duration(); // recording duration in ms
		unsigned int frame_count(); // number of frames
		unsigned long getpos(unsigned long time); // return memory position of frame nearest t;
	private:
		void Load(std::string name, std::ifstream * in_meta);
		std::string _name;
		CONFIG_STRUCT _config;
		unsigned short _recording_version;
		std::vector<META_STRUCT> _frames;
		std::map<unsigned long, META_STRUCT*> _frames_by_time;// key is timestamp-timestamp[0]
	};


	class RecordState {
	friend class RGBDStreamer;
	public:
		RecordState();
		bool StopRecording();
		bool StartRecording(std::string name, oi::core::rgbd::CONFIG_STRUCT config);
		bool StartReplaying(std::string name);
		bool StopReplaying();
		bool recording();
		bool replaying();

		unsigned int next_rec_frame_nr();

		std::ostream * data_writer();
		std::ostream * meta_writer();
		std::ifstream * data_reader();
		CONFIG_STRUCT * replay_config();
		std::string PATH;
		const std::string DATA_SUFFIX =  ".rgbd.data";
		const std::string META_SUFFIX = ".rgbd.meta";
		RGBD_HEADER_STRUCT replay_header;
		FileMeta * current_replay_file;
		bool loop;
	private:
		bool LoadMeta(std::string name);
		std::map<std::string, FileMeta> files_meta;
		CONFIG_STRUCT _replay_config;
		unsigned short _replay_version;
		bool _recording;
		bool _replaying;
		std::string _current_recording_name = "";
		unsigned int _current_frame;
		std::ofstream _out_data;
		std::ofstream _out_meta;
		std::ifstream _in_data;
		std::string _next_frame;
		std::chrono::milliseconds _replay_next_frame;
	};

	class RGBDStreamer {
	public:
		RGBDStreamer(StreamerConfig cfg, oi::core::network::UDPBase * c);
		void ConfigStream();
		void HandleData();
		void Run();
		void Exit();
		virtual bool HandleData(oi::core::network::DataContainer * dc) = 0;
		virtual bool OpenDevice() = 0;
		virtual bool CloseDevice() = 0;
		virtual int SendFrame() = 0;
		int SendConfig();
		int ReplayNextFrame();

		RecordState record_state;
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


		std::chrono::milliseconds _prev_frame;

		// Logging
		unsigned int fpsCounter;
		unsigned int bytesCounter;
		std::chrono::milliseconds lastFpsAverage;
		const std::chrono::milliseconds interval{ 2000 };

	private:
		// Networking
		oi::core::network::UDPBase * client;
		unsigned int highest_sequence;
	};


} } }