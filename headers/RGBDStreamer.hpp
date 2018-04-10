#pragma once
#include <stdint.h>
#include <asio.hpp>
#include <turbojpeg.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include "UDPConnector.hpp"
#include "RecordState.hpp"

namespace oi { namespace core { namespace rgbd {

	const unsigned char RGBD_DATA = 1 << 0;
	const unsigned char AUDIO_DATA = 1 << 1;
	const unsigned char LIVE_DATA = 1 << 2;
	const unsigned char BODY_DATA = 1 << 3;
	const unsigned char HD_DATA = 1 << 4;
	const unsigned char BIDX_DATA = 1 << 5;

	std::chrono::milliseconds NOW();

	enum TimeSpecification { T_FRAME, T_REL, T_ABS };


	typedef struct _AUDIO_HEADER_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char   msgType = 0x07;   // 00
		unsigned char   unused = 0x00;    // 01
		unsigned short  frequency;        // 02-03
		unsigned short  channels;         // 04-05
		unsigned short  samples;          // 06-07
		unsigned long long timestamp;     // 08-15
	} AUDIO_HEADER_STRUCT;


	typedef struct _RGBD_HEADER_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char  msgType = 0x03;   // 00 // 0x03 = Depth; 0x04 = Color; // 0x33 = BIDX
		unsigned char  deviceID = 0x00;  // 01
		unsigned short delta_t = 0;      // 02-03 Delta milliseconds
		unsigned short startRow = 0;     // 04-05
		unsigned short endRow = 0;       // 06-07
		unsigned long long  timestamp;   // 08-15
	} RGBD_HEADER_STRUCT;


	typedef struct _META_STRUCT {             // DO NOT REARRANGE THESE!
		std::chrono::milliseconds timestamp;  // 00-07
		unsigned long long memory_pos_rgbd;   // 08-15
		unsigned long long memory_pos_audio;  // 16-23
		unsigned long long memory_pos_body;   // 24-31
		unsigned long long memory_pos_hd;     // 32-39
		unsigned long long memory_pos_bidx;   // 40-47
		unsigned int  frame_nr;               // 48-51
		unsigned int  payload_size;           // 52-55
	} META_STRUCT;

	typedef struct _BODY_STRUCT {          // DO NOT REARRANGE THESE!
		unsigned long tracking_id;         // 000-003
		unsigned char left_hand_state;     // 004
		unsigned char right_hand_state;    // 005
		unsigned char unused1;             // 006
		unsigned char lean_tracking_state; // 007
		float leanX;                       // 008-011
		float leanY;                       // 012-015
		float joints_position[3 * 25];     // 016-315 (25*3*4)
		unsigned char unused2;             // 316
		unsigned char unused3;             // 317
		unsigned char unused4;             // 318
		unsigned char joints_tracked[25];  // 319-343
	} BODY_STRUCT;

	typedef struct _BODY_HEADER_STRUCT {
		unsigned char  msgType = 0x05;   // 00 // 0x05 = Body
		unsigned char  unused1 = 0x00;   // 01
		unsigned short n_bodies;         // 02-03
		unsigned char  unused2 = 0x00;   // 04
		unsigned char  unused3 = 0x00;   // 05
		unsigned char  unused4 = 0x00;   // 06
		unsigned char  unused5 = 0x00;   // 07
		unsigned long long timestamp;    // 08-15
	} BODY_HEADER_STRUCT;

	typedef struct _CONFIG_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char msgType = 0x01;    // 00 
		unsigned char deviceID = 0x00;   // 01
		unsigned char deviceType = 0x02; // 02
		unsigned char dataFlags = 0x00;  // 03    => for consistent alignment
		unsigned short frameWidth = 0;   // 04-05
		unsigned short frameHeight = 0;  // 06-07
		unsigned short maxLines = 0;     // 08-09
		unsigned short unused1 = 0;      // 10-11 => for consistent alignment
		float Cx = 0.0f;                 // 12-15
		float Cy = 0.0f;                 // 16-19
		float Fx = 0.0f;                 // 20-23
		float Fy = 0.0f;                 // 24-27
		float DepthScale = 0.0f;         // 28-31
		float Px = 0.0f;                 // 32-35
		float Py = 0.0f;                 // 36-39
		float Pz = 0.0f;                 // 40-43
		float Qx = 0.0f;                 // 44-47
		float Qy = 0.0f;                 // 48-51
		float Qz = 0.0f;                 // 52-55
		float Qw = 1.0f;                 // 56-59
		unsigned char unused2 = 0x00;    // 60
		unsigned char unused3 = 0x00;    // 61 
		unsigned char unused4 = 0x00;    // 62 
		char guid[33] = "00000000000000000000000000000000"; // 63-95
		unsigned char unused5 = 0x00;    // 96
		unsigned char unused6 = 0x00;    // 97 
		unsigned char unused7 = 0x00;    // 98 
		char filename[33] = "00000000000000000000000000000000"; // 99-131
	} CONFIG_STRUCT;

	class ScheduledCommand {
	public:
		std::chrono::milliseconds time;
		nlohmann::json cmd;
		bool operator() (ScheduledCommand left, ScheduledCommand right);
	};

	class StreamerConfig {
	public:
		void Parse(int argc, char *argv[]);
		bool useMatchMaking = true;
		int debugLevel = 0;
		std::string socketID = "kinect1";
		std::string remoteHost = "mm.openimpress.org";
		int remotePort = 6312;
		int listenPort = 0;
		std::string deviceSerial = ""; // 
		std::string pipeline = "cpu";
		std::string fileSave = "";
		std::string fileLoad = "";
		float maxDepth = 8.0f;
		std::string filePath = "";// "D:/RGBD/"; //
	};

	class FileMeta {
		friend class RecordState;
	public:
		std::string name(); // recording name
        CONFIG_STRUCT _config;

		std::chrono::milliseconds frameTime(unsigned int frame);
		std::chrono::milliseconds startTime(); // unix timestamp at start
		std::chrono::milliseconds endTime(); // unix timestamp at end
		std::chrono::milliseconds duration(); // recording duration in ms
		unsigned int frame_count(); // number of frames
		unsigned int getFrameByTime(std::chrono::milliseconds t, TimeSpecification ts);
		unsigned long long getAudioPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;
		unsigned long long getRGBDPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;
		unsigned long long getBodyPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;
		unsigned long long getBIDXPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;

		std::vector<META_STRUCT> _frames;
	private:
		void Load(std::string name, std::ifstream * in_meta);
		std::string _name;
		unsigned short _recording_version;
		std::map<std::chrono::milliseconds, unsigned int> _frames_by_time;// key is timestamp-timestamp[0]
	};


	class RecordState {
		friend class RGBDStreamer;
	public:
		RecordState();
		bool StopRecording();
		bool StartRecording(std::string name, oi::core::rgbd::CONFIG_STRUCT config);
		bool StartReplaying(std::string name);
		bool StartReplaying(std::string name, std::chrono::milliseconds startSlice, std::chrono::milliseconds endSlice, TimeSpecification ts);
		bool StopReplaying();
		bool recording();
		bool replaying();

		unsigned int next_rec_frame_nr();

		std::ostream * mjpg_writer();
		std::ostream * bidx_writer();
		std::ostream * rgbd_writer();
		std::ostream * audio_writer();
		std::ostream * body_writer();
		std::ostream * meta_writer();
		std::istream * rgbd_reader();
		std::istream * audio_reader();
		std::istream * body_reader();
		std::istream * bidx_reader();
		CONFIG_STRUCT * replay_config();
		std::string PATH;
		const std::string RGBD_SUFFIX = ".oi.rgbd";
		const std::string META_SUFFIX = ".oi.meta";
		const std::string AUDIO_SUFFIX = ".oi.audio";
		const std::string BODY_SUFFIX = ".oi.body";
		const std::string MJPG_SUFFIX = ".oi.mjpg";
		const std::string BIDX_SUFFIX = ".oi.bidx";
		const std::string HD_SUFFIX = ".oi.mpeg";
		FileMeta * _current_replay_file;
		//oi::util::av::AVEncodeTest * avencode;
		void ReplayReset();
		bool loop;
	private:
		unsigned long long _pos_start_rgbd;
		unsigned long long _pos_end_rgbd;
		unsigned long long _pos_start_body;
		unsigned long long _pos_end_body;
		unsigned long long _pos_start_audio;
		unsigned long long _pos_end_audio;
		unsigned long long _pos_start_bidx;
		unsigned long long _pos_end_bidx;
		unsigned int _frame_start_rgbd;
		unsigned int _frame_end_rgbd;

		unsigned int _next_replay_frame;

		bool LoadMeta(std::string name);
		std::map<std::string, FileMeta> files_meta;
		//CONFIG_STRUCT _replay_config;
		unsigned short _replay_version;
		bool _recording;
		bool _replaying;
		std::string _current_recording_name = "";
		std::chrono::milliseconds _current_recording_start_time;
		unsigned int _current_frame;
		std::ofstream _out_rgbd;
		std::ofstream _out_audio;
		std::ofstream _out_body;
		std::ofstream _out_mjpg;
		std::ofstream _out_bidx;
		std::ofstream _out_meta;
		std::ifstream _in_rgbd;
		std::ifstream _in_audio;
		std::ifstream _in_body;
		std::ifstream _in_bidx;
		std::chrono::milliseconds _replay_next_frame;
		std::chrono::milliseconds _replay_start_time;
		unsigned long long _last_rgbd_pos;
	};

	class RGBDStreamer {
	public:
		RGBDStreamer(StreamerConfig cfg, oi::core::network::UDPBase * c);
		void ConfigStream();
		void HandleData();
		void HandleCommands();
		void Run();
		void Exit();
		virtual bool HandleData(oi::core::network::DataContainer * dc) = 0;
		virtual bool OpenDevice() = 0;
		virtual bool CloseDevice() = 0;
		virtual int SendFrame() = 0;
		int SendConfig();
		int SendConfig(CONFIG_STRUCT * config);
		int ReplayNextFrame();

		RecordState record_state;

	protected:
		int _SendAudioFrame(unsigned int sequence, float * samples, size_t n_samples, unsigned short freq, unsigned short channels, std::chrono::milliseconds timestamp);
		int _SendBodyFrame(BODY_STRUCT * bodies, unsigned short n_bodies, std::chrono::milliseconds timestamp);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata, std::chrono::milliseconds timestamp);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned short * depthdata, std::chrono::milliseconds timestamp);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depth_any, unsigned short * depth_ushort, std::chrono::milliseconds timestamp);
		
		int _SendHDFrame(unsigned char * rgbdata, int width, int height, TJPF pix_fmt, std::chrono::milliseconds timestamp);
		int _SendBodyIndexFrame(unsigned char * bidata, int width, int height, TJPF pix_fmt, std::chrono::milliseconds timestamp);

		std::priority_queue<ScheduledCommand, std::vector<ScheduledCommand>, ScheduledCommand> scheduled_commands;

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
		virtual bool supports_audio() = 0;
		virtual bool supports_body() = 0;
		virtual bool supports_bidx() = 0;
		virtual bool supports_hd() = 0;

        CONFIG_STRUCT stream_config;
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
		unsigned int audioSamplesCounter;
		std::chrono::milliseconds lastFpsAverage;
		const std::chrono::milliseconds interval{ 2000 };
		bool _is_open;
	private:
		// Networking
		oi::core::network::UDPBase * client;
		unsigned int highest_sequence;
	};


} } }
