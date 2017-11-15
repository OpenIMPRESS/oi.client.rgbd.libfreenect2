#pragma once
#include <stdint.h>
#include <asio.hpp>
#include <turbojpeg.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include "UDPConnector.hpp"

namespace oi { namespace core { namespace rgbd {

	const unsigned char RGBD_DATA =  1 << 0;
	const unsigned char AUDIO_DATA = 1 << 1;
	const unsigned char LIVE_DATA =  1 << 2;

	enum TimeSpecification { FRAME, REL, ABS };

	static std::chrono::milliseconds NOW();

	typedef struct AUDIO_HEADER_STRUCT {
		unsigned char msgType = 0x07; // 00
		unsigned char unused = 0x00;  // 01
		unsigned short frequency;     // 02-03
		unsigned short channels;      // 04-05
		unsigned short samples;       // 06-07
		unsigned int sequence;        // 08-11
	} AUDIO_HEADER_STRUCT;


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
		unsigned long memory_pos_rgbd; // 08-15
		unsigned long memory_pos_audio; // 08-15
		unsigned int frame_nr; // 16-19
		unsigned int payload_size; // 20-23
	} META_STRUCT;

	class ScheduledCommand {
	public:
		std::chrono::milliseconds time;
		nlohmann::json cmd;
		bool operator() (ScheduledCommand left, ScheduledCommand right);
	};

	typedef struct CONFIG_STRUCT { // DO NOT REARRANGE THESE!
		unsigned char msgType = 0x11;    // 00 - ConfigBETA!!
		unsigned char deviceID = 0x00;   // 01
		unsigned char deviceType = 0x02; // 02
		unsigned char dataFlags = 0x00;  // 03    => for consistent alignment
		unsigned short frameWidth = 0;   // 04-05
		unsigned short frameHeight = 0;  // 06-07
		unsigned short maxLines = 0;     // 08-09
		unsigned short unused2 = 0;      // 10-11 => for consistent alignment
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
		char guid[33] = "00000000000000000000000000000000"; // 60-92
	} CONFIG_STRUCT;


	class StreamerConfig {
	public:
		void Parse(int argc, char *argv[]);
		bool useMatchMaking = true;
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
		CONFIG_STRUCT config();

		std::chrono::milliseconds startTime(); // unix timestamp at start
		std::chrono::milliseconds endTime(); // unix timestamp at end
		std::chrono::milliseconds duration(); // recording duration in ms
		unsigned int frame_count(); // number of frames
		unsigned long getAudioPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;
		unsigned long getRGBDPosition(std::chrono::milliseconds t, TimeSpecification ts); // return memory position of frame nearest t;
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
		bool StartReplaying(std::string name, std::chrono::milliseconds startSlice, std::chrono::milliseconds endSlice, TimeSpecification ts);
		bool StopReplaying();
		bool recording();
		bool replaying();

		unsigned int next_rec_frame_nr();

		std::ofstream * rgbd_writer();
		std::ofstream * audio_writer();
		std::ofstream * meta_writer();
		std::ifstream * rgbd_reader();
		std::ifstream * audio_reader();
		CONFIG_STRUCT * replay_config();
		std::string PATH;
		const std::string RGBD_SUFFIX =  ".oi.rgbd";
		const std::string META_SUFFIX = ".oi.meta";
		const std::string AUDIO_SUFFIX = ".oi.audio";
		RGBD_HEADER_STRUCT replay_rgbd_header;
		AUDIO_HEADER_STRUCT replay_audio_header;
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
		std::ofstream _out_rgbd;
		std::ofstream _out_audio;
		std::ofstream _out_meta;
		std::ifstream _in_rgbd;
		std::ifstream _in_audio;
		std::chrono::milliseconds _replay_next_frame;
		std::chrono::milliseconds _replay_start_time;
		unsigned long _last_rgbd_pos;
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
		int _SendAudioFrame(unsigned int sequence, float * samples, size_t n_samples, unsigned short freq, unsigned short channels);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned short * depthdata);
		int _SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depth_any, unsigned short * depth_ushort);
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

		RGBD_HEADER_STRUCT rgbd_header;
		AUDIO_HEADER_STRUCT audio_header;
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
		unsigned int audioSamplesCounter;
		std::chrono::milliseconds lastFpsAverage;
		const std::chrono::milliseconds interval{ 2000 };

	private:
		// Networking
		oi::core::network::UDPBase * client;
		unsigned int highest_sequence;
	};


} } }
