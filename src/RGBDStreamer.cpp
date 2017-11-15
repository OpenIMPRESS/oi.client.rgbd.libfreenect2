#include "RGBDStreamer.hpp"
#include <string>
#include <algorithm> 

namespace oi { namespace core { namespace rgbd {

	static std::chrono::milliseconds NOW() {
		return std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		);
	}

	RGBDStreamer::RGBDStreamer(StreamerConfig cfg, oi::core::network::UDPBase * c) {
		fpsCounter = 0;
		bytesCounter = 0;
		audioSamplesCounter = 0;
		lastFpsAverage = NOW();
		this->config = cfg;
		record_state.PATH = cfg.filePath;
		client = c;
	}

	void RGBDStreamer::Run() {
		OpenDevice();
		ConfigStream();

		stream_shutdown = false;
		while (!stream_shutdown) {
			int sentBytes = 0;

			// TODO:
			//		if we both record and REPLAY, can't we replay in a thread while waiting for frames?
			//  std::thread replayFrameThread(ReplayNextFrame, this);
			// ...and join it at the end?
			//   replayFrameThread.join()?
			if (record_state.recording() || (!record_state.recording() && !record_state.replaying())) {

				// Write meta files
				std::ostream * meta_writer = record_state.meta_writer();
				std::ostream * rgbd_writer = record_state.rgbd_writer();
				std::ostream * audio_writer = record_state.audio_writer();

				if ((unsigned long) rgbd_writer->tellp() == record_state._last_rgbd_pos) {
					// no new depth frame, don't update meta.
				} else {
					META_STRUCT frame_meta;
					frame_meta.timestamp = NOW();
					frame_meta.memory_pos_rgbd = rgbd_writer->tellp();
					record_state._last_rgbd_pos = frame_meta.memory_pos_rgbd;
					if (supports_audio())
						 frame_meta.memory_pos_audio = audio_writer->tellp();
					else frame_meta.memory_pos_audio = 0;
					frame_meta.frame_nr = record_state.next_rec_frame_nr();
					frame_meta.payload_size = 0;
					meta_writer->write((const char*)&frame_meta, sizeof(frame_meta)); // [memory_pos (unsigned long=8bytes)]
				}
				sentBytes += SendFrame(); // SendFrame implementations call _SendDepthFrame & _SendAudioFrame
			}

			if (record_state.replaying() && NOW() >= record_state._replay_next_frame) {
				sentBytes += ReplayNextFrame();
			}

			if (sentBytes > 0) {
				bytesCounter += sentBytes;
				fpsCounter++;
			} else if (sentBytes == -1) {
				stream_shutdown = true;
				continue;
			}

			HandleData(); // Parse & schedule commands

			HandleCommands();

			if (lastFpsAverage + interval <= NOW()) {
				lastFpsAverage = NOW();
				double timeDelta = (interval.count() / 1000.0);

				printf("\r >> Streaming at %5.2f Fps - Data: %5.2f Mbps - Audio: %5.2f Samples/s", fpsCounter / timeDelta, ((bytesCounter / 1024.0) / 1024) / timeDelta, audioSamplesCounter / timeDelta);
				std::cout << std::flush;

				fpsCounter = 0;
				bytesCounter = 0;
				audioSamplesCounter = 0;
				if (!record_state.replaying() && SendConfig() <= 0) stream_shutdown = true;
			}

		}
	}

	void RGBDStreamer::HandleCommands() {
		if (scheduled_commands.size() < 1) return;
		std::chrono::milliseconds t = NOW();
		if (scheduled_commands.top().time <= t) {
			nlohmann::json msg = scheduled_commands.top().cmd;
			scheduled_commands.pop();

			std::cout << "\nex: |" << msg.dump(2) << "|" << std::endl;

			if (msg["cmd"] == "application") {
				if (msg["val"] == "stop") {
					stream_shutdown = true;
				}
			}

			if (msg["cmd"] == "record") {
				if (msg["val"] == "startrec") {
					record_state.StartRecording(msg["file"].get<std::string>(), stream_config);
				}

				if (msg["val"] == "stoprec") {
					record_state.StopRecording();
				}

				if (msg["val"] == "startplay") {
					if (msg["slice"].get<bool>()) {
						TimeSpecification ts = REL;
						std::chrono::milliseconds tStart = std::chrono::milliseconds(msg["sliceStart"].get<long long>());
						std::chrono::milliseconds tEnd = std::chrono::milliseconds(msg["sliceEnd"].get<long long>()); ;
						if (record_state.StartReplaying(msg["file"]), tStart, tEnd, ts) {
							SendConfig(&(record_state.current_replay_file->config()));
						}
					} else {
						if (record_state.StartReplaying(msg["file"])) {
							SendConfig(&(record_state.current_replay_file->config()));
						}
					}
				}

				if (msg["val"] == "stopplay") {
					record_state.StopReplaying();
					SendConfig();
				}
			}


			if (msg["cmd"] == "extrinsics") {
				if (msg["val"] == "update") {
					stream_config.Px = msg["x"].get<float>();
					stream_config.Py = msg["y"].get<float>();
					stream_config.Pz = msg["z"].get<float>();
					stream_config.Qx = msg["qx"].get<float>();
					stream_config.Qy = msg["qy"].get<float>();
					stream_config.Qz = msg["qz"].get<float>();
					stream_config.Qw = msg["qw"].get<float>();
				}
			}
		}
	}


	void RGBDStreamer::Exit() {
		stream_shutdown = true;
		if (record_state.recording()) {
			record_state.StopRecording();
		}
		if (record_state.replaying()) {
			record_state.StopRecording();
		}
		CloseDevice();
		client->Close();
	}

	int RGBDStreamer::SendConfig() {
		return SendConfig(&stream_config);
	}

	int RGBDStreamer::SendConfig(CONFIG_STRUCT * config) {
		oi::core::network::DataContainer * dc;
		if (!client->GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}

		int data_len = sizeof(*config);
		memcpy(dc->dataBuffer, config, data_len);
		dc->_data_end = data_len;
		client->QueueForSending(&dc);
		return data_len;
	}

	int RGBDStreamer::_SendAudioFrame(unsigned int sequence, float * samples, size_t n_samples, unsigned short freq, unsigned short channels) {
		std::ostream * audio_writer = NULL;
		if (record_state.recording()) {
			audio_writer = record_state.audio_writer();
		}

		
		audioSamplesCounter += n_samples;

		oi::core::network::DataContainer * dc;
		if (!client->GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc->dataType = "_SendAudioFrame";

		audio_header.channels = channels;
		audio_header.frequency = freq;
		audio_header.samples = n_samples;
		audio_header.sequence = sequence;
		memcpy(&(dc->dataBuffer[0]), &audio_header, sizeof(audio_header));
		size_t writeOffset = sizeof(audio_header);
		unsigned int  audio_block_size = sizeof(float) * n_samples;

		memcpy(&(dc->dataBuffer[writeOffset]), samples, sizeof(float) * n_samples);
		writeOffset += audio_block_size;


		if (audio_writer != NULL) {
			unsigned char* audio_block = &(dc->dataBuffer[headerSize]);
			audio_writer->write((const char*)audio_block, audio_block_size);
		}

		/*
		for (int i = 0; i < n_samples; i++) {
			unsigned short sample = (unsigned short) (samples[i] * 32767);
			memcpy(&(dc->dataBuffer[writeOffset]), &sample, sizeof(sample));
			writeOffset += sizeof(sample);
		}*/

		size_t data_len = writeOffset;
		dc->_data_end = data_len;
		client->QueueForSending(&dc);
		return data_len;
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata) {
		return _SendRGBDFrame(sequence, rgbdata, depthdata, NULL);
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned short * depthdata) {
		return _SendRGBDFrame(sequence, rgbdata, NULL, depthdata);
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata_any, unsigned short * depthdata_ushort) {
		std::ostream * rgbd_writer = NULL;

		if (record_state.recording()) {
			rgbd_writer = record_state.rgbd_writer();
		}

		int res = 0;
		std::chrono::milliseconds now = NOW();
		std::chrono::milliseconds delta = now - _prev_frame;
		_prev_frame = now;

		unsigned short deltaValue = (unsigned short) delta.count();
		if (delta.count() >= 60000) deltaValue = 0; // Just to be sure that we don't overflow...

		rgbd_header.sequence = sequence;
		if (sequence > highest_sequence) highest_sequence = highest_sequence;
		else rgbd_header.sequence = ++highest_sequence;
		rgbd_header.delta_t = deltaValue;

		// Send color data first...
		rgbd_header.msgType = 0x04;
		rgbd_header.startRow = (unsigned short) 0;             // ... we can fit the whole...
		rgbd_header.endRow = (unsigned short) frame_height(); //...RGB data in one packet

		oi::core::network::DataContainer * dc_rgb;
		if (!client->GetFreeWriteContainer(&dc_rgb)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc_rgb->dataType = "_SendRGBDFrame (rgb)";

		memcpy(dc_rgb->dataBuffer, &rgbd_header, headerSize);

		// COMPRESS COLOR
		long unsigned int _jpegSize = dc_rgb->bufferSize - headerSize;
		unsigned char* _compressedImage = &(dc_rgb->dataBuffer[headerSize]);

		tjhandle _jpegCompressor = tjInitCompress();
		tjCompress2(_jpegCompressor, rgbdata, (int)frame_width(), 0, (int)frame_height(), color_pixel_format(),
			&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
			TJFLAG_FASTDCT);

		int c_data_len = headerSize + _jpegSize;
		dc_rgb->_data_end = c_data_len;

		if (rgbd_writer != NULL) {
			unsigned int jpeg_size = (unsigned int)_jpegSize;
			rgbd_writer->write((const char*)&deltaValue, sizeof(rgbd_header.delta_t)); // [delta_t (unsigned short)]
			rgbd_writer->write((const char*)&rgbd_header.startRow, sizeof(rgbd_header.startRow)); // [color_start_l (unsigned short)]
			rgbd_writer->write((const char*)&rgbd_header.endRow, sizeof(rgbd_header.endRow)); // [color_end_l (unsigned short)]
			rgbd_writer->write((const char*)&jpeg_size, sizeof(jpeg_size)); // [color_size  (unsigned int)]
			rgbd_writer->write((const char*)_compressedImage, jpeg_size); // [color_data ...]
		}

		if (!record_state.replaying()) {
			client->QueueForSending(&dc_rgb);
		} else {
			client->ReleaseForWriting(&dc_rgb);
		}

		res += c_data_len;
		tjDestroy(_jpegCompressor);

		// Send Depth
		rgbd_header.msgType = 0x03;

		if (rgbd_writer != NULL) {
			unsigned short npackets = (unsigned short)(frame_height() + linesPerMessage - 1) / linesPerMessage; // round up division
			rgbd_writer->write((const char*)&npackets, sizeof(npackets)); // [n_depth_packets(unsigned short)]
		}

		for (unsigned int startRow = 0; startRow < frame_height(); startRow += linesPerMessage) {
			size_t endRow = startRow + linesPerMessage;
			if (endRow >= frame_height()) endRow = frame_height();
			if (startRow >= endRow) break;

			rgbd_header.startRow = (unsigned short)startRow;
			rgbd_header.endRow = (unsigned short)endRow;

			oi::core::network::DataContainer * dc_depth;
			if (!client->GetFreeWriteContainer(&dc_depth)) {
				std::cout << "\nERROR: No free buffers available" << std::endl;
				return -1;
			}

			dc_depth->dataType = "_SendRGBDFrame (depth)";

			memcpy(dc_depth->dataBuffer, &rgbd_header, headerSize);
			size_t writeOffset = headerSize;

			if (depthdata_any != NULL) {
				size_t depthLineSizeR = frame_width() * raw_depth_stride();
				size_t depthLineSizeW = frame_width() * 2;
				size_t readOffset = startRow*depthLineSizeR;
				for (int line = startRow; line < endRow; line++) {
					for (int i = 0; i < frame_width(); i++) {
						float depthValue = 0;
						memcpy(&depthValue, &depthdata_any[readOffset + i * 4], sizeof(depthValue));
						unsigned short depthValueShort = (unsigned short)(depthValue);
						memcpy(&(dc_depth->dataBuffer[writeOffset + i * 2]), &depthValueShort, sizeof(depthValueShort));
					}
					writeOffset += depthLineSizeW;
					readOffset += depthLineSizeR;
				}
			} else if (depthdata_ushort != NULL) {
				size_t startRowStart = startRow * frame_width();
				// for pixel (in all lines), two bytes:
				size_t bytesToCopy = (endRow - startRow) * frame_width() * sizeof(depthdata_ushort[0]);
				memcpy(&(dc_depth->dataBuffer[writeOffset]), &(depthdata_ushort[startRowStart]), bytesToCopy);
				writeOffset += bytesToCopy;
			}

			int d_data_len = writeOffset;
			dc_depth->_data_end = d_data_len;

			if (rgbd_writer != NULL) {
				unsigned char* _depthBlock = &(dc_depth->dataBuffer[headerSize]);
				unsigned int depth_block_size = (unsigned int) (writeOffset-headerSize);
				rgbd_writer->write((const char*)&rgbd_header.startRow, sizeof(rgbd_header.startRow)); // [depth_start_l (unsigned short)]
				rgbd_writer->write((const char*)&rgbd_header.endRow, sizeof(rgbd_header.endRow)); // [depth_end_l (unsigned short)]
				rgbd_writer->write((const char*)&depth_block_size, sizeof(depth_block_size)); // [depth_size  (unsigned int)]
				rgbd_writer->write((const char*)_depthBlock, depth_block_size); // [depth_data ...]
			}

			if (!record_state.replaying()) {
				client->QueueForSending(&dc_depth);
			} else {
				client->ReleaseForWriting(&dc_depth);
			}
			res += d_data_len;
		}

		return res;
	}

	int RGBDStreamer::ReplayNextFrame() {
		std::ifstream * rgbd_reader = record_state.rgbd_reader();
		int res = 0;

		unsigned short delta = 0;
		unsigned short start_l = 0;
		unsigned short end_l = 0;
		unsigned short npackets = 0;
		unsigned int rgbLength = 0;
		unsigned int depthLength = 0;
		highest_sequence++;

		// RGB
		rgbd_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
		rgbd_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
		rgbd_reader->read(reinterpret_cast<char *>(&rgbLength), sizeof(rgbLength));
		// HEADER
		record_state.replay_rgbd_header.sequence = highest_sequence;
		record_state.replay_rgbd_header.delta_t = delta;
		record_state.replay_rgbd_header.startRow = start_l;
		record_state.replay_rgbd_header.endRow = end_l;
		record_state.replay_rgbd_header.deviceID = record_state.current_replay_file->config().deviceID;
		record_state.replay_rgbd_header.msgType = 0x04;

		oi::core::network::DataContainer * dc_rgb;
		if (!client->GetFreeWriteContainer(&dc_rgb)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc_rgb->dataType = "Replay (rgb)";
		
		memcpy(dc_rgb->dataBuffer, &record_state.replay_rgbd_header, headerSize);
		rgbd_reader->read(reinterpret_cast<char *>(&(dc_rgb->dataBuffer[headerSize])), rgbLength);
		dc_rgb->_data_end = headerSize + rgbLength;
		res += dc_rgb->_data_end;
		client->QueueForSending(&dc_rgb);

		// DEPTH
		rgbd_reader->read(reinterpret_cast<char *>(&npackets), sizeof(npackets));
		record_state.replay_rgbd_header.msgType = 0x03;

		for (unsigned int i = 0; i < npackets; i++) {
			rgbd_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
			rgbd_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
			rgbd_reader->read(reinterpret_cast<char *>(&depthLength), sizeof(depthLength));
			record_state.replay_rgbd_header.startRow = start_l;
			record_state.replay_rgbd_header.endRow = end_l;

			oi::core::network::DataContainer * dc_depth;
			if (!client->GetFreeWriteContainer(&dc_depth)) {
				std::cout << "\nERROR: No free buffers available" << std::endl;
				return -1;
			}

			dc_depth->dataType = "Replay (depth)";

			memcpy(dc_depth->dataBuffer, &record_state.replay_rgbd_header, headerSize);
			rgbd_reader->read(reinterpret_cast<char *>(&(dc_depth->dataBuffer[headerSize])), depthLength);
			dc_depth->_data_end = headerSize + depthLength;
			res += dc_depth->_data_end;
			client->QueueForSending(&dc_depth);
		}

		rgbd_reader->read(reinterpret_cast<char *>(&delta), sizeof(delta));

		std::chrono::milliseconds now = NOW();
		std::ifstream * audio_reader = record_state.audio_reader();

		// AUDIO
		if (audio_reader->is_open()) {
			std::chrono::milliseconds buffer_time = std::chrono::milliseconds(1500);
			std::chrono::milliseconds skip_time = std::chrono::milliseconds(500);
			std::chrono::milliseconds replay_time = now - (record_state._replay_start_time);
			while (true) {
				unsigned long pos = audio_reader->tellg();
				if (pos == 0) {
					std::streamsize skip_bytes = skip_time.count() * 16 * sizeof(float);
					audio_reader->ignore(skip_bytes);
					pos = audio_reader->tellg();
					std::cout << "skipped some: " << skip_bytes << std::endl;
				}
				// At 16kHz, we have 16 samples per millisecond, each sample being 4 bytes (float) 
				std::chrono::milliseconds replay_sent = std::chrono::milliseconds(pos / (16 * sizeof(float)));
				std::chrono::milliseconds buffer_catch_up = (replay_time - replay_sent) + buffer_time;
				
				if (buffer_catch_up.count() <= 30) break; // don't send too tiny packets
				size_t n_samples = std::min((int)(buffer_catch_up.count()*16), 1000); // Maximum 1000 samples


				oi::core::network::DataContainer * dc_audio;
				if (!client->GetFreeWriteContainer(&dc_audio)) {
					std::cout << "\nERROR: No free buffers available" << std::endl;
					return -1;
				}

				dc_audio->dataType = "Replay (audio)";

				record_state.replay_audio_header.sequence = highest_sequence++;
				record_state.replay_audio_header.frequency = 16000;
				record_state.replay_audio_header.channels = 1;
				record_state.replay_audio_header.samples = n_samples;
				audioSamplesCounter += n_samples;
				// WRITE HEADER
				memcpy(&(dc_audio->dataBuffer[0]), &record_state.replay_audio_header, sizeof(record_state.replay_audio_header));
				size_t writeOffset = sizeof(record_state.replay_audio_header);
				unsigned int  audio_block_size = sizeof(float) * n_samples;
				// WRITE AUDIO SAMPLES
				audio_reader->read(reinterpret_cast<char *>(&(dc_audio->dataBuffer[writeOffset])), audio_block_size);
				writeOffset += audio_block_size;
				dc_audio->_data_end = writeOffset;
				res += dc_audio->_data_end;
				client->QueueForSending(&dc_audio);
			}

		}

		// denotes end of file
		if (delta >= 60000) {
			std::cout << "\n CLEAN END " << std::endl;
			if (record_state.loop) {
				rgbd_reader->clear();
				rgbd_reader->seekg(0, std::ios::beg);
				if (audio_reader->is_open()) {
					audio_reader->clear();
					audio_reader->seekg(0, std::ios::beg);
				}
				unsigned short first_delta = 0;
				rgbd_reader->read(reinterpret_cast<char *>(&delta), sizeof(delta));
				record_state._replay_start_time = oi::core::rgbd::NOW();
			} else {
				record_state.StopReplaying();
				SendConfig();
			}
		}
		record_state._replay_next_frame = now + std::chrono::milliseconds(delta);

		return res;
	}

	void RGBDStreamer::HandleData() {
		oi::core::network::DataContainer * dc;
		if (!client->DequeueForReading(&dc)) return;

		std::string raw((char *) &(dc->dataBuffer[dc->data_start()]), dc->data_end() - dc->data_start());
		std::cout << "\nin: |" << raw << "|" << std::endl;

		if (!HandleData(dc)) {


			nlohmann::json msg = nlohmann::json::parse(
				&(dc->dataBuffer[dc->data_start()]),
				&(dc->dataBuffer[dc->data_end()]));
			if (msg.find("cmd") != msg.end() && msg["cmd"].is_string()) {
				ScheduledCommand sc;
				sc.cmd = msg;

				if (msg.find("time") != msg.end() && msg["time"].is_number()) {
					sc.time = std::chrono::milliseconds(msg["time"].get<long long>());
					std::cout << "\nScheduled message in milliseconds: " << (sc.time - NOW()).count() << std::endl;
				} else {
					sc.time = NOW();
				}

				scheduled_commands.push(sc);
			} else {
				std::cerr << "\nERROR: Unknown JSON message:\n" << msg.dump(2) << std::endl;
			}
		}

		client->ReleaseForReceiving(&dc);
	}


	void RGBDStreamer::ConfigStream() {
		linesPerMessage = (unsigned int)(MAX_UDP_PACKET_SIZE - headerSize) / (2 * frame_width());

		stream_config.frameWidth = (unsigned short) frame_width();
		stream_config.frameHeight = (unsigned short) frame_height();
		stream_config.maxLines = (unsigned short) linesPerMessage;
		stream_config.Cx = device_cx();
		stream_config.Cy = device_cy();
		stream_config.Fx = device_fx();
		stream_config.Fy = device_fy();
		stream_config.DepthScale = device_depth_scale();
		stream_config.dataFlags |= RGBD_DATA;
		if (supports_audio()) {
			stream_config.dataFlags |= AUDIO_DATA;
		}

		stream_config.dataFlags |= LIVE_DATA;
		std::string guid = device_guid();
        memcpy(&(stream_config.guid[0]), guid.c_str(), guid.length()+1); // +1 for null terminator

		config_msg_buf = new unsigned char[sizeof(stream_config)];
	}

	void StreamerConfig::Parse(int argc, char *argv[]) {

		std::string useMMParam("-mm");
		std::string socketIDParam("-id");
		std::string listenPortParam("-lp");
		std::string remotePortParam("-rp");
		std::string remoteHostParam("-rh");
		std::string serialParam("-sn");
		std::string pipelineParam("-pp");
		std::string maxDepthParam("-md");

		std::string fileSaveParam("-o");
		std::string fileLoadParam("-i");
		std::string filePathParam("-d");

		for (int count = 1; count < argc; count += 2) {
			if (socketIDParam.compare(argv[count]) == 0) {
				this->socketID = argv[count + 1];
			} else if (remoteHostParam.compare(argv[count]) == 0) {
				this->remoteHost = argv[count + 1];
			} else if (remotePortParam.compare(argv[count]) == 0) {
				this->remotePort = std::stoi(argv[count + 1]);
			} else if (listenPortParam.compare(argv[count]) == 0) {
				this->listenPort = std::stoi(argv[count + 1]);
			} else if (serialParam.compare(argv[count]) == 0) {
				this->deviceSerial = argv[count + 1];
			} else if (pipelineParam.compare(argv[count]) == 0) {
				this->pipeline = argv[count + 1];
			} else if (filePathParam.compare(argv[count]) == 0) {
				this->filePath = argv[count + 1];
			} else if (maxDepthParam.compare(argv[count]) == 0) {
				this->maxDepth = std::stof(argv[count + 1]);
			} else if (fileSaveParam.compare(argv[count]) == 0) {
				this->fileSave = argv[count + 1];
			} else if (fileLoadParam.compare(argv[count]) == 0) {
				this->fileLoad = argv[count + 1];
			} else if (useMMParam.compare(argv[count]) == 0) {
				this->useMatchMaking = std::stoi(argv[count + 1])==1;
			} else {
				std::cout << "Unknown Parameter: " << argv[count] << std::endl;
			}
		}

	}


	RecordState::RecordState() {
		_recording = false;
		_replaying = false;
		loop = true;
	}

	std::ofstream * RecordState::rgbd_writer() {
		return &_out_rgbd;
	}

	std::ofstream * RecordState::audio_writer() {
		return &_out_audio;
	}

	std::ofstream * RecordState::meta_writer() {
		return &_out_meta;
	}

	std::ifstream * RecordState::rgbd_reader() {
		return &_in_rgbd;
	}

	std::ifstream * RecordState::audio_reader() {
		return &_in_audio;
	}

	bool RecordState::recording() {
		return _recording;
	}

	bool RecordState::replaying() {
		return _replaying;
	}

	unsigned int RecordState::next_rec_frame_nr() {
		unsigned int res = _current_frame;
		_current_frame++;
		return res;
	}

	CONFIG_STRUCT * RecordState::replay_config() {
		return &_replay_config;
	}

	bool RecordState::LoadMeta(std::string name) {
		std::string filename_meta = PATH + name + META_SUFFIX;
		std::ifstream in_meta;
		in_meta.open(filename_meta, std::ios::binary | std::ios::in);
		if (in_meta.fail()) return false;
		FileMeta fm;
		fm.Load(name, &in_meta);
		files_meta[name] = fm;
		in_meta.close();
		in_meta.clear();
		return true;
	}

	bool RecordState::StartRecording(std::string name, CONFIG_STRUCT config) {
		if (_replaying) {
		}

		if (_recording) {
			std::cerr << "\nCannot start recording with name " << name << ". Already recording." << std::endl;
			return false;
		}
		if (files_meta.find(name) != files_meta.end()) {
			if (name == "default") {
				std::cout << "\nWARNING: overwriting default file." << std::endl;
				if (current_replay_file != NULL && current_replay_file->name() == name) {
					StopReplaying();
				}
				files_meta.erase(name);
			} else {
				std::cerr << "\nCannot start recording. Name " << name << " already exists." << std::endl;
				return false;
			}
		}

		std::string filename_audio = PATH + name + AUDIO_SUFFIX;
		std::string filename_rgbd  = PATH + name + RGBD_SUFFIX;
		std::string filename_meta  = PATH + name + META_SUFFIX;
		_current_frame = 0;
		_out_rgbd.open(filename_rgbd, std::ios::binary | std::ios::out | std::ios::trunc);
		_out_meta.open(filename_meta, std::ios::binary | std::ios::out | std::ios::trunc);
		if (config.dataFlags & AUDIO_DATA) {
			_out_audio.open(filename_audio, std::ios::binary | std::ios::out | std::ios::trunc);
		}
		unsigned short version = 1; // TODO: make this global
		_out_meta.write((const char*) &version, sizeof(version));

		config.dataFlags &= ~LIVE_DATA; // Lazy, should make a copy?
		_out_meta.write((const char*) &config,  sizeof(config));
		config.dataFlags |= LIVE_DATA;

		_last_rgbd_pos = 0;
		_current_recording_name = name;
		_recording = true;
		std::cout << "\nINFO: Started Recording " << name << std::endl;
		return true;
	}

	bool RecordState::StopRecording() {
		if (!_recording) return false;
		bool res = true;
		unsigned short last_delta = 60000;
		_out_rgbd.write((const char*)&last_delta, sizeof(last_delta)); // [memory_pos (unsigned long=8bytes)]
		if (_out_audio.is_open()) {
			std::cout << "Closed audio file." << std::endl;
			_out_audio.close();
			_out_audio.clear();
		}

		_out_rgbd.close();
		_out_rgbd.clear();
		_out_meta.close();
		_out_meta.clear();
		_recording = false;
		if (!LoadMeta(_current_recording_name)) {
			std::cerr << "Stopped recording but failed to load Meta" << std::endl;
			res = false;
		} else {
			std::cout << "\nINFO: Stopped Recording " << _current_recording_name << std::endl;
		}
		_current_recording_name = "";
		return res;
	}

	bool RecordState::StartReplaying(std::string name) {
		if (_recording && name == _current_recording_name) {
			std::cerr << "\nERROR: cannot replay file while recording to it." << std::endl;
			return false;
		}

		if (_replaying) {
			StopReplaying();
		}

		std::string filename_rgbd = PATH + name + RGBD_SUFFIX;
		std::string filename_audio = PATH + name + AUDIO_SUFFIX;

		_in_rgbd.open(filename_rgbd, std::ios::binary | std::ios::in);
		if (files_meta.find(name) == files_meta.end() && (!LoadMeta(name) || _in_rgbd.fail())) {
			std::cout << "\nERROR: Failed to open \"" << name << "\" for replaying" << std::endl;
		}


		_in_audio.open(filename_audio, std::ios::binary | std::ios::in);
		if (_in_audio.fail()) {
			std::cout << "\nNo audio file found. Won't replay any audio." << std::endl;
		}

		unsigned short first_delta = 0;
		_in_rgbd.read(reinterpret_cast<char *>(&first_delta), sizeof(first_delta));
		_replay_start_time = oi::core::rgbd::NOW();
		_replay_next_frame = _replay_start_time + std::chrono::milliseconds(first_delta);

		_replay_config = files_meta[name].config();
		current_replay_file = &files_meta[name];
		_replaying = true;
		return true;
	}

	bool RecordState::StartReplaying(std::string name, std::chrono::milliseconds startSlice, std::chrono::milliseconds endSlice, TimeSpecification ts) {
		return false;
	}

	bool RecordState::StopReplaying() {
		if (!_replaying) return false;
		_in_rgbd.close();
		_in_rgbd.clear();
		_in_audio.close();
		_in_audio.clear();
		current_replay_file = NULL;
		_replaying = false;
	}

	CONFIG_STRUCT FileMeta::config() {
		return _config;
	}

	void rgbd::FileMeta::Load(std::string name, std::ifstream * in_meta) {
		this->_name = name;
		in_meta->read(reinterpret_cast<char *>(&_recording_version), sizeof(_recording_version));
		in_meta->read(reinterpret_cast<char *>(&_config), sizeof(_config));
		while (true) {
			META_STRUCT frame_meta;
			in_meta->read(reinterpret_cast<char *>(&frame_meta), sizeof(frame_meta));
			in_meta->ignore(frame_meta.payload_size);
			if (in_meta->eof()) break;
			_frames.push_back(frame_meta);
		}
	}

	std::string FileMeta::name() {
		return _name;
	}

	bool rgbd::ScheduledCommand::operator()(ScheduledCommand left, ScheduledCommand right) {
		return left.time > right.time;
	}

} } }
