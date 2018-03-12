#include "RGBDStreamer.hpp"
#include "RecordState.hpp"
#include <string>
#include <algorithm> 
#include "..\headers\RGBDStreamer.hpp"

namespace oi { namespace core { namespace rgbd {

	std::chrono::milliseconds NOW() {
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
		_is_open = false;
		/* This tests that the structs used to (de-)serialize data for sending and file i/o have the expected
		* number of bytes allocated in memory. This may be different per compiler, but we explicitly pad those
		* array with dummy properties, so there is no reason for compilers to re-align.
		*  TODO: Make tests and move this there */
		std::cout << "BODY_HEADER_STRUCT 16==" << sizeof(oi::core::rgbd::BODY_HEADER_STRUCT) << std::endl;
		std::cout << "BODY_STRUCT 344==" << sizeof(oi::core::rgbd::BODY_STRUCT) << std::endl;
		std::cout << "META_STRUCT 56==" << sizeof(oi::core::rgbd::META_STRUCT) << std::endl;
		std::cout << "RGBD_HEADER_STRUCT 16==" << sizeof(oi::core::rgbd::RGBD_HEADER_STRUCT) << std::endl;
		std::cout << "AUDIO_HEADER_STRUCT 16==" << sizeof(oi::core::rgbd::AUDIO_HEADER_STRUCT) << std::endl;
		std::cout << "CONFIG_STRUCT 132==" << sizeof(oi::core::rgbd::CONFIG_STRUCT) << std::endl;
	}

	void RGBDStreamer::Run() {
		if (OpenDevice()) { // Add option to not start a live stream.
			std::cout << "Opened device for live streaming." << std::endl;
			ConfigStream();
			SendConfig();
			_is_open = true;
		} else {
			std::cout << "Did not open device for live streaming." << std::endl;
		}

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
				std::ostream * body_writer = record_state.body_writer();
				std::ostream * mjpg_writer = record_state.mjpg_writer();
				std::ostream * bidx_writer = record_state.bidx_writer();

				if (rgbd_writer == NULL || (unsigned long long) rgbd_writer->tellp() == record_state._last_rgbd_pos) {
					// no new depth frame, don't update meta.
				} else {
					META_STRUCT frame_meta;
					frame_meta.timestamp = NOW();
					std::chrono::milliseconds tRelative = frame_meta.timestamp - record_state._current_recording_start_time;

					frame_meta.memory_pos_rgbd = rgbd_writer->tellp();
					record_state._last_rgbd_pos = frame_meta.memory_pos_rgbd;

					if (supports_body() && body_writer != NULL)
						frame_meta.memory_pos_body = body_writer->tellp();
					else frame_meta.memory_pos_body = 0;

					if (supports_audio() && audio_writer != NULL) {
						// How many milliseconds of data is this?
						// memory_pos_audio is in bytes, so we have samples = 
						std::chrono::milliseconds tAudio((frame_meta.memory_pos_audio / sizeof(float)) / 16);
						if (tAudio < tRelative) {
							std::chrono::milliseconds timeBehind = tRelative - tAudio;
							if (timeBehind.count() > 1000) {
								std::cout << "\nAUDIO TIME BEHIND=" << timeBehind.count() << std::endl;
								unsigned int compensateMissing = timeBehind.count() * 16;
								float zeroSample = 0.0f;
								// we need 16 samples for a millisecond
								for (unsigned int i = 0; i < compensateMissing; i++) {
									audio_writer->write((const char*) &zeroSample, sizeof(float));
								}
							}
						}

						frame_meta.memory_pos_audio = audio_writer->tellp();

					} else frame_meta.memory_pos_audio = 0;

					if (supports_hd() && mjpg_writer != NULL)
						frame_meta.memory_pos_hd = mjpg_writer->tellp();
					else frame_meta.memory_pos_hd = 0;
					if (supports_bidx() && bidx_writer != NULL)
						frame_meta.memory_pos_bidx = bidx_writer->tellp();
					else frame_meta.memory_pos_bidx = 0;

					frame_meta.frame_nr = record_state.next_rec_frame_nr();
					frame_meta.payload_size = 0;
					meta_writer->write((const char*)&frame_meta, sizeof(frame_meta));
				}

				if (_is_open) sentBytes += SendFrame(); // SendFrame implementations call _SendDepthFrame & _SendAudioFrame
			}

			if (record_state.replaying()) {// && NOW() >= record_state._replay_next_frame
				sentBytes += ReplayNextFrame();
			}

			if (sentBytes > 0) {
				bytesCounter += sentBytes;
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
			}

		}
	}

	void RGBDStreamer::HandleCommands() {
		if (scheduled_commands.size() < 1) return;
		std::chrono::milliseconds t = NOW();
		if (scheduled_commands.top().time <= t) {
			nlohmann::json msg = scheduled_commands.top().cmd;
			scheduled_commands.pop();
			if (msg["cmd"] == "application") {
				if (msg["val"] == "stop") {
					stream_shutdown = true;
				}

				if (msg["val"] == "disable_device") {
					if (_is_open) {
						if (record_state.recording()) {
							record_state.StopRecording();
						}
						_is_open = !CloseDevice();
						if (_is_open) {
							std::cerr << "Failed to close device." << std::endl;
						}
					} else {
						std::cerr << "Device already closed." << std::endl;
					}
				}

				if (msg["val"] == "enable_device") {
					if (!_is_open) {
						_is_open = OpenDevice();
						if (!_is_open) {
							std::cerr << "Failed to open device." << std::endl;
						}
					} else {
						std::cerr << "Device already open." << std::endl;
					}

				}

				if (msg["val"] == "requestconfig") {
					if (record_state.replaying()) {
						SendConfig(&(record_state._current_replay_file->config()));
					} else {
						SendConfig();
					}
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
					bool _slice = msg["slice"].get<bool>();
					bool _loop = msg["loop"].get<bool>();
					if (_slice) {
						TimeSpecification ts = T_REL;
						std::chrono::milliseconds tStart = std::chrono::milliseconds(msg["sliceStart"].get<long long>());
						std::chrono::milliseconds tEnd = std::chrono::milliseconds(msg["sliceEnd"].get<long long>()); ;
						if (record_state.StartReplaying(msg["file"], tStart, tEnd, ts)) {
							record_state.loop = _loop;
							SendConfig(&(record_state._current_replay_file->config()));
						}
					} else {
						if (record_state.StartReplaying(msg["file"])) {
							record_state.loop = _loop;
							SendConfig(&(record_state._current_replay_file->config()));
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

	int RGBDStreamer::_SendAudioFrame(unsigned int sequence, float * samples, size_t n_samples, unsigned short freq, unsigned short channels, std::chrono::milliseconds timestamp) {
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

		static AUDIO_HEADER_STRUCT audio_header;
		static size_t header_size = sizeof(AUDIO_HEADER_STRUCT);

		audio_header.channels = channels;
		audio_header.frequency = freq;
		audio_header.samples = n_samples;
		audio_header.timestamp = timestamp.count();
		memcpy(&(dc->dataBuffer[0]), &audio_header, sizeof(audio_header));
		size_t writeOffset = sizeof(audio_header);
		unsigned int  audio_block_size = sizeof(float) * n_samples;

		memcpy(&(dc->dataBuffer[writeOffset]), samples, sizeof(float) * n_samples);
		writeOffset += audio_block_size;


		if (audio_writer != NULL) {
			unsigned char* audio_block = &(dc->dataBuffer[header_size]);
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

	int RGBDStreamer::_SendBodyFrame(BODY_STRUCT * bodies, unsigned short n_bodies, std::chrono::milliseconds timestamp) {
		int res = 0;
		std::ostream * body_writer = NULL;
		if (record_state.recording()) {
			body_writer = record_state.body_writer();
		}

		BODY_HEADER_STRUCT body_header;
		body_header.timestamp = timestamp.count();
		body_header.n_bodies = n_bodies;

		oi::core::network::DataContainer * dc;
		if (!client->GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc->dataType = "_SendBodyFrame";

		size_t header_size = sizeof(BODY_HEADER_STRUCT);
		size_t data_size   = sizeof(BODY_STRUCT) * n_bodies;

		memcpy(dc->dataBuffer, &body_header, header_size);
		memcpy(&(dc->dataBuffer[header_size]), bodies, data_size);

		dc->_data_end = header_size + data_size;

		if (body_writer != NULL) {
			body_writer->write((const char*)&n_bodies, sizeof(n_bodies));
			if (n_bodies > 0) {
				body_writer->write((const char*)bodies, data_size);
			}
		}

		if (!record_state.replaying() && n_bodies > 0) {
			client->QueueForSending(&dc);
		} else {
			client->ReleaseForWriting(&dc);
		}

		res += header_size + data_size;

		return res;
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata, std::chrono::milliseconds timestamp) {
		return _SendRGBDFrame(sequence, rgbdata, depthdata, NULL, timestamp);
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned short * depthdata, std::chrono::milliseconds timestamp) {
		return _SendRGBDFrame(sequence, rgbdata, NULL, depthdata, timestamp);
	}

	int RGBDStreamer::_SendRGBDFrame(unsigned long sequence, unsigned char * rgbdata, unsigned char * depthdata_any, unsigned short * depthdata_ushort, std::chrono::milliseconds timestamp) {
		std::ostream * rgbd_writer = NULL;

		if (record_state.recording()) {
			rgbd_writer = record_state.rgbd_writer();
		}

		int res = 0;
		std::chrono::milliseconds delta = timestamp - _prev_frame;
		_prev_frame = timestamp;

		unsigned short deltaValue = (unsigned short) delta.count();
		if (delta.count() >= 60000) deltaValue = 0; // Just to be sure that we don't overflow...


		static RGBD_HEADER_STRUCT rgbd_header;
		static size_t headerSize = sizeof(RGBD_HEADER_STRUCT);
		rgbd_header.timestamp = timestamp.count();
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

		fpsCounter++;
		return res;
	}


	int RGBDStreamer::_SendBodyIndexFrame(unsigned char * bidata, int width, int height, TJPF pix_fmt, std::chrono::milliseconds timestamp) {
		// This doesn't really send, only record for now...
		std::ostream * bidx_writer = NULL;
		int res = 0;

		if (record_state.recording()) {
			bidx_writer = record_state.bidx_writer();
		}

		static RGBD_HEADER_STRUCT rgbd_header;
		static size_t header_size = sizeof(RGBD_HEADER_STRUCT);
		rgbd_header.timestamp = timestamp.count();
		rgbd_header.delta_t = 0;
		rgbd_header.msgType = 0x33;
		rgbd_header.startRow = (unsigned short)0;
		rgbd_header.endRow = (unsigned short)frame_height(); 

		oi::core::network::DataContainer * dc;
		if (!client->GetFreeWriteContainer(&dc)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc->dataType = "_SendBodyIndexFrame";

		memcpy(dc->dataBuffer, &rgbd_header, header_size);

		// COMPRESS COLOR
		long unsigned int _jpegSize = dc->bufferSize - header_size;
		unsigned char* _compressedImage = &(dc->dataBuffer[header_size]);

		tjhandle _jpegCompressor = tjInitCompress();
		tjCompress2(_jpegCompressor, bidata, (int)width, 0, (int)height, pix_fmt,
			&_compressedImage, &_jpegSize, TJSAMP_GRAY, 75,
			TJFLAG_FASTDCT);

		if (bidx_writer != NULL) {
			unsigned int jpeg_size = (unsigned int)_jpegSize;
			bidx_writer->write((const char*)&jpeg_size, sizeof(jpeg_size));
			bidx_writer->write((const char*)_compressedImage, jpeg_size);
		}

		int c_data_len = header_size + _jpegSize;
		dc->_data_end = c_data_len;

		if (!record_state.replaying()) {
			client->QueueForSending(&dc);
		} else {
			client->ReleaseForWriting(&dc);
		}

		res += c_data_len;
		tjDestroy(_jpegCompressor);

		return res;
	}

	int RGBDStreamer::_SendHDFrame(unsigned char * rgbdata, int width, int height, TJPF pix_fmt, std::chrono::milliseconds timestamp) {
		// This doesn't really send, only record for now...
		const unsigned long max_hd_buffer = 1920 * 1080 * 2;
		static unsigned char hd_buffer[max_hd_buffer];
		std::ostream * mjpg_writer = record_state.mjpg_writer();
		if (!record_state.recording() || mjpg_writer == NULL) return 0;
		int res = 0;
		unsigned long _jpegSize = max_hd_buffer;
		unsigned char* _compressedImage = &(hd_buffer[0]);

		tjhandle _jpegCompressor = tjInitCompress();
		tjCompress2(_jpegCompressor, rgbdata, (int)width, 0, (int)height, pix_fmt,
			&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
			TJFLAG_FASTDCT);

		if (mjpg_writer != NULL) {
			unsigned int jpeg_size = (unsigned int) _jpegSize;
			mjpg_writer->write((const char*)&jpeg_size, sizeof(jpeg_size));
			mjpg_writer->write((const char*)_compressedImage, jpeg_size);
		}

		res += _jpegSize;
		tjDestroy(_jpegCompressor);


		//if (record_state.avencode != NULL) record_state.avencode->AddFrame(rgbdata);

		return res;
	}

	int RGBDStreamer::ReplayNextFrame() {
		// Can allways send audio:
		int res = 0;
		std::chrono::milliseconds timestamp = NOW();

		// AUDIO
		std::istream * audio_reader = record_state.audio_reader();
		if (audio_reader != NULL) {
			std::chrono::milliseconds buffer_time = std::chrono::milliseconds(5000);
			//std::chrono::milliseconds skip_time = std::chrono::milliseconds(500);
			std::chrono::milliseconds replay_time = timestamp - (record_state._replay_start_time);
			while (true) {
				unsigned long long pos = audio_reader->tellg();
				/*
				if (false && pos == record_state._pos_start_audio) {
					std::streamsize skip_bytes = skip_time.count() * 16 * sizeof(float);
					audio_reader->seekg(skip_bytes, std::ios::cur);
					pos = audio_reader->tellg();
				}*/

				if (pos >= record_state._pos_end_audio) {
					break;
				}

				// At 16kHz, we have 16 samples per millisecond, each sample being 4 bytes (float) 
				std::chrono::milliseconds replay_sent = std::chrono::milliseconds((pos - record_state._pos_start_audio) / (16 * sizeof(float)));
				std::chrono::milliseconds buffer_catch_up = (replay_time - replay_sent) + buffer_time;
				if (buffer_catch_up.count() < 500) break; // don't send too tiny packets
				size_t n_samples = std::min((int)(buffer_catch_up.count() * 16), 1000); // but also not too big :) 1000*16*4 = 64000

				oi::core::network::DataContainer * dc_audio;
				if (!client->GetFreeWriteContainer(&dc_audio)) {
					std::cout << "\nERROR: No free buffers available" << std::endl;
					return -1;
				}

				dc_audio->dataType = "Replay (audio)";
				static AUDIO_HEADER_STRUCT replay_audio_header;
				static size_t header_size = sizeof(AUDIO_HEADER_STRUCT);
				replay_audio_header.timestamp = timestamp.count();
				replay_audio_header.frequency = 16000;
				replay_audio_header.channels = 1;
				replay_audio_header.samples = n_samples;
				audioSamplesCounter += n_samples;
				// WRITE HEADER
				memcpy(&(dc_audio->dataBuffer[0]), &replay_audio_header, header_size);
				size_t writeOffset = header_size;
				unsigned int  audio_block_size = sizeof(float) * n_samples;
				// WRITE AUDIO SAMPLES
				audio_reader->read(reinterpret_cast<char *>(&(dc_audio->dataBuffer[writeOffset])), audio_block_size);
				writeOffset += audio_block_size;
				dc_audio->_data_end = writeOffset;
				res += dc_audio->_data_end;
				client->QueueForSending(&dc_audio);
			}

		}



		if (timestamp < record_state._replay_next_frame) return res;






		std::istream * rgbd_reader = record_state.rgbd_reader();

		unsigned short delta = 0;
		unsigned short start_l = 0;
		unsigned short end_l = 0;
		unsigned short npackets = 0;
		unsigned int rgbLength = 0;
		unsigned int depthLength = 0;
		highest_sequence++;


		//rgbd_reader->read(reinterpret_cast<char *>(&delta), sizeof(delta));

		// RGB
		rgbd_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
		rgbd_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
		rgbd_reader->read(reinterpret_cast<char *>(&rgbLength), sizeof(rgbLength));
		// HEADER

		static RGBD_HEADER_STRUCT replay_rgbd_header;
		static size_t header_size = sizeof(RGBD_HEADER_STRUCT);

		replay_rgbd_header.timestamp = timestamp.count();
		replay_rgbd_header.delta_t = delta;
		replay_rgbd_header.startRow = start_l;
		replay_rgbd_header.endRow = end_l;
		replay_rgbd_header.deviceID = record_state._current_replay_file->config().deviceID;
		replay_rgbd_header.msgType = 0x04;

		oi::core::network::DataContainer * dc_rgb;
		if (!client->GetFreeWriteContainer(&dc_rgb)) {
			std::cout << "\nERROR: No free buffers available" << std::endl;
			return -1;
		}
		dc_rgb->dataType = "Replay (rgb)";
		
		memcpy(dc_rgb->dataBuffer, &replay_rgbd_header, header_size);
		rgbd_reader->read(reinterpret_cast<char *>(&(dc_rgb->dataBuffer[header_size])), rgbLength);
		dc_rgb->_data_end = header_size + rgbLength;
		res += dc_rgb->_data_end;
		client->QueueForSending(&dc_rgb);

		// DEPTH
		rgbd_reader->read(reinterpret_cast<char *>(&npackets), sizeof(npackets));
		replay_rgbd_header.msgType = 0x03;

		for (unsigned int i = 0; i < npackets; i++) {
			rgbd_reader->read(reinterpret_cast<char *>(&start_l), sizeof(start_l));
			rgbd_reader->read(reinterpret_cast<char *>(&end_l), sizeof(end_l));
			rgbd_reader->read(reinterpret_cast<char *>(&depthLength), sizeof(depthLength));
			replay_rgbd_header.startRow = start_l;
			replay_rgbd_header.endRow = end_l;

			oi::core::network::DataContainer * dc_depth;
			if (!client->GetFreeWriteContainer(&dc_depth)) {
				std::cout << "\nERROR: No free buffers available" << std::endl;
				return -1;
			}

			dc_depth->dataType = "Replay (depth)";

			memcpy(dc_depth->dataBuffer, &replay_rgbd_header, header_size);
			rgbd_reader->read(reinterpret_cast<char *>(&(dc_depth->dataBuffer[header_size])), depthLength);
			dc_depth->_data_end = header_size + depthLength;
			res += dc_depth->_data_end;
			client->QueueForSending(&dc_depth);
		}

		std::istream * body_reader = record_state.body_reader();

		if (body_reader != NULL) {
			static BODY_HEADER_STRUCT replay_body_header;
			body_reader->read(reinterpret_cast<char *>(&replay_body_header.n_bodies), sizeof(replay_body_header.n_bodies));
			
			replay_body_header.timestamp = timestamp.count();
			size_t header_size = sizeof(replay_body_header);
			size_t data_size = replay_body_header.n_bodies * sizeof(BODY_STRUCT);

			if (data_size > 0) {
				oi::core::network::DataContainer * dc;
				if (!client->GetFreeWriteContainer(&dc)) {
					std::cout << "\nERROR: No free buffers available" << std::endl;
					return -1;
				}
				dc->dataType = "Replay (body)";

				memcpy(&(dc->dataBuffer[0]), &replay_body_header, header_size);
				body_reader->read(reinterpret_cast<char *>(&(dc->dataBuffer[header_size])), data_size);

				dc->_data_end = data_size + header_size;
				res += dc->_data_end;
				client->QueueForSending(&dc);
			}
		}

		fpsCounter++;

		// denotes end of file
		record_state._next_replay_frame++;

		if (rgbd_reader->tellg() >= record_state._pos_end_rgbd ||
			record_state._next_replay_frame >= record_state._current_replay_file->frame_count()) {
			std::cout << "\n <EOF> " << std::endl;
			if (record_state.loop) {
				record_state.ReplayReset();
				SendConfig(&(record_state._current_replay_file->config()));
			} else {
				record_state.StopReplaying();
				SendConfig();
			}
		} else {
			record_state._replay_next_frame = record_state._replay_start_time + 
				(record_state._current_replay_file->frameTime(record_state._next_replay_frame)- record_state._current_replay_file->frameTime(record_state._frame_start_rgbd));
		}


		return res;
	}

	void RGBDStreamer::HandleData() {
		oi::core::network::DataContainer * dc;
		if (!client->DequeueForReading(&dc)) return;

		std::string raw((char *) &(dc->dataBuffer[dc->data_start()]), dc->data_end() - dc->data_start());
		//std::cout << "\nin: |" << raw << "|" << std::endl;

		if (!HandleData(dc)) {
			try {
				nlohmann::json msg = nlohmann::json::parse(
					&(dc->dataBuffer[dc->data_start()]),
					&(dc->dataBuffer[dc->data_end()]));
				if (msg.find("cmd") != msg.end() && msg["cmd"].is_string()) {
					ScheduledCommand sc;
					sc.cmd = msg;

					if (msg.find("time") != msg.end() && msg["time"].is_number()) {
						sc.time = std::chrono::milliseconds(msg["time"].get<long long>());
					} else {
						sc.time = NOW();
					}

					std::cout << "\n>>" << msg.dump(2) << "<< in ms: " << (sc.time - NOW()).count() << std::endl;
					scheduled_commands.push(sc);
				} else {
					std::cerr << "\nERROR: Unknown JSON message:\n" << msg.dump(2) << std::endl;
				}

			} catch (int e) {
				std::cerr << "\nERROR: Misformatted JSON message:\n" << raw << std::endl;
			}
		}

		client->ReleaseForReceiving(&dc);
	}


	void RGBDStreamer::ConfigStream() {
		linesPerMessage = (unsigned int)(MAX_UDP_PACKET_SIZE - sizeof(RGBD_HEADER_STRUCT)) / (2 * frame_width());

		stream_config.frameWidth = (unsigned short) frame_width();
		stream_config.frameHeight = (unsigned short) frame_height();
		stream_config.maxLines = (unsigned short) linesPerMessage;
		stream_config.Cx = device_cx();
		stream_config.Cy = device_cy();
		stream_config.Fx = device_fx();
		stream_config.Fy = device_fy();
		stream_config.DepthScale = device_depth_scale();
		stream_config.dataFlags |= RGBD_DATA;

		if (supports_hd()) {
			stream_config.dataFlags |= HD_DATA;
		}

		if (supports_bidx()) {
			stream_config.dataFlags |= BIDX_DATA;
		}

		if (supports_audio()) {
			stream_config.dataFlags |= AUDIO_DATA;
		}

		if (supports_audio()) {
			stream_config.dataFlags |= BODY_DATA;
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


		std::string debugLevelParam("-dl");

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
			} else if (debugLevelParam.compare(argv[count]) == 0) {
				this->debugLevel = std::stoi(argv[count + 1]);
			} else {
				std::cout << "Unknown Parameter: " << argv[count] << std::endl;
			}
		}

	}
	bool rgbd::ScheduledCommand::operator()(ScheduledCommand left, ScheduledCommand right) {
		return left.time > right.time;
	}


	RecordState::RecordState() {
		_recording = false;
		_replaying = false;
		loop = true;
	}

	std::ostream * RecordState::mjpg_writer() {
		if (_out_mjpg.is_open()) return &_out_mjpg;
		else return NULL;
	}

	std::ostream * RecordState::bidx_writer() {
		if (_out_bidx.is_open()) return &_out_bidx;
		else return NULL;
	}

	std::ostream * RecordState::rgbd_writer() {
		if (_out_rgbd.is_open()) return &_out_rgbd;
		else return NULL;
	}

	std::ostream * RecordState::body_writer() {
		if (_out_body.is_open()) return &_out_body;
		else return NULL;
	}

	std::ostream * RecordState::audio_writer() {
		if (_out_audio.is_open()) return &_out_audio;
		else return NULL;
	}

	std::ostream * RecordState::meta_writer() {
		if (_out_meta.is_open()) return &_out_meta;
		else return NULL;
	}

	std::istream * RecordState::rgbd_reader() {
		if (_in_rgbd.is_open()) return &_in_rgbd;
		else return NULL;
	}

	std::istream * RecordState::audio_reader() {
		if (_in_audio.is_open()) return &_in_audio;
		else return NULL;
	}

	std::istream * RecordState::body_reader() {
		if (_in_body.is_open()) return &_in_body;
		else return NULL;
	}

	std::istream * RecordState::bidx_reader() {
		if (_in_bidx.is_open()) return &_in_bidx;
		else return NULL;
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
		return &(_current_replay_file->config());
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
				if (_current_replay_file != NULL && _current_replay_file->name() == name) {
					StopReplaying();
				}
				files_meta.erase(name);
			}
			else {
				std::cerr << "\nCannot start recording. Name " << name << " already exists." << std::endl;
				return false;
			}
		}

		CONFIG_STRUCT rec_config;
		memcpy(&rec_config, &config, sizeof(CONFIG_STRUCT));
		memcpy(&(rec_config.filename[0]), name.c_str(), name.length() + 1);

		std::string filename_audio = PATH + name + AUDIO_SUFFIX;
		std::string filename_body = PATH + name + BODY_SUFFIX;
		std::string filename_rgbd = PATH + name + RGBD_SUFFIX;
		std::string filename_mjpg = PATH + name + MJPG_SUFFIX;
		std::string filename_bidx = PATH + name + BIDX_SUFFIX;
		std::string filename_hd = PATH + name + HD_SUFFIX;
		std::string filename_meta = PATH + name + META_SUFFIX;

		_current_frame = 0;
		_out_rgbd.open(filename_rgbd, std::ios::binary | std::ios::out | std::ios::trunc);
		_out_meta.open(filename_meta, std::ios::binary | std::ios::out | std::ios::trunc);

		if (rec_config.dataFlags & AUDIO_DATA) {
			_out_audio.open(filename_audio, std::ios::binary | std::ios::out | std::ios::trunc);
		}
		if (rec_config.dataFlags & BODY_DATA) {
			_out_body.open(filename_body, std::ios::binary | std::ios::out | std::ios::trunc);
		}
		if (rec_config.dataFlags & HD_DATA) {
			_out_mjpg.open(filename_mjpg, std::ios::binary | std::ios::out | std::ios::trunc);
		}
		if (rec_config.dataFlags & BIDX_DATA) {
			_out_bidx.open(filename_bidx, std::ios::binary | std::ios::out | std::ios::trunc);
			//avencode = new oi::util::av::AVEncodeTest(filename_hd);
		}

		unsigned short version = 1; // TODO: make this global
		_out_meta.write((const char*)&version, sizeof(version));

		rec_config.dataFlags &= ~LIVE_DATA;
		_out_meta.write((const char*)&rec_config, sizeof(CONFIG_STRUCT));

		_last_rgbd_pos = 0;
		_current_recording_name = name;
		_recording = true;
		_current_recording_start_time = NOW();
		std::cout << "\nINFO: Started Recording " << name << std::endl;
		return true;
	}

	bool RecordState::StopRecording() {
		if (!_recording) return false;

		bool res = true;

		if (_out_audio.is_open()) {
			std::cout << "Closed audio file." << std::endl;
			_out_audio.close();
			_out_audio.clear();
		}

		if (_out_body.is_open()) {
			std::cout << "Closed body file." << std::endl;
			_out_body.close();
			_out_body.clear();
		}

		if (_out_mjpg.is_open()) {
			std::cout << "Closed mjpg file." << std::endl;
			_out_mjpg.close();
			_out_mjpg.clear();
			//if (avencode != NULL) avencode->Close();
		}

		if (_out_bidx.is_open()) {
			std::cout << "Closed bidx file." << std::endl;
			_out_bidx.close();
			_out_bidx.clear();
			//if (avencode != NULL) avencode->Close();
		}

		_out_rgbd.close();
		_out_rgbd.clear();
		_out_meta.close();
		_out_meta.clear();
		_recording = false;
		if (!LoadMeta(_current_recording_name)) {
			std::cerr << "Stopped recording but failed to load Meta" << std::endl;
			res = false;
		}
		else {
			std::cout << "\nINFO: Stopped Recording " << _current_recording_name << std::endl;
		}
		_current_recording_name = "";
		return res;
	}

	bool RecordState::StartReplaying(std::string name) {
		return StartReplaying(name, std::chrono::milliseconds(0), std::chrono::milliseconds(0), T_REL);
	}

	bool RecordState::StartReplaying(std::string name, std::chrono::milliseconds startSlice, std::chrono::milliseconds endSlice, TimeSpecification ts) {
		if (_recording && name == _current_recording_name) {
			std::cerr << "\nERROR: cannot replay file while recording to it." << std::endl;
			return false;
		}

		if (_replaying) {
			StopReplaying();
		}

		std::string filename_rgbd = PATH + name + RGBD_SUFFIX;
		std::string filename_audio = PATH + name + AUDIO_SUFFIX;
		std::string filename_body = PATH + name + BODY_SUFFIX;

		_in_rgbd.open(filename_rgbd, std::ios::binary | std::ios::in);
		if (files_meta.find(name) == files_meta.end() && (!LoadMeta(name) || _in_rgbd.fail())) {
			std::cout << "\nERROR: Failed to open \"" << name << "\" for replaying" << std::endl;
			return false;
		}

		_in_audio.open(filename_audio, std::ios::binary | std::ios::in);
		if (_in_audio.fail()) {
			std::cout << "\nNo audio file found. Won't replay any audio." << std::endl;
		}

		_in_body.open(filename_body, std::ios::binary | std::ios::in);
		if (_in_body.fail()) {
			std::cout << "\nNo body file found. Won't replay any audio." << std::endl;
		}

		_current_replay_file = &files_meta[name];

		// Play whole file instead
		if ((startSlice + endSlice).count() == 0 && ts == T_REL) {
			endSlice = _current_replay_file->duration();
		}

		std::cout << "\nSTARTING SLICE IN FILE " << name << " FROM " << startSlice.count() << " TO " << endSlice.count() << "\n";

		try {
			_pos_start_rgbd =   _current_replay_file->getRGBDPosition(startSlice, ts);
			_pos_end_rgbd =     _current_replay_file->getRGBDPosition(endSlice, ts);
			_pos_start_audio =  _current_replay_file->getAudioPosition(startSlice, ts);
			_pos_end_audio =    _current_replay_file->getAudioPosition(endSlice, ts);
			_pos_start_body =   _current_replay_file->getBodyPosition(startSlice, ts);
			_pos_end_body =     _current_replay_file->getBodyPosition(endSlice, ts);
			_pos_start_bidx =   _current_replay_file->getBIDXPosition(startSlice, ts);
			_pos_end_bidx =     _current_replay_file->getBIDXPosition(endSlice, ts);
			_frame_start_rgbd = _current_replay_file->getFrameByTime(startSlice, ts);
		} catch (int e) {
			std::cerr << "\nERROR: Failed to start playing slice: start/end times out of range!" << std::endl;
			StopReplaying();
			return false;
		}

		std::cout << "MEM: " << _pos_start_rgbd << " TO " << _pos_end_rgbd << "\n";

		_replaying = true;
		ReplayReset();
		return true;
	}

	void RecordState::ReplayReset() {
		_next_replay_frame = _frame_start_rgbd;

		if (_in_rgbd.is_open()) {
			_in_rgbd.clear();
			_in_rgbd.seekg(_pos_start_rgbd, std::ios::beg);
		}
		if (_in_audio.is_open()) {
			_in_audio.clear();
			_in_audio.seekg(_pos_start_audio, std::ios::beg);
		}
		if (_in_body.is_open()) {
			_in_body.clear();
			_in_body.seekg(_pos_start_body, std::ios::beg);
		}
		if (_in_bidx.is_open()) {
			_in_bidx.clear();
			_in_body.seekg(_pos_start_bidx, std::ios::beg);
		}
		_replay_start_time = oi::core::rgbd::NOW();
		_replay_next_frame = _replay_start_time;
	}

	bool RecordState::StopReplaying() {

		if (_in_rgbd.is_open()) {
			_in_rgbd.close();
			_in_rgbd.clear();
		}
		if (_in_audio.is_open()) {
			_in_audio.close();
			_in_audio.clear();
		}
		if (_in_body.is_open()) {
			_in_body.close();
			_in_body.clear();
		}
		if (_in_bidx.is_open()) {
			_in_bidx.close();
			_in_bidx.clear();
		}

		_current_replay_file = NULL;
		_replaying = false;
		return true;
	}

	CONFIG_STRUCT FileMeta::config() {
		return _config;
	}

	std::chrono::milliseconds FileMeta::frameTime(unsigned int frame) {
		return _frames[frame].timestamp;
	}

	std::chrono::milliseconds FileMeta::startTime()
	{
		return _frames.front().timestamp;
	}

	std::chrono::milliseconds FileMeta::endTime()
	{
		return _frames.back().timestamp;
	}

	std::chrono::milliseconds FileMeta::duration()
	{
		return endTime()-startTime();
	}

	unsigned int FileMeta::frame_count() {
		return _frames.size();
	}

	unsigned int FileMeta::getFrameByTime(std::chrono::milliseconds t, TimeSpecification ts) {
		std::chrono::milliseconds _t;
		if (ts == T_REL) {
			if (t < std::chrono::milliseconds(0) || t > duration()) {
				throw std::invalid_argument("Relative time not in range of recording.");
			}
			_t = t;
		}
		else {
			if (t < startTime() || t > endTime()) {
				throw std::invalid_argument("Absolute time not in range of recording.");
			}
			_t = t - startTime();
		}
		return (*_frames_by_time.lower_bound(_t)).second;
	}

	unsigned long long FileMeta::getBIDXPosition(std::chrono::milliseconds t, TimeSpecification ts) {
		return _frames[getFrameByTime(t, ts)].memory_pos_bidx;
	}

	unsigned long long FileMeta::getBodyPosition(std::chrono::milliseconds t, TimeSpecification ts) {
		return _frames[getFrameByTime(t, ts)].memory_pos_body;
	}

	unsigned long long FileMeta::getAudioPosition(std::chrono::milliseconds t, TimeSpecification ts) {
		return _frames[getFrameByTime(t, ts)].memory_pos_audio;
	}

	unsigned long long FileMeta::getRGBDPosition(std::chrono::milliseconds t, TimeSpecification ts) {
		return _frames[getFrameByTime(t, ts)].memory_pos_rgbd;
	}

	void rgbd::FileMeta::Load(std::string name, std::ifstream * in_meta) {
		this->_name = name;
		in_meta->read(reinterpret_cast<char *>(&_recording_version), sizeof(_recording_version));
		in_meta->read(reinterpret_cast<char *>(&_config), sizeof(_config));
		std::chrono::milliseconds t0;
		unsigned int frame_nr = 0;
		while (true) {
			META_STRUCT frame_meta;
			in_meta->read(reinterpret_cast<char *>(&frame_meta), sizeof(frame_meta));
			in_meta->ignore(frame_meta.payload_size);
			if (in_meta->eof()) break;

			if (frame_nr == 0) {
				t0 = frame_meta.timestamp;
			}

			_frames.push_back(frame_meta);
			//std::cout << (frame_meta.timestamp - t0).count() << "\t D: " << frame_meta.memory_pos_rgbd << "\tA: " << frame_meta.memory_pos_audio << "\tB" << frame_meta.memory_pos_body << "\n";
			_frames_by_time.insert(
				std::pair<std::chrono::milliseconds, unsigned int>(
					(frame_meta.timestamp - t0), frame_nr
				)
			);

			frame_nr++;
		}

		std::cout << "META loaded for " << name << ". Frames: " << frame_nr << " Duration: " << duration().count() << std::endl;
	}

	std::string FileMeta::name() {
		return _name;
	}



} } }
