#include <iostream>
#include <signal.h>
#include <fstream>
#include <cstdlib>
#include <chrono>

#include <asio.hpp>
#include <turbojpeg.h>
#include "LibFreenect2Streamer.hpp"

asio::io_service io_service_;

int main(int argc, char *argv[]) {
	oi::core::rgbd::StreamerConfig cfg;
	cfg.Parse(argc, argv);
	std::cout << "CFG: " << cfg.listenPort << " " << cfg.remotePort << " " << cfg.remoteHost << std::endl;
	oi::core::network::UDPBase client(cfg.listenPort, cfg.remotePort, cfg.remoteHost, io_service_);
	client.Init(1024, 32, 65506, 32);

	oi::core::rgbd::LibFreenect2Streamer lf2stream(cfg, &client);
	lf2stream.Run();
	lf2stream.Exit();

	system("pause");
}





//UDPConnector * client;

/*
void sigint_handler(int s) {
	stream_shutdown = true;
}
bool kinect_paused = false;
libfreenect2::Freenect2Device *devtopause;
void sigusr1_handler(int s) {
	if (devtopause == 0)  return;
	if (kinect_paused)    devtopause->start();
	else                  devtopause->stop();
	kinect_paused = !kinect_paused;
}*/
/*
int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence);
int openAndStream(string serial, string pipelineId);
int sendConfig();
int replayFrame(uint32_t sequence);
*/

/*
string socketIDValue = "kinect1";
string ipValue = "127.0.0.1"; // mm.openimpress.org
string portValue = "10101"; // 6312
string listenPortValue = "10102"; // ""
string serialValue = "";
string linesValue = "-1"; // 24
string pipelineValue = "opencl"; // opencl
string maxDepthValue = "10";
string fileDumpValue = ""; // dump.bin
string useMMValue = "1";
bool useMM = true;
*/


/*
ofstream dumpStream;
ifstream loadStream;

int currentFrame = 0;
int frameLoopLength = 30*5;
bool replay = false;
milliseconds prevRecFrame;
milliseconds prevReplayFrame;
milliseconds nextReplayFrame;
uint32_t lastSequence;

if (fileDumpValue.length() > 0) {
	dumpStream.open(fileDumpValue, ios::binary | ios::out | ios::trunc);
}
*/


	
	//client->Init(socketIDValue, dev->getSerialNumber(), true, ipValue, portValue, useMM);
    /*
   
	while (!stream_shutdown) {
        milliseconds ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

        
        if (!replay) {
            if (!listener.waitForNewFrame(frames, 5 * 1000)) {
                return -1;
            }

            libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

            registration->apply(rgb, depth, &undistorted, &registered);
            
            if (streamFrame(&undistorted, &registered, rgb->sequence) == -1) {
                stream_shutdown = true;
            }
            
            if (fileDumpValue.length() > 0) {
                //cout << "REC FRAME " << currentFrame << " OF " << frameLoopLength << endl;
            }
            lastSequence = rgb->sequence;
            currentFrame++;
            fpsCounter++;
        }
        
        if (replay && nextReplayFrame <= ms) {
            replayFrame(lastSequence+currentFrame+1);
            
            if (fileDumpValue.length() > 0) {
                //cout << "REP FRAME " << currentFrame << " OF " << frameLoopLength << endl;
            }
            currentFrame++;
            fpsCounter++;
        }

        
        if (currentFrame >= frameLoopLength && fileDumpValue.length() > 0) {
            replay = !replay;
            currentFrame = 0;
            
            if (replay) { // we start replaying...
                cout << ">>> Start replaying" << endl;
                dumpStream.close();
                dumpStream.clear();
                loadStream.open(fileDumpValue, ios::binary | ios::in);
                nextReplayFrame = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            } else {
                cout << ">>> Start recording" << endl;
                loadStream.close();
                loadStream.clear();
                dumpStream.open(fileDumpValue, ios::binary | ios::out | ios::trunc);
                // close file and open it with writer;
                prevRecFrame = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
            }
        }

		if (lastFpsAverage + interval <= ms) {
			lastFpsAverage = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
			cout << "Average FPS: " << fpsCounter / 2.0 << endl;
			fpsCounter = 0;
            sendConfig();
		}

	}

    if (fileDumpValue.length() > 0) {
        dumpStream.close();
        loadStream.close();
    }
    
	dev->stop();
	dev->close();
	delete registration;
    client->Close();
    
    cout << "Ended gracefully." << endl;
    
	return 0;*/


/*

int replayFrame(uint32_t seq) {
    milliseconds now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    long unsigned int delta = 0;
    long unsigned int rgbLength = 0;
    long unsigned int depthLength = 0;
    long unsigned int npackets = 0;
    
    // parse delta
    loadStream.read(reinterpret_cast<char *>(&delta), sizeof(delta));
    //cout << "delta: " << delta << endl;
    
    // parse rgb image size
    loadStream.read(reinterpret_cast<char *>(&rgbLength), sizeof(rgbLength));
    //cout << "rgbLength: " << rgbLength << endl;
    
    // read rgb into memory
    loadStream.read((char *) frameStreamBufferColor, rgbLength); //&frameStreamBufferColor[0]?
    
    // set sequence number in header
    
    // send rgb data
    client->SendData(frameStreamBufferColor, rgbLength);
    
    // parse number of depth packets
    loadStream.read(reinterpret_cast<char *>(&npackets), sizeof(npackets));
    //cout << "npackets: " << npackets << endl;
    
    // for each packet...
    for (unsigned int i = 0; i < npackets; i++) {
        // parse depth image size
        loadStream.read(reinterpret_cast<char *>(&depthLength), sizeof(depthLength));
        ///cout << "Packet " << i << ", depthLength: " << depthLength << endl;
        
        // read depth into memory
        loadStream.read((char *) frameStreamBufferDepth, depthLength);
        
        // set sequence number in header
        
        // send depth data
        client->SendData(frameStreamBufferDepth, depthLength);
    }

    nextReplayFrame = now+milliseconds(delta);
    return 0;
}

int streamFrame(libfreenect2::Frame * regdepth, libfreenect2::Frame * regrgb, uint32_t sequence) {
    milliseconds now = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    milliseconds delta = now-prevRecFrame;
    prevRecFrame = now;
    
    img_header.sequence = sequence;

    // Send color data first...
	img_header.msgType = 0x04;
	img_header.startRow = (unsigned short)0;               // ... we can fit the whole...
	img_header.endRow = (unsigned short)regdepth->height; //...RGB data in one packet
	memcpy(&frameStreamBufferColor[0], &img_header, headerSize);

	// COMPRESS COLOR
	long unsigned int _jpegSize = frameStreamBufferColorSize-headerSize;
	unsigned char* _compressedImage = &frameStreamBufferColor[headerSize];

	// replace  _complressedImage with &frameStreamBufferColor[headerSize]
	tjhandle _jpegCompressor = tjInitCompress();
	tjCompress2(_jpegCompressor, regrgb->data, (int) regdepth->width, 0, (int) regdepth->height, TJPF_BGRX,
		&_compressedImage, &_jpegSize, TJSAMP_444, JPEG_QUALITY,
		TJFLAG_FASTDCT);

	//cout << "JPEG Size: " << _jpegSize << "\n";
	//memcpy(&frameStreamBufferColor[headerSize], _compressedImage, _jpegSize);

    long unsigned int colorDataSize = headerSize + _jpegSize;
    
	try {
        client->SendData(frameStreamBufferColor, colorDataSize);
	} catch (exception& e) {
		cerr << "Exception stream RGB Frame: " << e.what() << "\n";
	}
    
    // deltaMS sizeRGB RGB sizeDepth DEPTH

    if (fileDumpValue.length() > 0 && !replay) {
        long unsigned int deltaValue = (long unsigned int) delta.count();
        //cout << "frame" << currentFrame << " delta:" << deltaValue;
        //cout << " rgbLength:" << colorDataSize;
        
        dumpStream.write((const char*) &deltaValue, sizeof(deltaValue));
        dumpStream.write((const char*) &colorDataSize, sizeof(colorDataSize));
        dumpStream.write((const char*) frameStreamBufferColor, colorDataSize);
    }
    
	tjDestroy(_jpegCompressor);
	

	// Send Depth
	img_header.msgType = 0x03;
    
    unsigned long int npackets = (unsigned long int) (regdepth->height + linesPerMessage - 1) / linesPerMessage; // round up division
    if (fileDumpValue.length() > 0 && !replay) {
        dumpStream.write((const char*) &npackets, sizeof(npackets));
        //cout << " npackets:" << npackets << endl;
    }
    
	for (unsigned int startRow = 0; startRow < regdepth->height; startRow += linesPerMessage) {

		size_t endRow = startRow + linesPerMessage;
		if (endRow >= regdepth->height) endRow = regdepth->height;
		if (startRow >= endRow) break;

		//size_t totalLines = endRow - startRow;

        img_header.startRow = (unsigned short) startRow;
        img_header.endRow = (unsigned short) endRow;
        memcpy(&frameStreamBufferDepth[0], &img_header, headerSize);
        size_t writeOffset = headerSize;

		size_t depthLineSizeR = regdepth->width * regdepth->bytes_per_pixel;
		size_t depthLineSizeW = regdepth->width * 2;
		size_t readOffset = startRow*depthLineSizeR;
		for (int line = startRow; line < endRow; line++) {
			for (int i = 0; i < regdepth->width; i++) {
				float depthValue = 0;
				memcpy(&depthValue, &regdepth->data[readOffset + i * 4], sizeof(depthValue));
				unsigned short depthValueShort = (unsigned short)(depthValue);
				memcpy(&frameStreamBufferDepth[writeOffset + i * 2], &depthValueShort, sizeof(depthValueShort));
                
			}

			writeOffset += depthLineSizeW;
			readOffset += depthLineSizeR;
		}

		//size_t colorLineSizeR = regrgb->width * regrgb->bytes_per_pixel;
		//readOffset = startRow*colorLineSizeR;

		//stb_compress_dxt(&frameStreamBuffer[writeOffset], &regrgb->data[readOffset], 512, (int)totalLines, 0);

		try {
            client->SendData(frameStreamBufferDepth, writeOffset);
		} catch (exception& e) {
			cerr << "Exception stream Depth frame: " << e.what() << "\n";
        }
        long unsigned int depthDataSize = (unsigned long int) writeOffset;
        if (fileDumpValue.length() > 0 && !replay) {
            dumpStream.write((const char*) &depthDataSize, sizeof(depthDataSize));
            dumpStream.write((const char*) frameStreamBufferDepth, depthDataSize);
        }
        
	}

	return 0;
}*/