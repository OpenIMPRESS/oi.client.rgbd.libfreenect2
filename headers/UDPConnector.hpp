#pragma once
#include <asio.hpp>
#include <chrono>
#include <thread>
#include <queue>
#include <string>
#include <iostream>
#include <cstdlib>

#include "json.hpp"

namespace oi { namespace core { namespace network {

	class OldUDPConnector {
	public:

		OldUDPConnector(asio::io_service& io_service);

		// Configure the UDPConnector
		void Init(std::string _socketID, std::string _UID, bool _isSender, const std::string& host, const std::string& port, bool _useMatchmaking);


		// Send data to destination
		void SendData(unsigned char* data, size_t size);
		void SendData(char* data, size_t size);

		// Close connection & worker threads
		void Close();

		// Check connection status
		bool isConnected();

		// TODO: check incomming data
		nlohmann::json ReadData();

	private:
		std::queue<nlohmann::json> msgQueue;

		asio::io_service& io_service_;
		asio::ip::udp::socket socket_;
		asio::ip::udp::endpoint serverEndpoint_;
		asio::ip::udp::endpoint clientEndpoint_;

		std::string localIP;
		std::string socketID;
		std::string UID;

		bool isSender;
		bool useMatchmaking;
		bool connected = false;

		std::thread _listenThread;
		bool _listenRunning = false;
		std::thread _updateThread;
		bool _updateRunning = false;

		std::chrono::milliseconds lastRegister;
		std::chrono::milliseconds registerInterval;
		std::chrono::milliseconds lastSentHB;
		std::chrono::milliseconds HBInterval;
		std::chrono::milliseconds lastReceivedHB;
		std::chrono::milliseconds connectionTimeout;

		void _SendData(const std::string& msg, asio::ip::udp::endpoint endpoint);
		std::string getLocalIP();
		void DataListener();
		void HandleReceivedData(char* data, int len);
		void Update();
		void Register();
		void Punch();
	};

	class DataContainer {
	public:
		DataContainer(int i, size_t bs) {
			id = i;
			bufferSize = bs;
			dataBuffer = new unsigned char[bs];
			_data_end = 0;
		}

		int id;
		size_t bufferSize;
		unsigned char * dataBuffer;
		friend class UDPBase;
		char packet_id();
		unsigned int header_start();
		unsigned int data_start();
		unsigned int data_end();

		unsigned int _header_start;
		unsigned int _data_start;
		unsigned int _data_end;
		asio::ip::udp::endpoint endpoint();
	private:
		asio::ip::udp::endpoint _endpoint;
	};

	class UDPBase {
	public:
		UDPBase(int listenPort, int sendPort, std::string sendHost, asio::io_service& io_service);

		/// Starts listening
		bool Init(size_t receive_buffer_size, int receive_containers, size_t send_buffer_size, int send_containers);

		/// Stop listening, deallocate resources.
		void Close();



		/// Get unused container to put receive data into
		bool GetFreeReceiveContainer(DataContainer ** container);

		/// Queue Container with data for receiving
		void QueueForReading(DataContainer ** container);


		/// Dequeue container with received data for processing
		bool DequeueForReading(DataContainer ** container);

		/// Release container to unused receive queue
		void ReleaseForReceiving(DataContainer ** container);


		/// Queue Container with data for sending
		bool GetFreeWriteContainer(DataContainer ** container);

		/// Queue Container with data for sending
		void QueueForSending(DataContainer ** container);
		void QueueForSending(DataContainer ** container, asio::ip::udp::endpoint ep);




	private:
		/// Dequeue container with received data for sending
		/// NEED TO HAVE LOCK
		bool DequeueForSending(DataContainer ** container);

		/// Release container to unused send queue
		/// NEED TO HAVE LOCK
		void ReleaseForWriting(DataContainer ** container);

		int SendData(unsigned char* data, size_t size);
		int SendData(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);
		int _SendData(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);

		std::queue<DataContainer*> unused_receive;
		std::queue<DataContainer*> queued_receive;

		std::queue<DataContainer*> unused_send;
		std::queue<DataContainer*> queued_send;

		std::condition_variable send_cv;
		std::mutex listen_mutex;
		std::mutex send_mutex;

		void DataListener();
		void DataSender();

		int listenPort;
		int sendPort;
		bool running;
		std::thread * listen_thread;
		std::thread * send_thread;
		std::string sendHost;
		asio::io_service& io_service_;
		asio::ip::udp::socket socket_;
		asio::ip::udp::resolver resolver_;
		asio::ip::udp::endpoint endpoint_;
	};

	class UDPConnector : public UDPBase {
	public:
		UDPConnector();
		size_t SendData(unsigned char* data, size_t size);

	};
} } } // oi::core::network
