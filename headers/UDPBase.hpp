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

		/// Queues data for sending.
		/// This copyies data to a container internally...
		/// So the queue pattern is better suited for bigger chunks of data!
		int SendData(unsigned char* data, size_t size);
		int SendData(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);
		
		/// Send the data directly without using the send queue
		/// This does not copy, but blocks until data is sent.
		int SendDataBlocking(unsigned char* data, size_t size);
		int SendDataBlocking(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);

		bool connected();
	protected:
		virtual void HandleReceivedData(DataContainer * container);

		bool _connected;

		/// Dequeue container with received data for sending
		/// NEED TO HAVE LOCK
		bool DequeueForSending(DataContainer ** container);

		/// Release container to unused send queue
		/// NEED TO HAVE LOCK
		void ReleaseForWriting(DataContainer ** container);

		virtual int _SendDataBlocking(unsigned char* data, size_t size, asio::ip::udp::endpoint endpoint);

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

} } } // oi::core::network
