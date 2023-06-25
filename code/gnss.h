#pragma once

#include <deque>
#include <json.hpp>
#include <mutex>

#include <SerialPort.h>
#include <SerialStream.h>
#include "thread"
#include "utils/TimeStampReceiver.h"
#include "minmea.h"
namespace mandeye
{


class GNSSClient : public mandeye_utils::TimeStampReceiver
{
public:

	nlohmann::json produceStatus();

	//! Spins up a thread that reads from the serial port
	bool startListener(const std::string& portName, int baudRate);

	//! Start logging into the buffers
	void startLog();

	//! Stop logging into the buffers
	void stopLog();

	//! Retrieve all data from the buffer, in form of CSV lines
	std::deque<std::string> retrieveData();

private:
	std::mutex m_bufferMutex;
	std::deque<std::string> m_buffer;
	std::string m_lastLine;
	bool m_isLogging{false};
	minmea_sentence_gga lastGGA;
	LibSerial::SerialPort m_serialPort;
	LibSerial::SerialStream m_serialPortStream;
	std::thread m_serialPortThread;
	void worker();

	bool init_succes{false};

	//! Convert a minmea_sentence_gga to a CSV line
	std::string GgaToCsvLine(const minmea_sentence_gga& gga, double laserTimestamp);

};
} // namespace mandeye