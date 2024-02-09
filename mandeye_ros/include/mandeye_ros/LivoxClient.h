#pragma once

#include "livox_lidar_def.h"
#include <deque>
#include <json.hpp>
#include <mutex>
#include <thread>
#include "utils/TimeStampProvider.h"
#include <set>
namespace mandeye
{
typedef struct
{
	int32_t x; /**< X axis, Unit:mm */
	int32_t y; /**< Y axis, Unit:mm */
	int32_t z; /**< Z axis, Unit:mm */
	uint8_t reflectivity; /**< Reflectivity */
	uint8_t tag; /**< Tag */
} LivoxLidarCartesianHighRawPoint;

typedef struct
{
	uint8_t dev_type;
	char sn[16];
	char lidar_ip[16];
} LivoxLidarInfo;

struct LivoxPoint
{
	LivoxLidarCartesianHighRawPoint point;
	uint64_t timestamp;
	uint8_t line_id;
	uint16_t laser_id;
};

typedef struct
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
} LivoxLidarImuRawPoint;

struct LivoxIMU
{
	LivoxLidarImuRawPoint point;
	uint64_t timestamp;
	uint16_t laser_id;
};

typedef struct {
  uint8_t ret_code; /**< Return code. */
} LivoxLidarRebootResponse;

typedef struct {
  uint8_t ret_code;
  uint16_t error_key;
} LivoxLidarAsyncControlResponse;


typedef struct {
  uint8_t version;
  uint16_t length;
  uint16_t time_interval;      /**< unit: 0.1 us */
  uint16_t dot_num;
  uint16_t udp_cnt;
  uint8_t frame_cnt;
  uint8_t data_type;
  uint8_t time_type;
  uint8_t rsvd[12];
  uint32_t crc32;
  uint8_t timestamp[8];
  uint8_t data[1];             /**< Point cloud data. */
} LivoxLidarEthernetPacket;

typedef struct {
  uint8_t ret_code;
  uint16_t param_num;
  uint8_t data[1];
} LivoxLidarDiagInternalInfoResponse;

/** Fuction return value defination, refer to \ref LivoxStatus. */
typedef int32_t livox_status;

using LivoxPointsBuffer = std::deque<LivoxPoint>;
using LivoxPointsBufferPtr = std::shared_ptr<std::deque<LivoxPoint>>;
using LivoxPointsBufferConstPtr = std::shared_ptr<const std::deque<LivoxPoint>>;

using LivoxIMUBuffer = std::deque<LivoxIMU>;
using LivoxIMUBufferPtr = std::shared_ptr<std::deque<LivoxIMU>>;
using LivoxIMUBufferConstPtr = std::shared_ptr<const std::deque<LivoxIMU>>;

} // namespace mandeye