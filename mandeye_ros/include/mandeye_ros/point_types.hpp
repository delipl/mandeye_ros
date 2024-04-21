#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

// - name: x
//   offset: 0
//   datatype: 7
//   count: 1
// - name: y
//   offset: 4
//   datatype: 7
//   count: 1
// - name: z
//   offset: 8
//   datatype: 7
//   count: 1
// - name: intensity
//   offset: 12
//   datatype: 7
//   count: 1
// - name: tag
//   offset: 16
//   datatype: 2
//   count: 1
// - name: line
//   offset: 17
//   datatype: 2
//   count: 1
// - name: timestamp
//   offset: 18
//   datatype: 8
//   count: 1

namespace pcl
{
struct LivoxPoint
{
	PCL_ADD_POINT4D;
	float intensity;
	uint8_t tag;
	uint8_t line;
	double timestamp;
	PCL_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(
	pcl::LivoxPoint,
	(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))
