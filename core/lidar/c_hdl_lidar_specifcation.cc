/*
 * c_hdl_lidar_specifcation.cc
 *
 *  Created on: Mar 20, 2022
 *      Author: amyznikov
 */

#include "c_hdl_lidar_specifcation.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<HDLSensorType>()
{
  static const c_enum_member members[] = {
      { HDLSensor_HDL32E, "HDL32E", "" },
      { HDLSensor_VLP16, "VLP16", "" },
      { HDLSensor_VLP32AB, "VLP32AB", "" },
      { HDLSensor_VLP16HiRes, "VLP16HiRes", "" },
      { HDLSensor_VLP32C, "VLP32C", "" },
      { HDLSensor_HDL64, "HDL64", "" },
      { HDLSensor_VLS128, "VLS128", "" },
      { HDLSensor_unknown, nullptr, nullptr },
  };

  return members;
}


template<>
const c_enum_member* members_of<HDLReturnMode>()
{
  static const c_enum_member members[] = {
      { HDL_STRONGEST_RETURN, "STRONGEST_RETURN", "" },
      { HDL_LAST_RETURN, "LAST_RETURN", "" },
      { HDL_DUAL_RETURN, "DUAL_RETURN", "" },
      { HDL_TRIPLE_RETURN, "TRIPLE_RETURN", "" },
      { HDL_DUAL_RETURN_WITH_CONFIDENCE, "DUAL_RETURN_WITH_CONFIDENCE", "" },
      { HDLReturnMode_unknown, nullptr, nullptr },
  };

  return members;
}
template<>
const c_enum_member * members_of<HDLFramingMode>()
{
  static const c_enum_member members[] = {
      { HDLFraming_Rotation, "Rotation", "" },
      { HDLFraming_Packet, "Packet", "" },
      { HDLFraming_DataBlock, "Block", "" },
      { HDLFraming_Rotation, nullptr },
  };

  return members;
}

static const c_hdl_lidar_specifcation default_vlp16_lidar_specifcation = {
    .sensor = HDLSensor_VLP16,
    .distance_resolution = 0.002, // [m]
    .lasers = {
        { 0, 0, 0, -15 },
        { 1, 8, 0, 1 },
        { 2, 1, 0, -13 },
        { 3, 9, 0, 3 },
        { 4, 2, 0, -11 },
        { 5, 10, 0, 5 },
        { 6, 3, 0, -9 },
        { 7, 11, 0, 7 },
        { 8, 4, 0, -7 },
        { 9, 12, 0, 9 },
        { 10, 5, 0, -5 },
        { 11, 13, 0, 11 },
        { 12, 6, 0, -3 },
        { 13, 14, 0, 13 },
        { 14, 7, 0, -1 },
        { 15, 15, 0, 15 },
    }
};

static const c_hdl_lidar_specifcation default_vlp32a_lidar_specifcation = {
    .sensor = HDLSensor_VLP32AB,
    .distance_resolution = 0.002, // [m]
    .lasers = {
        {0,0,-1.400,-25.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {1,17,+4.200,-1.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {2,15,-1.400,-1.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {3,1,+1.400,-15.639,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {4,2,-1.400,-11.310,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {5,20,+1.400,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {6,18,-4.200,-0.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {7,3,+1.400,-8.843,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {8,4,-1.400,-7.254,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {9,21,+4.200,+0.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {10,19,-1.400,-0.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {11,5,+1.400,-6.148,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {12,6,-4.200,-5.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {13,24,+1.400,+1.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {14,22,-4.200,+0.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {15,8,+1.400,-4.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {16,7,-1.400,-4.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {17,25,+4.200,+1.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {18,23,-1.400,+1.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {19,9,+4.200,-3.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {20,10,-4.200,-3.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {21,27,+1.400,+3.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {22,26,-1.400,+2.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {23,12,+1.400,-2.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {24,11,-1.400,-3.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {25,29,+1.400,+7.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {26,28,-1.400,+4.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {27,13,+4.200,-2.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {28,14,-4.200,-2.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {29,31,+1.400,+15.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {30,30,-1.400,+10.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {31,16,+1.400,-1.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
    }
};

static const struct c_hdl_lidar_specifcation default_vlp32c_lidar_specifcation = {
    .sensor = HDLSensor_VLP32C,
    .distance_resolution = 0.004, // [m]
    .lasers = {
        {0,0,-1.400,-25.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {1,17,+4.200,-1.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {2,15,-1.400,-1.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {3,1,+1.400,-15.639,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {4,2,-1.400,-11.310,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {5,20,+1.400,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {6,18,-4.200,-0.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {7,3,+1.400,-8.843,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {8,4,-1.400,-7.254,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {9,21,+4.200,+0.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {10,19,-1.400,-0.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {11,5,+1.400,-6.148,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {12,6,-4.200,-5.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {13,24,+1.400,+1.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {14,22,-4.200,+0.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {15,8,+1.400,-4.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {16,7,-1.400,-4.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {17,25,+4.200,+1.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {18,23,-1.400,+1.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {19,9,+4.200,-3.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {20,10,-4.200,-3.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {21,27,+1.400,+3.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {22,26,-1.400,+2.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {23,12,+1.400,-2.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {24,11,-1.400,-3.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {25,29,+1.400,+7.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {26,28,-1.400,+4.667,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {27,13,+4.200,-2.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {28,14,-4.200,-2.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {29,31,+1.400,+15.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {30,30,-1.400,+10.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {31,16,+1.400,-1.333,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
    }
};

static const struct c_hdl_lidar_specifcation default_hdl32e_lidar_specifcation = {
    .sensor = HDLSensor_HDL32E,
    .distance_resolution = 0.002, // [m]
    .lasers = {
        {0,0,+0.000,-30.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {1,16,+0.000,-9.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {2,1,+0.000,-29.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {3,17,+0.000,-8.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {4,2,+0.000,-28.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {5,18,+0.000,-6.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {6,3,+0.000,-26.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {7,19,+0.000,-5.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {8,4,+0.000,-25.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {9,20,+0.000,-4.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {10,5,+0.000,-24.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {11,21,+0.000,-2.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {12,6,+0.000,-22.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {13,22,+0.000,-1.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {14,7,+0.000,-21.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {15,23,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {16,8,+0.000,-20.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {17,24,+0.000,+1.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {18,9,+0.000,-18.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {19,25,+0.000,+2.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {20,10,+0.000,-17.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {21,26,+0.000,+4.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {22,11,+0.000,-16.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {23,27,+0.000,+5.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {24,12,+0.000,-14.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {25,28,+0.000,+6.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {26,13,+0.000,-13.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {27,29,+0.000,+8.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {28,14,+0.000,-12.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {29,30,+0.000,+9.330,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {30,15,+0.000,-10.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
        {31,31,+0.000,+10.670,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000,+0.000},
    }
};

static const struct c_hdl_lidar_specifcation default_hdl64_lidar_specifcation = {
    .sensor = HDLSensor_HDL64,
    .distance_resolution = 0.002, // [m]
    .lasers = {
        {0,36,-4.550,-6.290,+2.150,+0.260,+1.238,+1.281,+1.251,+17.000,+1.200},
        {1,37,-2.460,-5.890,+2.140,-0.260,+1.390,+1.408,+1.408,+24.000,+1.100},
        {2,58,+3.300,+1.150,+2.050,+0.260,+1.369,+1.389,+1.373,+23.000,+0.900},
        {3,59,+5.520,+1.520,+2.050,-0.260,+1.359,+1.379,+1.385,+24.000,+1.300},
        {4,38,-0.230,-5.610,+2.140,+0.260,+1.271,+1.331,+1.266,+24.000,+1.300},
        {5,39,+2.050,-5.290,+2.130,-0.260,+1.425,+1.445,+1.442,+24.000,+1.300},
        {6,32,-1.010,-7.630,+2.160,+0.260,+1.316,+1.381,+1.364,+16.000,+0.400},
        {7,33,+1.220,-7.260,+2.160,-0.260,+1.333,+1.352,+1.334,+16.500,+1.300},
        {8,40,+4.050,-4.960,+2.130,+0.260,+1.295,+1.339,+1.306,+24.000,+1.300},
        {9,41,+6.290,-4.570,+2.120,-0.260,+1.375,+1.404,+1.409,+21.500,+1.100},
        {10,34,+3.260,-7.000,+2.150,+0.260,+1.238,+1.301,+1.291,+20.000,+0.400},
        {11,35,+5.540,-6.640,+2.150,-0.260,+1.328,+1.363,+1.394,+14.000,+1.200},
        {12,48,-4.550,-2.220,+2.090,+0.260,+1.318,+1.348,+1.331,+19.500,+1.100},
        {13,49,-2.280,-1.840,+2.090,-0.260,+1.430,+1.434,+1.441,+24.000,+1.300},
        {14,42,-5.310,-4.260,+2.120,+0.260,+1.297,+1.337,+1.277,+17.000,+1.200},
        {15,43,-3.090,-3.840,+2.110,-0.260,+1.316,+1.331,+1.326,+23.000,+0.900},
        {16,50,-0.260,-1.550,+2.090,+0.260,+1.299,+1.306,+1.287,+24.000,+1.300},
        {17,51,+2.010,-1.180,+2.080,-0.260,+1.433,+1.437,+1.450,+24.000,+1.200},
        {18,44,-1.040,-3.580,+2.110,+0.260,+1.322,+1.362,+1.327,+24.000,+1.300},
        {19,45,+1.210,-3.200,+2.110,-0.260,+1.397,+1.401,+1.398,+24.000,+1.300},
        {20,52,+4.010,-0.880,+2.080,+0.260,+1.295,+1.314,+1.290,+24.000,+1.300},
        {21,53,+6.230,-0.520,+2.070,-0.260,+1.288,+1.316,+1.334,+18.500,+1.100},
        {22,46,+3.230,-2.910,+2.100,+0.260,+1.319,+1.347,+1.308,+24.000,+1.300},
        {23,47,+5.460,-2.530,+2.100,-0.260,+1.386,+1.400,+1.420,+24.000,+1.300},
        {24,60,-4.580,+1.860,+2.040,+0.260,+1.326,+1.363,+1.319,+22.500,+0.900},
        {25,61,-2.320,+2.190,+2.040,-0.260,+1.409,+1.418,+1.402,+24.000,+1.200},
        {26,54,-5.350,-0.150,+2.070,+0.260,+1.367,+1.389,+1.358,+24.000,+1.300},
        {27,55,-3.090,+0.230,+2.060,-0.260,+1.381,+1.372,+1.368,+21.000,+1.100},
        {28,62,-0.280,+2.510,+2.030,+0.260,+1.353,+1.389,+1.383,+16.500,+0.900},
        {29,63,+1.950,+2.810,+2.030,-0.260,+1.305,+1.332,+1.327,+24.000,+0.900},
        {30,56,-1.100,+0.480,+2.060,+0.260,+1.373,+1.383,+1.352,+24.000,+1.300},
        {31,57,+1.180,+0.890,+2.050,-0.260,+1.385,+1.403,+1.396,+24.000,+1.300},
        {32,4,-7.450,-21.770,+1.590,+0.260,+1.207,+1.244,+1.229,+11.000,+1.800},
        {33,5,-3.990,-21.280,+1.580,-0.260,+1.304,+1.361,+1.379,+0.250,+1.000},
        {34,26,+5.010,-10.670,+1.460,+0.260,+1.421,+1.449,+1.476,+0.250,+1.100},
        {35,27,+8.210,-9.990,+1.450,-0.260,+1.291,+1.317,+1.336,+7.500,+1.500},
        {36,6,-0.510,-20.910,+1.580,+0.260,+1.181,+1.293,+1.242,+8.500,+1.400},
        {37,7,+2.960,-20.340,+1.570,-0.260,+1.233,+1.288,+1.278,+9.500,+1.500},
        {38,0,-1.820,-23.950,+1.620,+0.260,+1.378,+1.416,+1.395,+3.500,+1.200},
        {39,1,+1.690,-23.440,+1.610,-0.260,+1.225,+1.257,+1.277,+0.250,+1.100},
        {40,8,+6.430,-19.740,+1.560,+0.260,+1.375,+1.427,+1.438,+2.000,+1.100},
        {41,9,+9.790,-19.140,+1.560,-0.260,+1.226,+1.250,+1.265,+9.000,+1.800},
        {42,2,+5.230,-22.820,+1.600,+0.260,+1.353,+1.385,+1.381,+0.250,+1.100},
        {43,3,+8.700,-22.270,+1.600,-0.260,+1.087,+1.131,+1.130,+8.500,+1.500},
        {44,16,-7.270,-15.580,+1.510,+0.260,+1.228,+1.292,+1.239,+11.000,+1.900},
        {45,17,-3.800,-15.150,+1.510,-0.260,+1.269,+1.320,+1.339,+8.000,+0.700},
        {46,10,-8.630,-18.720,+1.550,+0.260,+1.396,+1.442,+1.419,+11.000,+1.900},
        {47,11,-5.100,-18.190,+1.550,-0.260,+1.186,+1.253,+1.258,+0.250,+0.700},
        {48,18,-0.460,-14.720,+1.500,+0.260,+1.208,+1.247,+1.229,+14.000,+1.300},
        {49,19,+2.860,-14.160,+1.500,-0.260,+1.257,+1.277,+1.306,+12.000,+1.600},
        {50,12,-1.750,-17.770,+1.540,+0.260,+1.204,+1.266,+1.249,+13.000,+1.700},
        {51,13,+1.690,-17.300,+1.530,-0.260,+1.209,+1.241,+1.232,+18.000,+0.900},
        {52,20,+6.130,-13.730,+1.490,+0.260,+1.174,+1.215,+1.233,+0.250,+0.900},
        {53,21,+9.490,-13.130,+1.490,-0.260,+1.282,+1.270,+1.326,+10.000,+1.900},
        {54,14,+5.010,-16.710,+1.530,+0.260,+1.366,+1.431,+1.416,+3.000,+1.000},
        {55,15,+8.430,-16.200,+1.520,-0.260,+1.218,+1.226,+1.231,+11.500,+2.000},
        {56,28,-7.000,-9.370,+1.440,+0.260,+1.294,+1.323,+1.316,+8.500,+1.700},
        {57,29,-3.760,-9.150,+1.440,-0.260,+1.317,+1.333,+1.363,+0.250,+1.300},
        {58,22,-8.300,-12.370,+1.480,+0.260,+1.442,+1.419,+1.429,+10.000,+2.000},
        {59,23,-4.980,-12.050,+1.470,-0.260,+1.258,+1.278,+1.298,+0.250,+0.800},
        {60,30,-0.450,-8.650,+1.440,+0.260,+1.439,+1.477,+1.454,+5.000,+1.400},
        {61,31,+2.650,-8.110,+1.430,-0.260,+1.298,+1.341,+1.354,+3.500,+1.300},
        {62,24,-1.710,-11.590,+1.470,+0.260,+1.392,+1.425,+1.396,+11.500,+1.300},
        {63,25,+1.540,-11.120,+1.460,-0.260,+1.268,+1.299,+1.288,+10.000,+1.000},
    }
};

static const struct c_hdl_lidar_specifcation default_vls128_lidar_specifcation = {
    .sensor = HDLSensor_VLS128,
    .distance_resolution = 0.004, // [m]
    .lasers = {
        { 0, 4, -6.354, -11.742 },
        { 1, 53, -4.548, -1.99 },
        { 2, 102, -2.732, 3.4 },
        { 3, 22, -0.911, -5.29 },
        { 4, 64, 0.911, -0.78 },
        { 5, 113, 2.732, 4.61 },
        { 6, 34, 4.548, -4.08 },
        { 7, 83, 6.354, 1.31 },
        { 8, 12, -6.354, -6.5 },
        { 9, 61, -4.548, -1.11 },
        { 10, 110, -2.732, 4.28 },
        { 11, 31, -0.911, -4.41 },
        { 12, 72, 0.911, 0.1 },
        { 13, 121, 2.732, 6.48 },
        { 14, 42, 4.548, -3.2 },
        { 15, 91, 6.354, 2.19 },
        { 16, 36, -6.354, -3.86 },
        { 17, 85, -4.548, 1.53 },
        { 18, 6, -2.732, -9.244 },
        { 19, 55, -0.911, -1.77 },
        { 20, 96, 0.911, 2.74 },
        { 21, 17, 2.732, -5.95 },
        { 22, 66, 4.548, -0.56 },
        { 23, 115, 6.354, 4.83 },
        { 24, 44, -6.354, -2.98 },
        { 25, 93, -4.548, 2.41 },
        { 26, 14, -2.732, -6.28 },
        { 27, 63, -0.911, -0.89 },
        { 28, 104, 0.911, 3.62 },
        { 29, 25, 2.732, -5.07 },
        { 30, 74, 4.548, 0.32 },
        { 31, 123, 6.354, 7.58 },
        { 32, 68, -6.354, -0.34 },
        { 33, 117, -4.548, 5.18 },
        { 34, 38, -2.732, -3.64 },
        { 35, 87, -0.911, 1.75 },
        { 36, 0, 0.911, -25 },
        { 37, 49, 2.732, -2.43 },
        { 38, 98, 4.548, 2.96 },
        { 39, 19, 6.354, -5.73 },
        { 40, 76, -6.354, 0.54 },
        { 41, 125, -4.548, 9.7 },
        { 42, 46, -2.732, -2.76 },
        { 43, 95, -0.911, 2.63 },
        { 44, 8, 0.911, -7.65 },
        { 45, 57, 2.732, -1.55 },
        { 46, 106, 4.548, 3.84 },
        { 47, 27, 6.354, -4.85 },
        { 48, 100, -6.354, 3.18 },
        { 49, 24, -4.548, -5.11 },
        { 50, 70, -2.732, -0.12 },
        { 51, 119, -0.911, 5.73 },
        { 52, 32, 0.911, -4.3 },
        { 53, 81, 2.732, 1.09 },
        { 54, 2, 4.548, -16.042 },
        { 55, 51, 6.354, -2.21 },
        { 56, 108, -6.354, 4.06 },
        { 57, 29, -4.548, -4.63 },
        { 58, 78, -2.732, 0.76 },
        { 59, 127, -0.911, 15 },
        { 60, 40, 0.911, -3.42 },
        { 61, 89, 2.732, 1.97 },
        { 62, 10, 4.548, -6.85 },
        { 63, 59, 6.354, -1.33 },
        { 64, 20, -6.354, -5.62 },
        { 65, 69, -4.548, -0.23 },
        { 66, 118, -2.732, 5.43 },
        { 67, 39, -0.911, -3.53 },
        { 68, 80, 0.911, 0.98 },
        { 69, 1, 2.732, -19.582 },
        { 70, 50, 4.548, -2.32 },
        { 71, 99, 6.354, 3.07 },
        { 72, 28, -6.354, -4.74 },
        { 73, 77, -4.548, 0.65 },
        { 74, 126, -2.732, 11.75 },
        { 75, 47, -0.911, -2.65 },
        { 76, 88, 0.911, 1.86 },
        { 77, 9, 2.732, -7.15 },
        { 78, 58, 4.548, -1.44 },
        { 79, 107, 6.354, 3.95 },
        { 80, 52, -6.354, -2.1 },
        { 81, 101, -4.548, 3.29 },
        { 82, 21, -2.732, -5.4 },
        { 83, 71, -0.911, -0.01 },
        { 84, 112, 0.911, 4.5 },
        { 85, 33, 2.732, -4.19 },
        { 86, 82, 4.548, 1.2 },
        { 87, 3, 6.354, -13.565 },
        { 88, 60, -6.354, -1.22 },
        { 89, 109, -4.548, 4.17 },
        { 90, 30, -2.732, -4.52 },
        { 91, 79, -0.911, 0.87 },
        { 92, 120, 0.911, 6.08 },
        { 93, 41, 2.732, -3.31 },
        { 94, 90, 4.548, 2.08 },
        { 95, 11, 6.354, -6.65 },
        { 96, 84, -6.354, 1.42 },
        { 97, 5, -4.548, -10.346 },
        { 98, 54, -2.732, -1.88 },
        { 99, 103, -0.911, 3.51 },
        { 100, 16, 0.911, -6.06 },
        { 101, 65, 2.732, -0.67 },
        { 102, 114, 4.548, 4.72 },
        { 103, 35, 6.354, -3.97 },
        { 104, 92, -6.354, 2.3 },
        { 105, 13, -4.548, -6.39 },
        { 106, 62, -2.732, -1 },
        { 107, 111, -0.911, 4.39 },
        { 108, 23, 0.911, -5.18 },
        { 109, 73, 2.732, 0.21 },
        { 110, 122, 4.548, 6.98 },
        { 111, 43, 6.354, -3.09 },
        { 112, 116, -6.354, 4.98 },
        { 113, 37, -4.548, -3.75 },
        { 114, 86, -2.732, 1.64 },
        { 115, 7, -0.911, -8.352 },
        { 116, 48, 0.911, -2.54 },
        { 117, 97, 2.732, 2.85 },
        { 118, 18, 4.548, -5.84 },
        { 119, 67, 6.354, -0.45 },
        { 120, 124, -6.354, 8.43 },
        { 121, 45, -4.548, -2.87 },
        { 122, 94, -2.732, 2.52 },
        { 123, 15, -0.911, -6.17 },
        { 124, 56, 0.911, -1.66 },
        { 125, 105, 2.732, 3.73 },
        { 126, 26, 4.548, -4.96 },
        { 127, 75, 6.354, 0.43 },
    }
};

static std::string vlp16_lidar_config_file;
static std::string vlp32a_lidar_config_file;
static std::string vlp32c_lidar_config_file;
static std::string hdl32e_lidar_config_file;
static std::string hdl64_lidar_config_file;
static std::string vls128_lidar_config_file;

const c_hdl_lidar_specifcation * get_default_hdl_lidar_specification(HDLSensorType sensor_type)
{
  switch (sensor_type) {
    case HDLSensor_VLP16:
      return &default_vlp16_lidar_specifcation;
    case HDLSensor_VLP16HiRes:
      return nullptr;
    case HDLSensor_VLP32AB:
      return &default_vlp32a_lidar_specifcation;
    case HDLSensor_VLP32C:
      return &default_vlp32c_lidar_specifcation;
    case HDLSensor_HDL32E:
      return &default_hdl32e_lidar_specifcation;
    case HDLSensor_HDL64:
      return &default_hdl64_lidar_specifcation;
    case HDLSensor_VLS128:
      return &default_vls128_lidar_specifcation;
    default:
      break;
  }

  return nullptr;
}

std::string get_hdl_lidar_specification_config_file(HDLSensorType sensor_type)
{
  switch (sensor_type) {
  case HDLSensor_VLP16:
    return vlp16_lidar_config_file;
  case HDLSensor_VLP16HiRes:
    return "";
  case HDLSensor_VLP32AB:
    return vlp32a_lidar_config_file;
  case HDLSensor_VLP32C:
    return vlp32c_lidar_config_file;
  case HDLSensor_HDL32E:
    return hdl32e_lidar_config_file;
  case HDLSensor_HDL64:
    return hdl64_lidar_config_file;
  case HDLSensor_VLS128:
    return vls128_lidar_config_file;
  default:
    break;
  }

  CF_ERROR("lidar_specification_db entry not found for sensor_type %s (%d)",
      toString(sensor_type), (int )sensor_type);

  return "";
}

bool set_hdl_lidar_specification_config_file(HDLSensorType sensor_type,
    const std::string & config_file_pathname)
{
  switch (sensor_type) {
  case HDLSensor_VLP16:
    vlp16_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_VLP32AB:
    vlp32a_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_VLP32C:
    vlp32c_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_HDL32E:
    hdl32e_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_HDL64:
    hdl64_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_VLS128:
    vls128_lidar_config_file = config_file_pathname;
    break;
  case HDLSensor_VLP16HiRes:
    default:
    CF_ERROR("lidar_specification_db entry not found for sensor_type %s (%d)",
        toString(sensor_type), (int )sensor_type);
    return false;
  }

  return true;
}

