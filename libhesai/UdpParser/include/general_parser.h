/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/

/*
 * File:       general_parser.h
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Declare GeneralParser class
*/

#ifndef GENERAL_PARSER_H_
#define GENERAL_PARSER_H_
#define CIRCLE (36000 * 256)
#define MAX_LASER_NUM (512)
#define CIRCLE_ANGLE (36000)
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif
#define M_PI_DIVIDE_180 (0.01745329251994329575)
#define _180_DIVIDE_M_PI (57.29577951308232087721)

#ifndef _MSC_VER
#include <semaphore.h>
#endif
#include <list>
#include <cmath>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include "logger.h"
#include "lidar_types.h"
#include "plat_utils.h"
#include "fault_message.h"
namespace hesai
{
namespace lidar
{  

DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
DEFINE_MEMBER_CHECKER(confidence)
DEFINE_MEMBER_CHECKER(weightFactor)
DEFINE_MEMBER_CHECKER(envLight)

DEFINE_SET_GET(intensity, uint8_t)  
DEFINE_SET_GET(ring, uint16_t)  
DEFINE_SET_GET(timestamp, double)  
DEFINE_SET_GET(timeSecond, uint64_t)  
DEFINE_SET_GET(timeNanosecond, uint32_t)
DEFINE_SET_GET(confidence, uint8_t)  
DEFINE_SET_GET(weightFactor, uint8_t)
DEFINE_SET_GET(envLight, uint8_t)



template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, confidence)>::type setConfidence(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, confidence)>::type setConfidence(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.confidence = value;
}
template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, weightFactor)>::type setWeightFactor(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, weightFactor)>::type setWeightFactor(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.weightFactor = value;
}
template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, envLight)>::type setEnvLight(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, envLight)>::type setEnvLight(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.envLight = value;
}
// get command
template <typename T_Point>
inline typename std::enable_if<!PANDAR_HAS_MEMBER(T_Point, timestamp)>::type getTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<PANDAR_HAS_MEMBER(T_Point, timestamp)>::type getTimestamp(T_Point& point,
                                                                                      double& value)
{
  value = point.timestamp;
}
// get end
inline float deg2Rad(float deg)
{
    return (float)(deg * 0.01745329251994329575);
}

inline float rad2Deg(float rad)
{
    return (float)(rad * 57.29577951308232087721);
}

enum DistanceCorrectionType {
  OpticalCenter,
  GeometricCenter,
};

struct PacketSeqnumLossMessage {
  uint64_t last_total_package_count;
  uint32_t last_seqnum;
  uint32_t loss_count;
  uint32_t start_time;
  uint32_t total_loss_count;
  uint64_t total_packet_count;
  uint32_t max_sequence;
  bool is_packet_loss;
  bool is_init;
  PacketSeqnumLossMessage() {
    last_total_package_count = 0;
    last_seqnum = 0;
    loss_count = 0;
    start_time = 0;
    total_loss_count = 0;
    total_packet_count = 0;
    max_sequence = 0xFFFFFFFF;
    is_packet_loss = false;
    is_init = false;
  }
};

struct PacketTimeLossMessage {
  uint64_t last_timestamp;
  uint32_t timeloss_count;
  uint32_t timeloss_start_time;
  uint32_t total_timeloss_count;
  uint64_t last_total_package_count;
  bool is_init;
  PacketTimeLossMessage() {
    last_timestamp = 0;
    timeloss_count = 0;
    timeloss_start_time = 0;
    total_timeloss_count = 0;
    last_total_package_count = 0;
    is_init = false;
  }
};

enum StructType {
  CORRECTION_STRUCT = 1,
  FIRETIME_STRUCT = 2,
};

struct CorrectionData {
  float elevation[DEFAULT_MAX_LASER_NUM];
  float azimuth[DEFAULT_MAX_LASER_NUM];
  bool display[DEFAULT_MAX_LASER_NUM];
  std::string hash;
  CorrectionData() {
    memset(elevation, 0, sizeof(float) * DEFAULT_MAX_LASER_NUM);
    memset(azimuth, 0, sizeof(float) * DEFAULT_MAX_LASER_NUM);
    for (int i = 0; i < DEFAULT_MAX_LASER_NUM; ++i) {  
        display[i] = true;
    } 
    hash = "";
  }
};

struct LastUtcTime {
  uint64_t last_time = 0;
  int16_t last_utc[6];
  LastUtcTime() {
    last_time = 0;
    last_utc[0] = -1;
    last_utc[1] = -1;
    last_utc[2] = -1;
    last_utc[3] = -1;
    last_utc[4] = -1;
    last_utc[5] = -1;
  }
};

enum DistanceCorrectionType {
  OpticalCenter,
  GeometricCenter,
};

struct PacketSeqnumLossMessage{
  uint32_t start_seqnum;
  uint32_t last_seqnum;
  uint32_t loss_count;
  uint32_t start_time;
  uint32_t total_loss_count;
  uint32_t total_start_seqnum;
  bool is_packet_loss;
  PacketSeqnumLossMessage() {
    start_seqnum = 0;
    last_seqnum = 0;
    loss_count = 0;
    start_time = 0;
    total_loss_count = 0;
    total_start_seqnum = 0;
    is_packet_loss = false;
  }
};

struct PacketTimeLossMessage{
  uint64_t start_timestamp;
  uint64_t last_timestamp;
  uint32_t timeloss_count;
  uint32_t timeloss_start_time;
  uint32_t total_timeloss_count;
  uint64_t total_start_timestamp;
  uint32_t last_total_package_count;
  PacketTimeLossMessage() {
    start_timestamp = 0;
    last_timestamp = 0;
    timeloss_count = 0;
    timeloss_start_time = 0;
    total_timeloss_count = 0;
    total_start_timestamp = 0; 
    last_total_package_count = 0;
  }
};


// class GeneralParser
// the GenneralParser class is a base class for parsering packets and computing points
// you can parser the udp or pcap packets using the DecodePacket fuction
// you can compute xyzi of points using the ComputeXYZI fuction, which uses cpu to compute
template <typename T_Point>
class GeneralParser {
 public:
  GeneralParser();
  virtual ~GeneralParser();

  // get lidar correction file from local file,and pass to udp parser 
  virtual void LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);

  // get lidar firetime correction file from local file,and pass to udp parser 
  virtual void LoadFiretimesFile(std::string firetimes_path);
  virtual int LoadFiretimesString(char *firetimes_string);

  // compute lidar firetime correciton
  virtual double GetFiretimesCorrection(int laserId, double speed);

  /*
    输入参数：
    distance:  udp数据中的原始距离值
    azimuth：  角度修正文件中的水平角
    elevation：角度修正文件中的俯仰角
    type:      指udp里的距离的原点，目前有两种情况，一种是udp中的距离原点在光心（optical_center),如jt,
               一种是udp的距离原点在几何中心（geometric_center)，一般是除JT外的其他雷达
    输出参数：
    distance:  对于d_center为几何中心的情况，直接输出原始的distance, 对于d_center为光心的情况，把修正后相对于几何中心的距离输出
    azimuth:   光心修正的修正量，需要加到原来的azimuth里(代替原始的角度修正文件中的水平角)
    elevation: 光心修正后的elevation
  */
  void GetDistanceCorrection(LidarOpticalCenter optical_center, int &azimuth, int &elevation, float &distance, DistanceCorrectionType type);
  void SetLidarType(std::string lidar_type);
  // covert a origin udp packet to decoded data, and pass the decoded data to a frame struct to reduce memory copy
  virtual int DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket); 
   
  // compute xyzi of points from decoded packet
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index);
  // Under thread safety, increase the points_num in the frame
  void FrameNumAdd();

  // parse the detailed content of the fault message message
  virtual void ParserFaultMessage(UdpPacket& udp_packet, FaultMessageInfo &fault_message_info);

  // set frame azimuth
  virtual void SetFrameAzimuth(float frame_start_azimuth);
  // Statistical packet loss data
  void CalPktLoss(uint32_t PacketSeqnum, FrameDecodeParam param);
  void CalPktTimeLoss(uint64_t PacketTimestamp, FrameDecodeParam param);
  int IsChannelFovFilter(int fov, int channel_index, FrameDecodeParam &param);
  // crc
  void CRCInit();
  uint32_t CRCCalc(const uint8_t *bytes, int len, int zeros_num);
  // remake 
  void setRemakeDefaultConfig(LidarDecodedFrame<T_Point> &frame);
  void DoRemake(int azi, int elev, const RemakeConfig& rq, int& point_idx);
  // update right memory space
  virtual void setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame);
  // used for updating cuda correction and firetime by itself
  uint32_t* getCorrectionLoadSequenceNum() { return &correction_load_sequence_num_; }
  uint32_t* getFiretimeLoadSequenceNum() { return &firetime_load_sequence_num_; }
  uint32_t* getDcfLoadSequenceNum() { return &dcf_load_sequence_num_; }
  bool* getCorrectionLoadFlag() { return &get_correction_file_; }
  bool* getFiretimeLoadFlag() { return &get_firetime_file_; }
  bool* getDcfLoadFlag() { return &get_dcf_file_; }
  void loadCorrectionSuccess() { get_correction_file_ = true; correction_load_sequence_num_++; }
  void loadFiretimeSuccess() { get_firetime_file_ = true; firetime_load_sequence_num_++; }
  void loadDcfSuccess() { get_dcf_file_ = true; dcf_load_sequence_num_++; }
  virtual int FrameProcess(LidarDecodedFrame<T_Point> &frame);

  //set enable_packet_loss_tool_
  virtual void EnablePacketLossTool(bool enable);
  virtual void EnablePacketTimeLossTool(bool enable);
  virtual void PacketTimeLossToolContinue(bool enable);
  void CalPktLoss(uint32_t PacketSeqnum);
  void CalPktTimeLoss(uint64_t PacketTimestamp);
  void CRCInit();
  uint32_t CRCCalc(const uint8_t *bytes, int len, int zeros_num);
  uint32_t GetComputePacketNum() { return compute_packet_num; }
  void SetComputePacketNumToZero() { compute_packet_num = 0; }
  LidarOpticalCenter GetOpticalCenter() { return optical_center; }
  void SetOpticalCenterFlag(bool flag);
  void SetXtSpotCorrection(bool flag) { xt_spot_correction = flag; }

  void TransformPoint(float& x, float& y, float& z);
  void SetTransformPara(float x, float y, float z, float roll, float pitch, float yaw);
  void EnableUpdateMonitorInfo();
  void DisableUpdateMonitorInfo();
  uint16_t *GetMonitorInfo1();
  uint16_t *GetMonitorInfo2();
  uint16_t *GetMonitorInfo3();
  std::vector<double> elevation_correction_{0};
  std::vector<double> azimuth_collection_{0};
  uint32_t total_packet_count_;
  PacketSeqnumLossMessage seqnum_loss_message_;
  PacketTimeLossMessage time_loss_message_;
  uint32_t m_CRCTable[256];

 protected:
  float cos_all_angle_[CIRCLE];
  float sin_all_angle_[CIRCLE];
  CorrectionData correction;
  float firetime_correction_[DEFAULT_MAX_LASER_NUM];
  bool get_correction_file_ = false;
  bool get_firetime_file_ = false;
  bool get_dcf_file_ = false;
  uint32_t correction_load_sequence_num_ = 0;
  uint32_t firetime_load_sequence_num_ = 0;
  uint32_t dcf_load_sequence_num_ = 0;
  int32_t last_azimuth_;
  int32_t last_last_azimuth_;
  double firetime_correction_[MAX_LASER_NUM];
  bool enable_packet_loss_tool_;
  bool enable_packet_timeloss_tool_;
  bool packet_timeloss_tool_continue_;
  std::string lidar_type_;
  Transform transform_;
  float frame_start_azimuth_;
  LidarOpticalCenter optical_center;
  std::atomic<uint32_t> compute_packet_num;
  int rotation_flag;
  bool xt_spot_correction;
};
}
}
#include "general_parser.cc"

#endif  // GENERAL_PARSER_H_
