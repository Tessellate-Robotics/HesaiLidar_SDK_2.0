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
 * File:       udp7_2_parser.cc
 * Author:     Zhang Yu <zhangyu@hesaitech.com>
 * Description: Implemente Udp7_2Parser class
*/

#include "udp7_2_parser.h"
using namespace hesai::lidar;
template<typename T_Point>
Udp7_2Parser<T_Point>::Udp7_2Parser() {
  this->last_cloumn_id_ = -1;
  this->default_remake_config.min_azi = 40.0f;
  this->default_remake_config.max_azi = 140.0f;
  this->default_remake_config.ring_azi_resolution = 0.625f;
  this->default_remake_config.max_azi_scan = 160;   // (max_azi - min_azi) / ring_azi_resolution
  this->default_remake_config.min_elev = -37.5f;
  this->default_remake_config.max_elev = 37.5f;
  this->default_remake_config.ring_elev_resolution = 0.625f;
  this->default_remake_config.max_elev_scan = 120;   // (max_elev - min_elev) / ring_elev_resolution
  LogInfo("init 7_2 parser");
}
template<typename T_Point>
Udp7_2Parser<T_Point>::~Udp7_2Parser() { LogInfo("release Udp7_2Parser "); }

template <typename T_Point>
void Udp7_2Parser<T_Point>::setFrameRightMemorySpace(LidarDecodedFrame<T_Point> &frame) {
  frame.resetMalloc(2000, 128);
}

template<typename T_Point>
void Udp7_2Parser<T_Point>::LoadCorrectionFile(std::string correction_path) {
  LogInfo("load correction file from local correction.csv now!");
  std::ifstream fin(correction_path);
  if (fin.is_open()) {
    LogDebug("Open correction file success");
    int length = 0;
    fin.seekg(0, std::ios::end);
    length = static_cast<int>(fin.tellg());
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    int ret = LoadCorrectionString(buffer);
    delete[] buffer;
    if (ret != 0) {
      LogError("Parse local Correction file Error");
    } else {
      LogInfo("Parse local Correction file Success!!!");
    }
  } else {
    LogError("Open correction file failed");
    return;
  }
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionString(const char *correction_string, int len) {
  if (LoadCorrectionDatData(correction_string, len) == 0) {
    return 0;
  }
  return LoadCorrectionCsvData(correction_string, len);
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionCsvData(char *correction_string) {
  std::istringstream ifs(correction_string);
	std::string line;
  // first line "Laser id,Elevation,Azimuth"
	if(std::getline(ifs, line)) {  
		LogInfo("Parse Lidar Correction...");
	}
	int lineCounter = 0;
	std::vector<std::string>  firstLine;
  split_string(firstLine, line, ',');
  while (std::getline(ifs, line)) {
    if(line.length() < strlen("1,1,1,1")) {
      return -1;
    } 
    else {
      lineCounter++;
    }
    float elev, azimuth;
    int lineId = 0;
    int columnId = 0;
    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> columnId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;
    if (lineId > CHANNEL_MAX || lineId <= 0 || columnId > COLUMN_MAX || columnId <= 0){
      LogError("data error, lineId:%d, columnId:%d", lineId, columnId);
      continue;
    }
    corrections_.elevations[lineId - 1][columnId - 1] = elev * kResolutionInt;
    corrections_.azimuths[lineId - 1][columnId - 1] = azimuth * kResolutionInt;
  }
  this->get_correction_file_ = true;
	return 0;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionDatData(char *correction_string) {

  try {
    corrections_.clear();
    std::string correction_string_str(correction_string, len);
    std::istringstream ifs(correction_string_str);
    std::string line;
    // first line "Laser id,Elevation,Azimuth"
    if(std::getline(ifs, line)) {  
      LogInfo("Parse Lidar Correction...");
    }
    int lineCounter = 0;
    std::vector<std::string>  firstLine;
    split_string(firstLine, line, ',');
    while (std::getline(ifs, line)) {
      if(line.length() < strlen("1,1,1,1")) {
        return -1;
      } 
      else {
        lineCounter++;
      }
      float elev, azimuth;
      int lineId = 0;
      int columnId = 0;
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> lineId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> columnId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> elev;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> azimuth;
      if (lineId > FT::CHANNEL_MAX || lineId <= 0 || columnId > FT::COLUMN_MAX || columnId <= 0){
        throw std::invalid_argument("data error, lineId or columnId out of range");
      }
      corrections_.elevations[(lineId - 1) * FT::FT2_CORRECTION_LEN + columnId - 1] = elev;
      corrections_.azimuths[(lineId - 1) * FT::FT2_CORRECTION_LEN + columnId - 1] = azimuth;
    }
    this->loadCorrectionSuccess();
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return 0;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::LoadCorrectionDatData(const char *data, int len) {
  try {
    if (len < 32) {
      throw std::invalid_argument("correction string length is too short");
    }
    const char *p = data;
    FT::PandarFTCorrectionsHeader header = *(FT::PandarFTCorrectionsHeader *)p;
    if (0xee == header.pilot[0] && 0xff == header.pilot[1]) {
      switch (header.version[1]) {
        case 0: {
          corrections_.header = header;
          p += sizeof(FT::PandarFTCorrectionsHeader);
          corrections_.column_number = *((uint8_t *)p);
          p++;
          corrections_.channel_number = *((uint8_t *)p);
          p++;
          corrections_.resolution = *((uint8_t *)p);
          p++;
          int doubleAngleNum = corrections_.column_number * corrections_.channel_number * 2;
          if (static_cast<size_t>(len) < 9 + sizeof(int16_t) * doubleAngleNum + 32) {
            throw std::invalid_argument("correction string length is too short");
          }
          memcpy((void*)corrections_.angles, p, sizeof(int16_t) * doubleAngleNum);
          p += sizeof(int16_t) * doubleAngleNum;
          memcpy((void*)corrections_.hashValue, p, 32);
          for (int col = 0; col < corrections_.column_number; col++) {
              for (int i = 0; i < corrections_.channel_number; i++) {
                  int idx = col * corrections_.channel_number + i;
                  corrections_.azimuths[i * FT::FT2_CORRECTION_LEN + col] = 1.f * corrections_.angles[idx] * corrections_.resolution * 0.01;
              }
          }
          for (int col = 0; col < corrections_.column_number; col++) {
              for (int i = 0; i < corrections_.channel_number; i++) {
                  int idx = doubleAngleNum / 2 + col * corrections_.channel_number + i;
                  corrections_.elevations[i * FT::FT2_CORRECTION_LEN + col] = 1.f * corrections_.angles[idx] * corrections_.resolution * 0.01;
              }
          }
          this->get_correction_file_ = true;
          delete[] angles;
          delete[] hashValue;
          return 0;
        } break;
        case 1: {
          corrections_.header = header;
          p += sizeof(FT::PandarFTCorrectionsHeader);
          corrections_.column_number = *((uint8_t *)p);
          p++;
          corrections_.channel_number = *((uint8_t *)p);
          p++;
          corrections_.resolution = *((uint8_t *)p);
          p++;
          int doubleAngleNum = corrections_.column_number * corrections_.channel_number * 2;
          if (static_cast<size_t>(len) < 9 + sizeof(int32_t) * doubleAngleNum + 32) {
            throw std::invalid_argument("correction string length is too short");
          }
          memcpy((void*)corrections_.angles_32, p, sizeof(int32_t) * doubleAngleNum);
          p += sizeof(int32_t) * doubleAngleNum;
          memcpy((void*)corrections_.hashValue, p, 32);
          for (int col = 0; col < corrections_.column_number; col++) {
              for (int i = 0; i < corrections_.channel_number; i++) {
                  int idx = col * corrections_.channel_number + i;
                  corrections_.azimuths[i * FT::FT2_CORRECTION_LEN + col] = 1.f * corrections_.angles_32[idx] * corrections_.resolution * 0.01;
              }
          }
          for (int col = 0; col < corrections_.column_number; col++) {
              for (int i = 0; i < corrections_.channel_number; i++) {
                  int idx = doubleAngleNum / 2 + col * corrections_.channel_number + i;
                  corrections_.elevations[i * FT::FT2_CORRECTION_LEN + col] = 1.f * corrections_.angles_32[idx] * corrections_.resolution * 0.01;
              }
          }
          this->get_correction_file_ = true;
          delete[] angles;
          delete[] hashValue;
          return 0;
        } break;
        default:
          break;
      }
    }
    return -1;
  } catch (const std::exception &e) {
    LogFatal("load correction error: %s", e.what());
    return -1;
  }
  return -1;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::ComputeXYZI(LidarDecodedFrame<T_Point> &frame, int packet_index) {
  // T_Point point;
  for (int i = 0; i < frame.laser_num; i++) {
    int point_index = packet_index * frame.per_points_num + i;
    float distance = static_cast<float>(frame.pointData[point_index].distances * frame.distance_unit);
    int azimuth = 0;
    int elevation = 0;   
    if (this->get_correction_file_) {
      azimuth = frame.pointData[point_index].azimuth * kFineResolutionFloat;
      elevation = frame.pointData[point_index].elevation * kFineResolutionFloat;
      elevation = (CIRCLE + elevation) % CIRCLE;
      azimuth = (CIRCLE + azimuth) % CIRCLE;
    }
    if (frame.config.fov_start != -1 && frame.config.fov_end != -1)
    {
      int fov_transfer = azimuth / 256 / 100;
      if (fov_transfer < frame.config.fov_start || fov_transfer > frame.config.fov_end){//不在fov范围continue
          memset(&frame.points[point_index], 0, sizeof(T_Point));
        continue;
      }
    }           
    float xyDistance = distance * this->cos_all_angle_[(elevation)];
    float x = xyDistance * this->sin_all_angle_[(azimuth)];
    float y = xyDistance * this->cos_all_angle_[(azimuth)];
    float z = distance * this->sin_all_angle_[(elevation)];
    this->TransformPoint(x, y, z);
    setX(frame.points[point_index], x);
    setY(frame.points[point_index], y);
    setZ(frame.points[point_index], z);
    setIntensity(frame.points[point_index], frame.pointData[point_index].reflectivities);
    setTimestamp(frame.points[point_index], double(frame.sensor_timestamp[packet_index]) / kMicrosecondToSecond);
    setRing(frame.points[point_index], static_cast<uint16_t>(i));
  }
  GeneralParser<T_Point>::FrameNumAdd();
  return 0;
}

template<typename T_Point>
bool Udp7_2Parser<T_Point>::IsNeedFrameSplit(uint16_t column_id, uint16_t total_column) {
  (void)total_column;
  if (column_id < this->last_cloumn_id_) {
      return true;
    }
  return false;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket)
{
  if (!this->get_correction_file_) {
    static bool printErrorBool = true;
    if (printErrorBool) {
      LogInfo("No available angle calibration files, prohibit parsing of point cloud packages");
      printErrorBool = false;
    }
    return -1;
  }
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ) {
    return -1;
  }
  const HS_LIDAR_HEADER_FT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_FT_V2 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));

  const HS_LIDAR_TAIL_FT_V2 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_FT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V2) +
          (sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V2) * pHeader->GetChannelNum()));  

  if (frame.use_timestamp_type == 0) {
    frame.sensor_timestamp[frame.packet_num] = pTail->GetMicroLidarTimeU64();
  } else {
    frame.sensor_timestamp[frame.packet_num] = udpPacket.recv_timestamp;
  }
  uint32_t packet_seqnum = pTail->sequence_num;
  this->CalPktLoss(packet_seqnum);
  uint64_t packet_timestamp = pTail->GetMicroLidarTimeU64();
  this->CalPktTimeLoss(packet_timestamp);   
  frame.host_timestamp = GetMicroTickCountU64();
  frame.per_points_num = pHeader->GetChannelNum();
  frame.scan_complete = false;
  frame.distance_unit = pHeader->GetDistUnit();
  int index = frame.packet_num * pHeader->GetChannelNum();
  frame.block_num = 1;
  frame.laser_num = pHeader->GetChannelNum();

  const HS_LIDAR_BODY_CHN_UNIT_FT_V2 *pChnUnit =
      reinterpret_cast<const HS_LIDAR_BODY_CHN_UNIT_FT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V2));
  for (int blockid = 0; blockid < 1; blockid++) {
    for (int i = 0; i < pHeader->GetChannelNum(); i++) {
      frame.pointData[index].azimuth = corrections_.azimuths[i][pTail->column_id];
      frame.pointData[index].elevation = corrections_.elevations[i][pTail->column_id];
      frame.pointData[index].reflectivities = pChnUnit->GetReflectivity();  
      frame.pointData[index].distances = pChnUnit->GetDistance();
      pChnUnit = pChnUnit + 1;
      index++;
    }
  }
  if (IsNeedFrameSplit(pTail->column_id, pHeader->total_column)) {
    frame.scan_complete = true;
  }
  this->last_cloumn_id_ = pTail->column_id;
  frame.packet_num++;
  return 0;
}

template<typename T_Point>
int Udp7_2Parser<T_Point>::DecodePacket(LidarDecodedFrame<T_Point> &frame, const UdpPacket& udpPacket, const int packet_index) 
{
  uint32_t packet_index_use = (packet_index >= 0 ? packet_index : frame.packet_num);
  if (udpPacket.buffer[0] != 0xEE || udpPacket.buffer[1] != 0xFF ||
      udpPacket.buffer[2] != 7 || udpPacket.buffer[3] != 2) {
    LogDebug("Invalid point cloud");
    return -1;
  }
  frame.scan_complete = false;

  const HS_LIDAR_HEADER_FT_V2 *pHeader =
      reinterpret_cast<const HS_LIDAR_HEADER_FT_V2 *>(
          &(udpPacket.buffer[0]) + sizeof(HS_LIDAR_PRE_HEADER));
  if (frame.frame_init_ == false) {
    frame.block_num = 1;
    frame.laser_num = pHeader->GetChannelNum();
    frame.per_points_num = pHeader->GetChannelNum();
    frame.distance_unit = pHeader->GetDistUnit();
    if (frame.per_points_num > frame.maxPointPerPacket) {
      LogFatal("per_points_num(%u) out of %d", frame.per_points_num, frame.maxPointPerPacket);
      return -1;
    }
    if (frame.laser_num > FT::CHANNEL_MAX) {
      LogFatal("laser_num(%u) out of %d", frame.laser_num, FT::CHANNEL_MAX);
      return -1;
    }
    frame.frame_init_ = true;
  }
  if (pHeader->GetChannelNum() != frame.laser_num
      || pHeader->GetDistUnit() != frame.distance_unit) {
    LogFatal("block_num or laser_num or distance_unit is not match");
    return -1;
  }
  const HS_LIDAR_TAIL_FT_V2 *pTail =
      reinterpret_cast<const HS_LIDAR_TAIL_FT_V2 *>(
          (const unsigned char *)pHeader + sizeof(HS_LIDAR_HEADER_FT_V2) +
          (sizeof(HS_LIDAR_BODY_CHN_UNIT_FT_V2) * pHeader->GetChannelNum()));  
  
  if (IsNeedFrameSplit(pTail->column_id)) {
    frame.scan_complete = true;
  }
  this->last_cloumn_id_ = pTail->column_id;
  if (frame.scan_complete)
    return 0;
    
  this->CalPktLoss(pTail->GetSeqNum(), frame.fParam);
  this->CalPktTimeLoss(pTail->GetMicroLidarTimeU64(this->last_utc_time), frame.fParam);

  if (frame.fParam.use_timestamp_type == 0) {
    frame.packetData[packet_index_use].t.sensor_timestamp = pTail->GetMicroLidarTimeU64(this->last_utc_time);
  } else {
    frame.packetData[packet_index_use].t.sensor_timestamp = udpPacket.recv_timestamp;
  }   
  if (frame.frame_start_timestamp == 0) frame.frame_start_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
  frame.frame_end_timestamp = double(frame.packetData[packet_index_use].t.sensor_timestamp) / kMicrosecondToSecond;
  
  frame.return_mode = pTail->GetReturnMode();
 
  auto packet_size = udpPacket.packet_len;
  if (this->last_max_packet_num_ != frame.maxPacketPerFrame) {
    this->last_max_packet_num_ = frame.maxPacketPerFrame;
    if (frame.point_cloud_raw_data != nullptr) delete[] frame.point_cloud_raw_data;
    frame.point_cloud_size = packet_size;
    frame.point_cloud_raw_data = new uint8_t[frame.point_cloud_size * frame.maxPacketPerFrame];
    memset(frame.point_cloud_raw_data, 0, frame.point_cloud_size * frame.maxPacketPerFrame);
  }
  if (frame.point_cloud_size != packet_size) {
    LogFatal("point cloud size is should be %d, but is %d", frame.point_cloud_size, packet_size);
    return -1;
  }
  memcpy(frame.point_cloud_raw_data + packet_index_use * frame.point_cloud_size, udpPacket.buffer, packet_size);
  frame.packet_num++;
  return 0;
}

