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
#ifndef Udp1_4_PARSER_GPU_H_
#define Udp1_4_PARSER_GPU_H_
#include "general_parser_gpu.h"
#include "udp_protocol_v1_4.h"
namespace hesai
{
namespace lidar
{

int compute_1_4_cuda(uint8_t* point_cloud_cu_, CudaPointXYZAER* points_cu_, uint32_t point_cloud_size, const float* correction_azi, 
    const float* correction_ele, const pandarN::FiretimesPandarN* firetimes, const FrameDecodeParam* fParam, LidarOpticalCenter optical_center, 
    uint32_t packet_num, uint16_t block_num, uint16_t channel_num, bool is_jt128);

// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp1_4ParserGpu: public GeneralParserGpu<T_Point>{
 private:
  float* channel_azimuths_cu_;
  float* channel_elevations_cu_;
  PointDecodeData* point_data_cu_;
  uint64_t* sensor_timestamp_cu_;
 public:
  Udp1_4ParserGpu(std::string lidar_type, uint16_t maxPacket, uint16_t maxPoint) 
     : GeneralParserGpu<T_Point>(maxPacket, maxPoint) {
    lidar_type_ = lidar_type;
    if (lidar_type == STR_PANDARN) {
      this->optical_center.setNoFlag(LidarOpticalCenter{-0.012, 0.04356, 0});
    } 
    else if (lidar_type == STR_OT128) {
      this->optical_center.setNoFlag(LidarOpticalCenter{-0.01, 0.045, 0});
    } 
    /* JT128 begin */
    else if (lidar_type == STR_OTHER) {
      this->optical_center.setNoFlag(LidarOpticalCenter{-0.0076, 0.01363, 0.01271});
    }
    /* JT128 end */
    cudaSafeMalloc((void**)&pandar_firetimes_cu_, sizeof(pandarN::FiretimesPandarN));
  }
  ~Udp1_4ParserGpu() {
    cudaSafeFree(pandar_firetimes_cu_);
  }

  virtual void LoadFiretimesStruct(void *_firetimes) {
    if (this->init_suc_flag_ == false) return;
    pandar_firetimes_ptr_ = (pandarN::FiretimesPandarN*)_firetimes;
    CUDACheck(cudaMemcpy(pandar_firetimes_cu_, pandar_firetimes_ptr_, sizeof(pandarN::FiretimesPandarN), cudaMemcpyHostToDevice));
  }
  virtual void updateFiretimeFile() {
    if (this->init_suc_flag_ == false) return;
    if (*this->get_firetime_file_ && this->firetime_load_sequence_num_cuda_use_ != *this->firetime_load_sequence_num_) {
      this->firetime_load_sequence_num_cuda_use_ = *this->firetime_load_sequence_num_;
      CUDACheck(cudaMemcpy(pandar_firetimes_cu_, pandar_firetimes_ptr_, sizeof(pandarN::FiretimesPandarN), cudaMemcpyHostToDevice));
    }
  }

  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  bool corrections_loaded_;
};
}
}
#endif  // Udp1_4_PARSER_GPU_H_
