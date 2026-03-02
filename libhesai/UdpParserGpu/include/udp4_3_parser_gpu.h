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
#ifndef Udp4_3_PARSER_GPU_H_
#define Udp4_3_PARSER_GPU_H_
#include "general_parser_gpu.h"
#include "udp_protocol_v4_3.h"
namespace hesai
{
namespace lidar
{

// class Udp4_3ParserGpu
// computes points for PandarAT128
// you can compute xyzi of points using the ComputeXYZI fuction, which uses gpu to compute
template <typename T_Point>
class Udp4_3ParserGpu: public GeneralParserGpu<T_Point>{
 private:
  // corrections
  bool corrections_loaded_;
  int32_t* channel_azimuths_cu_;
  int32_t* channel_elevations_cu_;
  int8_t* dazis_cu;
  int8_t* deles_cu;
  PointDecodeData* point_data_cu_;
  uint64_t* sensor_timestamp_cu_;
  uint32_t* mirror_azi_begins_cu;
  uint32_t* mirror_azi_ends_cu;

 public:
  Udp4_3ParserGpu(uint16_t maxPacket, uint16_t maxPoint) 
     : GeneralParserGpu<T_Point>(maxPacket, maxPoint) {
    cudaSafeMalloc((void**)&AT_correction_cu_, sizeof(AT::ATCorrectionFloat));
  }
  ~Udp4_3ParserGpu() {
    cudaSafeFree(AT_correction_cu_);
  }

  virtual void LoadCorrectionStruct(void *_correction) {
    if (this->init_suc_flag_ == false) return;
    AT_correction_ptr = (AT::ATCorrections*)_correction;
    CUDACheck(cudaMemcpy(AT_correction_cu_, &AT_correction_ptr->floatCorr, sizeof(AT::ATCorrectionFloat), cudaMemcpyHostToDevice));
  }
  virtual void LoadFiretimesStruct(void *) {}
  virtual void updateCorrectionFile() {
    if (this->init_suc_flag_ == false) return;
    if (*this->get_correction_file_ && this->correction_load_sequence_num_cuda_use_ != *this->correction_load_sequence_num_) {
      this->correction_load_sequence_num_cuda_use_ = *this->correction_load_sequence_num_;
      CUDACheck(cudaMemcpy(AT_correction_cu_, &AT_correction_ptr->floatCorr, sizeof(AT::ATCorrectionFloat), cudaMemcpyHostToDevice));
    }
  }
  virtual void updateFiretimeFile() {}

  // compute xyzi of points from decoded packet， use gpu device
  // param packet is the decoded packet; xyzi of points after computed is puted in frame  
  virtual int ComputeXYZI(LidarDecodedFrame<T_Point> &frame);
  virtual int LoadCorrectionFile(std::string correction_path);
  virtual int LoadCorrectionString(char *correction_string);
  PandarATCorrections m_PandarAT_corrections;

};
}
}
#endif  // Udp4_3_PARSER_GPU_H_