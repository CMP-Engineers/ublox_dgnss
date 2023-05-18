// Copyright 2023 CMP Engineers Pty Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_INS_HPP_
#define UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_INS_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::esf::ins
{

enum class ang_rate_valid_t : u1_t {not_valid = 0, valid = 1};
enum class accel_valid_t : u1_t {not_valid = 0, valid = 1};

struct bitfield0_t
{
  union {
    x1_t all;
    struct
    {
      u1_t version : 8;
      ang_rate_valid_t xAngRateValid : 1;
      ang_rate_valid_t yAngRateValid : 1;
      ang_rate_valid_t zAngRateValid : 1;
      accel_valid_t xAccelValid : 1;
      accel_valid_t yAccelValid : 1;
      accel_valid_t zAccelValid : 1;
    } bits;
  };
};

// overload the << operator for scoped enums since they do not automatically convert to ints
std::ostream& operator<<(std::ostream& os, const ang_rate_valid_t& e) {
    os << static_cast<int>(e);
    return os;
}

std::ostream& operator<<(std::ostream& os, const accel_valid_t& e) {
    os << static_cast<int>(e);
    return os;
}

class ESFINSPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_ESF;
  static const msg_id_t MSG_ID = UBX_ESF_ALG;

  bitfield0_t bitfield0;  // version and validity flags
  u4_t reserved0;
  u4_t iTOW;       // ms - GPS Time of week of the navigation epoch
  i4_t xAngRate;   // 1e-3 deg/s Compensated x-axis angular rate
  i4_t yAngRate;   // 1e-3 deg/s Compensated y-axis angular rate
  i4_t zAngRate;   // 1e-3 deg/s Compensated z-axis angular rate
  i4_t xAccel;     // 1e-3 deg/s Compensated x-axis acceleration (gravity-free).
  i4_t yAccel;     // 1e-3 deg/s Compensated y-axis acceleration (gravity-free).
  i4_t zAccel;     // 1e-3 deg/s Compensated z-axis acceleration (gravity-free).

public:
  ESFINSPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  ESFINSPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    bitfield0 = buf_offset<bitfield0_t>(&payload_, 0);
    reserved0 = buf_offset<u4_t>(&payload_, 4);
    iTOW = buf_offset<u4_t>(&payload_, 8);
    xAngRate = buf_offset<i4_t>(&payload_, 12);
    yAngRate = buf_offset<i4_t>(&payload_, 16);
    zAngRate = buf_offset<i4_t>(&payload_, 20);
    xAccel = buf_offset<i4_t>(&payload_, 24);
    yAccel = buf_offset<i4_t>(&payload_, 28);
    zAccel = buf_offset<i4_t>(&payload_, 32);
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << bitfield0.bits.version;
    oss << " xAngRateValid: " << bitfield0.bits.xAngRateValid;
    oss << " yAngRateValid: " << bitfield0.bits.yAngRateValid;
    oss << " zAngRateValid: " << bitfield0.bits.zAngRateValid;
    oss << " xAccelValid: " << bitfield0.bits.xAccelValid;
    oss << " yAccelValid: " << bitfield0.bits.yAccelValid;
    oss << " zAccelValid: " << bitfield0.bits.zAccelValid;
    oss << " iTOW: " << iTOW;
    oss << " xAngRate: " << xAngRate;
    oss << " yAngRate: " << yAngRate;
    oss << " zAngRate: " << zAngRate;
    oss << " xAccel: " << xAccel;
    oss << " yAccel: " << yAccel;
    oss << " zAccel: " << zAccel;
    return oss.str();
  }
};
}  // namespace ubx::esf::ins
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_INS_HPP_
