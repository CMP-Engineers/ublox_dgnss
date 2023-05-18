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

#ifndef UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_ALG_HPP_
#define UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_ALG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::esf::alg
{

enum class auto_mnt_alg_on_t : u1_t {not_running = 0, running = 1};
enum class mnt_alg_status_t : u1_t {usr_dfnd_angles = 0, roll_pitch_ongoing = 1, roll_pitch_yaw_ongoing = 2,
  coarse_alg = 3, fine_alg = 4};

struct flags_t
{
  union {
    x1_t all;
    struct
    {
      auto_mnt_alg_on_t autoMntAlgOn : 1;
      mnt_alg_status_t status : 3;
    } bits;
  };
};

// overload the << operator for scoped enums since they do not automatically convert to ints
std::ostream& operator<<(std::ostream& os, const auto_mnt_alg_on_t& e) {
    os << static_cast<int>(e);
    return os;
}

std::ostream& operator<<(std::ostream& os, const mnt_alg_status_t& e) {
    os << static_cast<int>(e);
    return os;
}

enum class tilt_alg_err_t : u1_t {no_error = 0, error = 1};
enum class yaw_alg_err_t : u1_t {no_error = 0, error = 1};
enum class euler_alg_err_t : u1_t {no_error = 0, error = 1};

struct error_t
{
  union {
    x1_t all;
    struct
    {
      tilt_alg_err_t tiltAlgError : 1;
      yaw_alg_err_t yawAlgError : 1;
      euler_alg_err_t angleError: 1;
    } bits;
  };
};

std::ostream& operator<<(std::ostream& os, const tilt_alg_err_t& e) {
    os << static_cast<int>(e);
    return os;
}

std::ostream& operator<<(std::ostream& os, const yaw_alg_err_t& e) {
    os << static_cast<int>(e);
    return os;
}

std::ostream& operator<<(std::ostream& os, const euler_alg_err_t& e) {
    os << static_cast<int>(e);
    return os;
}

class ESFALGPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_ESF;
  static const msg_id_t MSG_ID = UBX_ESF_ALG;

  u4_t iTOW;        // ms - GPS Time of week of the navigation epoch.
  u1_t version;     // message version (0x02 for this version)
  flags_t flags;
  error_t error;
  u1_t reserved0;
  u4_t yaw;
  i2_t pitch;
  i2_t roll;

public:
  ESFALGPayload()
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
  }
  ESFALGPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    iTOW = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_, 4);
    flags = buf_offset<flags_t>(&payload_, 5);
    error = buf_offset<error_t>(&payload_, 6);
    reserved0 = buf_offset<u1_t>(&payload_, 7);
    yaw = buf_offset<u4_t>(&payload_, 8);
    pitch = buf_offset<i2_t>(&payload_, 12);
    roll = buf_offset<i2_t>(&payload_, 14);

  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "iTOW: " << iTOW;
    oss << " version: " << version; // unary plus '+' forces character to be treated as an integer rather than ASCII representation
    oss << " autoMntAlgOn: " << flags.bits.autoMntAlgOn;
    oss << " status: " << flags.bits.status;
    oss << " tiltAlgError: " << error.bits.tiltAlgError;
    oss << " yawAlgError: " << error.bits.yawAlgError;
    oss << " angleError: " << error.bits.angleError;
    oss << " yaw: " << yaw;
    oss << " pitch: " << pitch;
    oss << " roll: " << roll;
    return oss.str();
  }
};
}  // namespace ubx::esf::alg
#endif  // UBLOX_DGNSS_NODE__UBX__ESF__UBX_ESF_ALG_HPP_
