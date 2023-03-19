// Copyright 2023 Australian Robotics Supplies & Technology
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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_ESF_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_ESF_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
#include "ublox_dgnss_node/ubx/esf/ubx_esf_status.hpp"

namespace ubx::esf
{

typedef UBXFrameComms<esf::status::ESFStatusPayload, usb::Connection> UbxESFStatusFrameComms;

class UbxEsf
{
private:
  std::shared_ptr<usb::Connection> usbc_;
  std::shared_ptr<UbxESFStatusFrameComms> status_;

public:
  explicit UbxEsf(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    status_ = std::make_shared<UbxESFStatusFrameComms>(usbc_);
  }
  std::shared_ptr<UbxESFStatusFrameComms> status()
  {
    return status_;
  }
};
}  // namespace ubx::esf
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_ESF_HPP_