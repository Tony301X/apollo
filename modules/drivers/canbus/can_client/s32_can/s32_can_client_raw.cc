/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file socket_can_client.cc
 * @brief the encapsulate call the api of s32 can card according to
 *can_client.h interface
 **/

#include "modules/drivers/canbus/can_client/s32_can/s32_can_client_raw.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool S32CanClientRaw::Init(const CANCardParameter &parameter) {
  return true;
}

S32CanClientRaw::~S32CanClientRaw() {
  if (dev_handler_) {
    Stop();
  }
}

ErrorCode S32CanClientRaw::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }
  // open device
  dev_handler_ = open("/dev/s32can0", O_RDWR);
  if (dev_handler_ < 0) {
    AERROR << "open device error code [" << dev_handler_ << "]: ";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  is_started_ = true;
  return ErrorCode::OK;
}

void S32CanClientRaw::Stop() {
  if (is_started_) {
    is_started_ = false;

    int ret = close(dev_handler_);
    if (ret < 0) {
      AERROR << "close error code:" << ret << ", " << GetErrorString(ret);
    } else {
      AINFO << "close s32 can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode S32CanClientRaw::Send(const std::vector<CanFrame> &frames,
                                   int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "Nvidia can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    if (frames[i].len != CANBUS_MESSAGE_LENGTH) {
      AERROR << "frames[" << i << "].len = " << frames[i].len
             << ", which is not equal to can message data length ("
             << CANBUS_MESSAGE_LENGTH << ").";
      return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
    }
    send_frames_[i].can_id = frames[i].id;
    send_frames_[i].can_dlc = frames[i].len;
    std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);

    // Synchronous transmission of CAN messages
    int ret = static_cast<int>(
        write(dev_handler_, &send_frames_[i], sizeof(send_frames_[i])));
    if (ret <= 0) {
      AERROR << "send message failed, error code: " << ret;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
  }

  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode S32CanClientRaw::Receive(std::vector<CanFrame> *const frames,
                                      int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Nvidia can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    auto ret = read(dev_handler_, &recv_frames_[i], sizeof(recv_frames_[i]));

    if (ret < 0) {
      AERROR << "receive message failed, error code: " << ret;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
    if (recv_frames_[i].can_dlc != CANBUS_MESSAGE_LENGTH) {
      AERROR << "recv_frames_[" << i
             << "].can_dlc = " << recv_frames_[i].can_dlc
             << ", which is not equal to can message data length ("
             << CANBUS_MESSAGE_LENGTH << ").";
      return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
    }
    cf.id = recv_frames_[i].can_id;
    cf.len = recv_frames_[i].can_dlc;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].can_dlc);
    frames->push_back(cf);
  }
  return ErrorCode::OK;
}

std::string S32CanClientRaw::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
