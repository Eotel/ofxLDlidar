/**
 * @file cmd_interface_unix.cpp
 * @author LDRobot (support@ldrobot.com)
 * @author eotel
 * @brief  unix-based (Linux and macOS) serial port App
 *         Modified from cmd_interface_linux.cpp to support macOS
 * @version 0.2
 * @date 2024-09-27
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights reserved.
 * @copyright Copyright (c) 2024  eotel
 *
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "serial_interface_unix.h"
#include "log_module.h"

#define MAX_ACK_BUF_LEN 4096

namespace ldlidar {

SerialInterfaceUnix::SerialInterfaceUnix()
    : rx_thread_(nullptr), rx_count_(0), read_callback_(nullptr) {
  com_handle_ = -1;
  com_baudrate_ = 0;
}

SerialInterfaceUnix::~SerialInterfaceUnix() { Close(); }

  bool SerialInterfaceUnix::Open(std::string &port_name, uint32_t com_baudrate) {
  int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

  com_handle_ = open(port_name.c_str(), flags);
  if (-1 == com_handle_) {
    LOG_ERROR("Open open error,%s", strerror(errno));
    return false;
  }

  com_baudrate_ = com_baudrate;

  struct termios options;
  if (ioctl(com_handle_, IOSSIOSPEED, &com_baudrate_) == -1) {
    LOG_ERROR("IOSSIOSPEED error: %s", strerror(errno));
    close(com_handle_);
    com_handle_ = -1;
    return false;
  }

  tcgetattr(com_handle_, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  tcsetattr(com_handle_, TCSANOW, &options);

  tcflush(com_handle_, TCIFLUSH);

  rx_thread_exit_flag_ = false;
  rx_thread_ = new std::thread(RxThreadProc, this);
  is_cmd_opened_ = true;

  return true;
}

bool SerialInterfaceUnix::Close() {
  if (!is_cmd_opened_) {
    return true;
  }

  rx_thread_exit_flag_ = true;

  if (com_handle_ != -1) {
    close(com_handle_);
    com_handle_ = -1;
  }

  if ((rx_thread_ != nullptr) && rx_thread_->joinable()) {
    rx_thread_->join();
    delete rx_thread_;
    rx_thread_ = nullptr;
  }

  is_cmd_opened_ = false;

  return true;
}

bool SerialInterfaceUnix::ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len,
                                   uint32_t *rx_len) {
  static timespec timeout = {0, (long)(100 * 1e6)};
  int32_t len = -1;

  if (IsOpened()) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(com_handle_, &read_fds);
    int r = pselect(com_handle_ + 1, &read_fds, NULL, NULL, &timeout, NULL);
    if (r < 0) {
      // Select was interrupted
      if (errno == EINTR) {
        return false;
      }
    } else if (r == 0) {  // timeout
      return false;
    }

    if (FD_ISSET(com_handle_, &read_fds)) {
      len = (int32_t)read(com_handle_, rx_buf, rx_buf_len);
      if ((len != -1) && rx_len) {
        *rx_len = len;
      }
    }
  }
  return len == -1 ? false : true;
}

bool SerialInterfaceUnix::WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len,
                                  uint32_t *tx_len) {
  int32_t len = -1;

  if (IsOpened()) {
    len = (int32_t)write(com_handle_, tx_buf, tx_buf_len);
    if ((len != -1) && tx_len) {
      *tx_len = len;
    }
  }
  return len == -1 ? false : true;
}

void SerialInterfaceUnix::RxThreadProc(void *param) {
  SerialInterfaceUnix *cmd_if = (SerialInterfaceUnix *)param;
  char *rx_buf = new char[MAX_ACK_BUF_LEN + 1];
  while (!cmd_if->rx_thread_exit_flag_.load()) {
    uint32_t readed = 0;
    bool res = cmd_if->ReadFromIO((uint8_t *)rx_buf, MAX_ACK_BUF_LEN, &readed);
    if (res && readed) {
      cmd_if->rx_count_ += readed;
      if (cmd_if->read_callback_ != nullptr) {
        cmd_if->read_callback_(rx_buf, readed);
      }
    }
  }

  delete[] rx_buf;
}

} // namespace ldlidar

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/