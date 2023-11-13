/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim, Autonomous Systems Lab,
 *  ETH Zürich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Performance Tracker for planner
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <any>
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <vector>

class DataLogger {
 public:
  DataLogger();
  virtual ~DataLogger();
  std::vector<std::string> getKeys() { return keys_; };
  void setPrintHeader(bool header) { print_header_ = header; };
  void addKey(const std::string key) { keys_.push_back(key); };
  void setKeys(const std::vector<std::string> keys) { keys_ = keys; };
  int count() { return data_list_.size(); };
  std::vector<std::unordered_map<std::string, std::any>> data() { return data_list_; };
  void setSeparator(const std::string separator) { field_seperator = separator; };
  void record(const std::unordered_map<std::string, std::any> data);
  void writeToFile(const std::string path);

 private:
  int id_{0};
  std::vector<std::string> keys_;
  std::vector<std::unordered_map<std::string, std::any>> data_list_;
  bool print_header_{false};
  std::string field_seperator{","};
};

#endif
