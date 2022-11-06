/****************************************************************************
 *
 *   Copyright (c) 2021 Jaeyoung Lim. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 * @brief Performance Tracker
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
#include "terrain_navigation/data_logger.h"
#include <fstream>
#include <iostream>

DataLogger::DataLogger() {}

DataLogger::~DataLogger() {}

void DataLogger::record(const std::unordered_map<std::string, std::any> data) { data_list_.push_back(data); }

void DataLogger::writeToFile(const std::string path) {
  // Write data to files
  std::cout << "[DataLogger] Writing data to file! " << path << std::endl;
  std::ofstream output_file;
  output_file.open(path, std::ios::app);
  if (print_header_) {
    for (auto key : keys_) {
      output_file << key << field_seperator;
    }
    output_file << "\n";
  }

  for (auto data : data_list_) {
    for (auto key : keys_) {
      if (std::string* s = std::any_cast<std::string>(&data.at(key))) {
        output_file << std::any_cast<std::string&>(data.at(key)) << field_seperator;
      } else if (double* i = std::any_cast<double>(&data.at(key))) {
        output_file << std::any_cast<double&>(data.at(key)) << field_seperator;
      }
    }
    output_file << "\n";
  }

  output_file.close();
  return;
}
