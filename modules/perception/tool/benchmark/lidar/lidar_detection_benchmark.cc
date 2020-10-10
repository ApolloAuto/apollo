/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <boost/program_options.hpp>

#include "modules/perception/tool/benchmark/lidar/detection_evaluation.h"

int main(int argc, char** argv) {
  boost::program_options::options_description boost_desc("Allowed options");
  boost_desc.add_options()("help", "produce help message")(
      "cloud", boost::program_options::value<std::string>(),
      "provide the point cloud folder or list")(
      "result", boost::program_options::value<std::string>(),
      "provide the result folder or list")(
      "groundtruth", boost::program_options::value<std::string>(),
      "provide the groundtruth folder or list")(
      "is_folder", boost::program_options::value<bool>(),
      "provide folder or list")(
      "loading_thread_num",
      boost::program_options::value<unsigned int>()->default_value(4),
      "thread number for loading data")(
      "evaluation_thread_num",
      boost::program_options::value<unsigned int>()->default_value(4),
      "thread number for evaluation")(
      "parallel_processing_num",
      boost::program_options::value<unsigned int>()->default_value(4),
      "parallel processing number for each time")(
      "reserve",
      boost::program_options::value<std::string>()->default_value(""));

  boost::program_options::variables_map boost_args;
  boost::program_options::store(
      boost::program_options::parse_command_line(argc, argv, boost_desc),
      boost_args);
  boost::program_options::notify(boost_args);

  if (boost_args.count("help") || !boost_args.count("cloud") ||
      !boost_args.count("result") || !boost_args.count("groundtruth") ||
      !boost_args.count("is_folder")) {
    std::cout << boost_desc << std::endl;
    return 0;
  }

  const std::string cloud = boost_args["cloud"].as<std::string>();
  const std::string result = boost_args["result"].as<std::string>();
  const std::string groundtruth = boost_args["groundtruth"].as<std::string>();
  bool is_folder = boost_args["is_folder"].as<bool>();
  unsigned int loading_thread_num =
      boost_args["loading_thread_num"].as<unsigned int>();
  unsigned int evaluation_thread_num =
      boost_args["evaluation_thread_num"].as<unsigned int>();
  unsigned int parallel_processing_num =
      boost_args["parallel_processing_num"].as<unsigned int>();
  const std::string reserve = boost_args["reserve"].as<std::string>();

  apollo::perception::benchmark::DetectionEvaluation evaluator;
  evaluator.init(cloud, result, groundtruth, is_folder, loading_thread_num,
                 evaluation_thread_num, parallel_processing_num, reserve);

  evaluator.run_evaluation();

  std::cerr << evaluator << std::endl;

  return 0;
}
