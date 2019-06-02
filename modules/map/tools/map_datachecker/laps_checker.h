
/******************************************************************************
 * Created on Sat Aug 18 2018
 *
 * Copyright (c) 2018 Baidu.com, Inc. All Rights Reserved
 *
 * @file: laps_checker.h 
 * @desc: description
 * @author: yuanyijun@baidu.com
  *****************************************************************************/

#ifndef _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LAPS_CHECKER_H
#define _MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LAPS_CHECKER_H

#include <vector>
#include <map>
#include <memory>
#include <utility>
#include "common.hpp"
#include "modules/map/tools/map_datachecker/proto/collection_error_code.pb.h"

namespace adu {
namespace workers {
namespace collection {

struct GridMeta
{
    double alpha;
    std::vector<int> idxs;
};
typedef std::vector<GridMeta> Grid;

class LapsChecker 
{

public:
    LapsChecker(std::vector<FramePose>& poses, int laps_to_check, std::shared_ptr<JSonConf> sp_conf);
    //inline std::shared_ptr<LapsChecker> get_laps_checker() { return shared_from_this();}
    ErrorCode check();
    double get_progress();
    double get_confidence();
    size_t get_lap();
    ErrorCode get_return_state();
    ~LapsChecker();

private:
    void do_check();
    int setup_grids_map();
    int check_laps();
    int check_params();
    int get_min_max();
    int do_setup_grids_map();
    double calc_alpha(int pose_index);
    int put_pose_to_grid(int pose_index, int grid_y, int grid_x);
    int put_pose_to_neighbor_grid(int pose_index);
    int get_passed_grid(int pose_index, std::vector<int>& grid_x, std::vector<int>& grid_y);
    double slope(double x1, double y1, double x2, double y2);
    int gather_timestamps(std::vector<double>& stamps, double alpha, int center_x, int center_y);
    inline int set_progress(double p);
    //debug related
    int print_grid_map_to_file();
public:
    std::vector<FramePose>& _poses;
    double _maxx, _maxy, _minx, _miny; 
//    std::shared_ptr<std::vector<std::vector<Grid>>> _grids_map;
    std::vector<std::vector<Grid>> _grids_map;
    bool finished;
private:
//    std::shared_ptr<std::vector<double>> _confidence;
    std::vector<double> _confidence;
    double _progress;
    size_t _laps_to_check;
    size_t _possible_max_laps;
    size_t _lap;
    std::shared_ptr<JSonConf> _sp_conf;
    ErrorCode _return_state;
};


} // collection
} // workers
} // adu

#endif //_MODULES_HMI_WORKERS_MAP_DATACHECKER_INCLUDE_LAPS_CHECKER_H
