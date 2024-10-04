//
// Created by Daiki Miura on 2024/09/27.
//

#ifndef API_H
#define API_H

#include <sensor_msgs/LaserScan.h>
#include <string>

struct LaserScanSetting
{
    std::string frame_id;
    bool laser_scan_dir;
    bool enable_angle_crop_func;
    double angle_crop_min;
    double angle_crop_max;
};

#endif //API_H
