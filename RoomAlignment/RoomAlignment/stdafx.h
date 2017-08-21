/*

Copyright(C) 2017 Julian Löhr
All rights reserved.

Licensed under:
MIT License

*/
#pragma once

#include "targetver.h"

#include <iostream>

#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include <functional>

#include <array>
#include <map>
#include <vector>

#include <mosquitto.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Windows.h>

#include "json.hpp"
#include "PCLUtil.h"
