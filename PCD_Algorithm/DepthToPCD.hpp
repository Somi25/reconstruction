#pragma once
#include "defs.h"
PCDPtr DepthToPCD(cv::Mat in);
PCDPtr DepthToPCD(cv::Mat in, Intrinsic intr, Distortion dist);
PCDPtr DepthToPCD(cv::Mat in, Intrinsic intr, Distortion dist, std::string imageType, std::string coordinateSystem);