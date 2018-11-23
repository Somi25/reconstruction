#pragma once
#include "defs.h"
//Z
PCDPtr DepthToPCD(cv::Mat in);
PCDPtr DepthToPCD(cv::Mat in, Intrinsic intr, Distortion dist);
PCDPtr DepthToPCD(cv::Mat in, Intrinsic intr, Distortion dist, std::string imageType, std::string coordinateSystem);

//O
PCDPtr DepthToPCD(cv::Mat in, int depth_unit);
PCDPtr DepthToPCD(cv::Mat in, int depth_unit, float v_viewing_angle, float h_viewing_angle);