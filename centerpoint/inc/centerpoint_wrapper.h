#ifndef __CENTERPOINT_WRAPPER_H__
#define __CENTERPOINT_WRAPPER_H__

#include "preprocess.h"
#include "postprocess.h"

bool load_centerpoint_model(const std::string& trt_model_path);

bool infer_centerpoint_model(float* points, const int pointNum, std::vector<Box>& predResult);

#endif