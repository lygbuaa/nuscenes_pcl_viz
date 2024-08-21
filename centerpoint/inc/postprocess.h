#ifndef __CENTERPOINT_POSTPROCESS__
#define __CENTERPOINT_POSTPROCESS__

#include "buffers.h"
#include "common.h"
#include "config.h"
#include "preprocess.h"

struct DetBox3d_t{
    float x;
    float y;
    float z;
    float l;
    float h;
    float w;
    float velX;
    float velY;
    float theta;

    float score;
    int cls;
    bool isDrop; // for nms
};

void postprocess(const samplesCommon::BufferManager& buffers, std::vector<DetBox3d_t>& predResult);
#endif