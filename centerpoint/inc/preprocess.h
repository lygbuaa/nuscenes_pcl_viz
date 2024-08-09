#ifndef __CENTERPOINT_PREPROCESS__
#define __CENTERPOINT_PREPROCESS__
#include <iostream>
#include <fstream>
#include "config.h"

#define LOGPF(format, ...) fprintf(stderr ,"[%s:%d] " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#define INPUT_POINT_STRIDE 4

void preprocess(float* points, float* feature, int* indices, int pointNum);

bool readBinFile(std::string& filename, void*& bufPtr, int& pointNum);

bool PCDTxt2Arrary(std::string file_name, void*& bufPtr, int& pointNum, int num_feature = INPUT_POINT_STRIDE);

template <typename T>
bool saveBinFile(std::string savePath, T* output, size_t shape)
{
    //Save one out node
    std::fstream file(savePath, std::ios::out | std::ios::binary);
    if (!file)
    {
        std::cout << "Error opening file." << savePath << std::endl;;
        return false;
    }
    file.write(reinterpret_cast<char *>(output), shape*sizeof(T));
    file.close();
    return true;
}
#endif