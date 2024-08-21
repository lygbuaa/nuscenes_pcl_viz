#ifndef __BASIC_UTILS__
#define __BASIC_UTILS__

#include <cmath>
#include "logging_utils.h"

#ifndef M_PI
#define M_PI 3.14159265f
#endif
static constexpr float DEG2RAD_ = 180.0f / M_PI;

class BasicUtils
{
public:
    /** normalize yaw into (-pi, pi] */
    static float NormalizeYawRad(float yaw_rad)
    {
        return (yaw_rad - ceil(yaw_rad / (2 * M_PI) - 0.5) * M_PI * 2);
    }

    static float limit_inverse_trigo_value(float val)
    {
        if(val > (1.0f - 1e-4))
        {
            val = (1.0f - 1e-4);
        }
        else if(val < (1e-4 - 1.0f))
        {
            val = (1e-4 - 1.0f);
        }
        return val;
    }

    /**
    // Converts rpy[roll, pitch, yaw] to quaternion[w, x, y, z]
    // https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    */
    static void QuatFromEuler(const float* rpy, float* const wxyz)
    {
        float roll = rpy[0];
        float pitch = rpy[1];
        float yaw = rpy[2];
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);
        wxyz[0] = cy * cp * cr + sy * sp * sr;  //## w
        wxyz[1] = cy * cp * sr - sy * sp * cr;  //## x
        wxyz[2] = sy * cp * sr + cy * sp * cr;  //## y
        wxyz[3] = sy * cp * cr - cy * sp * sr;  //## z
    }

    /**
    // Converts quaternion[w, x, y, z] to rpy[roll, pitch, yaw]
    // https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    */
    static void EulerFromQuat(const float* wxyz, float* const rpy)
    {
        float w = wxyz[0];
        float x = wxyz[1];
        float y = wxyz[2];
        float z = wxyz[3];
        float sinr_cosp = 2 * (w * x + y * z);
        float cosr_cosp = 1 - 2 * (x * x + y * y);
        float roll = atan2(sinr_cosp, cosr_cosp);
        float sinp = 2 * (w * y - z * x);
        sinp = limit_inverse_trigo_value(sinp);
        float pitch = asin(sinp);
        float siny_cosp = 2 * (w * z + x * y);
        float cosy_cosp = 1 - 2 * (y * y + z * z);
        float yaw = atan2(siny_cosp, cosy_cosp);
        rpy[0] = roll;
        rpy[1] = pitch;
        rpy[2] = yaw;
    }

    static float Sigmoid(float x)
    {
	    return 1.0f / (1.0f + exp(-x));
    }

    static void SaveImageFile(const void* buf, const int len, const char* path)
    {
        FILE * pFile;
        pFile = fopen (path, "wb");
        fwrite(buf, sizeof(uint8_t), len, pFile);
        fclose(pFile);
    }

    static float GetHostTimeSec()
    {
        struct timespec now;
        /** use CLOCK_MONOTONIC instead of CLOCK_REALTIME, in case of UTC time sync */
        clock_gettime(CLOCK_REALTIME, &now);
        float nowSec = now.tv_sec + now.tv_nsec*1e-9;
        return nowSec;
    }

};

#endif //__BASIC_UTILS__