#ifndef __LSCH128X1_ENCODER_H__
#define __LSCH128X1_ENCODER_H__

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "logging_utils.h"

typedef std::vector<int32_t> CH128X1_DATE_t;

typedef union{
    uint32_t us_part;
    uint8_t buf[4];
}CH128X1_TSUS_u;

typedef union{
    int16_t int_part;
    uint8_t buf[2];
}CH128X1_INT16_u;

typedef union{
    uint16_t int_part;
    uint8_t buf[2];
}CH128X1_UINT16_u;

typedef std::vector<std::vector<uint8_t>> MSOP_Frames_t;
typedef std::vector<uint8_t> DIFOP_Frame_t;

class LSCH128X1Encoder
{
public:
    LSCH128X1Encoder()
    {
    }

    ~LSCH128X1Encoder()
    {
    }

    static CH128X1_DATE_t TimestampToDate(const uint64_t timestamp_us)
    {
        std::chrono::microseconds ts_us(timestamp_us);
        std::chrono::time_point <std::chrono::system_clock, std::chrono::microseconds> tp_us(ts_us);
        std::chrono::system_clock::time_point tp (tp_us);
        std::time_t tt = std::chrono::system_clock::to_time_t(tp);
        struct tm * timeinfo = localtime (&tt);
        // const char* date_str = ctime(&tt);
        char tmpbuf[256] = {0};
        strftime(tmpbuf, sizeof(tmpbuf), "%Y %m %d %H %M %S", timeinfo);
        // RLOGI("timestamp: %ld us, date: %s", timestamp_us, tmpbuf);
        CH128X1_DATE_t date_vec;
        std::stringstream tmpss(tmpbuf);
        std::string tmpstr;
        while(getline(tmpss, tmpstr, ' '))
        {
            int32_t val = std::atoi(tmpstr.c_str());
            date_vec.push_back(val);
            // RLOGI("date_vec: %d", val);
        }

        uint32_t us_part = (timestamp_us - std::floor(timestamp_us/1e6)*1e6);
        date_vec.push_back(us_part);
        // RLOGI("us_part: %d", us_part);

        return date_vec;
    }

    static DIFOP_Frame_t MakeDifopFrame(const uint64_t timestamp_us)
    {
        static uint32_t difop_count_ = 0;
        DIFOP_Frame_t difop_frame;
        difop_frame.resize(CH128X1_FRAME_SIZE_);
        // std::vector<uint8_t>::iterator frame_head_it = difop_frame.begin();
        // difop_frame.insert(frame_head_it, CH128X1_DIFOP_HEADER_, CH128X1_DIFOP_HEADER_ + 8);
        for(uint32_t i=0; i<8; i++)
        {
            difop_frame[i] = CH128X1_DIFOP_HEADER_[i];
        }
        const uint16_t motor_speed = 600; //rpm
        difop_frame[8] = (uint8_t)((motor_speed & 0xff00) >> 8);
        difop_frame[9] = (uint8_t)(motor_speed & 0x00ff);

        CH128X1_DATE_t date_vec = TimestampToDate(timestamp_us);
        difop_frame[52] = ((date_vec[0]) - 2000) % 255; // year 0~255
        difop_frame[53] = date_vec[1]; // month 1~12
        difop_frame[54] = date_vec[2]; // day 1~31
        difop_frame[55] = date_vec[3]; // hour 0~23
        difop_frame[56] = date_vec[4]; // min  0~59
        difop_frame[57] = date_vec[5]; // sec  0~59
        // difop_frame.insert(frame_head_it+1204, CH128X1_DIFOP_TAIL_, CH128X1_DIFOP_TAIL_ + 2);
        difop_frame[1204] = CH128X1_DIFOP_TAIL_[0]; 
        difop_frame[1205] = CH128X1_DIFOP_TAIL_[1]; 

        difop_count_ += 1;
        // RLOGI("difop[%d] frame size %d", difop_count_, difop_frame.size());
        assert(CH128X1_FRAME_SIZE_ == difop_frame.size());
        return difop_frame;
    }

    /** package pcl points into 1206*N bytes */
    static MSOP_Frames_t MakeMsopFrames(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, const uint64_t timestamp_us)
    {
        static uint32_t msop_count_ = 0;
        MSOP_Frames_t msop_frames;

        const uint32_t points_count = cloud_ptr->size();
        const uint32_t subframe_count = uint32_t(points_count / CH128X1_SUBFRAME_POINTS_); //the max is 8983
        for(uint32_t j=0; j<subframe_count; j++)
        {
            std::vector<uint8_t> msop_frame;
            uint32_t subframe_count = CH128X1_SUBFRAME_POINTS_;
            /** attach header on the first msop frame */
            if(j==0)
            {
                msop_frame.insert(msop_frame.end(), CH128X1_MSOP_HEADER_, CH128X1_MSOP_HEADER_+7);
                subframe_count -= 1;
            }
            for(uint32_t i=0; i<subframe_count; i++)
            {
                const uint32_t idx = i + j*CH128X1_SUBFRAME_POINTS_;
                pcl::PointXYZI& p = cloud_ptr->at(idx);
                float fx = p.x;
                float fy = p.y;
                float fz = p.z;
                float fi = p.intensity / 100.0f;
                if(idx % 1000 == 0)
                {
                    // RLOGI("point[%d]: %.3f, %.3f, %.3f, %.3f", idx, fx, fy, fz, fi);
                }

                float dist = sqrt(fx*fx + fy*fy);
                float hori_angle = acos(fy/dist)*57.3f;
                float vert_angle = atan(fz/dist)*57.3f;
                uint32_t line_num = std::round((vert_angle - LOWER_FOV_) / VERTICAL_RESOLUTION_);

                std::vector<uint8_t> subframe;
                subframe.resize(CH128X1_SUBFRAME_BYTES_);
                subframe[0] = line_num;

                CH128X1_INT16_u hori_angle_u;
                hori_angle_u.int_part = (int16_t)(100*hori_angle);
                subframe[1] = hori_angle_u.buf[0];
                subframe[2] = hori_angle_u.buf[1];

                CH128X1_UINT16_u dist_cm_u;
                dist_cm_u.int_part = (int16_t)(100*dist);
                subframe[3] = dist_cm_u.buf[0];
                subframe[4] = dist_cm_u.buf[1];

                uint8_t dist_residual = uint8_t(256*(dist*100 - dist_cm_u.int_part));
                subframe[5] = dist_residual;
                subframe[6] = uint8_t(255*fi);

                msop_frame.insert(msop_frame.end(), subframe.begin(), subframe.end()); 
            }

            /** %Y %m %d %H %M %S  %US */
            CH128X1_DATE_t date_vec = TimestampToDate(timestamp_us);
            CH128X1_TSUS_u tsus_u;
            tsus_u.us_part = date_vec[6];
            std::vector<uint8_t> hmsus;
            hmsus.resize(7);
            hmsus[0] = uint8_t(date_vec[3]);
            hmsus[1] = uint8_t(date_vec[4]);
            hmsus[2] = uint8_t(date_vec[5]);
            hmsus[3] = uint8_t(tsus_u.buf[0]);
            hmsus[4] = uint8_t(tsus_u.buf[1]);
            hmsus[5] = uint8_t(tsus_u.buf[2]);
            hmsus[6] = uint8_t(tsus_u.buf[3]);
            msop_frame.insert(msop_frame.end(), hmsus.begin(), hmsus.end());

            msop_frame.push_back(0x80); //stands for CH128X1
            msop_frame.push_back(0x02); //stands for double return
            msop_count_ += 1;
            msop_frames.push_back(msop_frame);
            assert(CH128X1_FRAME_SIZE_ == msop_frame.size());
            // if(msop_frame.size() != CH128X1_FRAME_SIZE_)
            // {
            //     RLOGE("msop[%d] frame[%ld] size %d", msop_count_, msop_frames.size(), msop_frame.size());
            // }
            // RLOGI("msop[%d] frame[%ld] size %d", msop_count_, msop_frames.size(), msop_frame.size());
        }
        RLOGI("in this frame msop count: [%d], msop total count: [%d]", msop_frames.size(), msop_count_);
        return msop_frames;
    }

    static void test()
    {
        std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
        std::chrono::system_clock::duration dtn = tp.time_since_epoch();
        // uint64_t ts_us = std::chrono::duration_cast<std::chrono::microseconds>(dtn).count();
        uint64_t ts_us = 1533151603512404;
        TimestampToDate(ts_us);
    }

public:
    constexpr static uint32_t CH128X1_FRAME_SIZE_ = 1206; // msop frame contain 171 points, and 9 bytes additional info
    constexpr static uint32_t CH128X1_SUBFRAME_BYTES_ = 7;
    constexpr static uint32_t CH128X1_SUBFRAME_POINTS_ = 171;
    constexpr static uint8_t CH128X1_MSOP_HEADER_[7] = {0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11};
    constexpr static uint8_t CH128X1_DIFOP_HEADER_[8] = {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55};
    constexpr static uint8_t CH128X1_DIFOP_TAIL_[2] = {0x0F, 0xF0};
    constexpr static uint8_t CH128X1_UCWP_HEADER_[8] = {0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA};
    constexpr static uint8_t CH128X1_UCWP_TAIL_[2] = {0x0F, 0xF0};
    constexpr static float LOWER_FOV_ = -18.0;
    constexpr static float UPPER_FOV_ = 7.0;
    constexpr static uint32_t CHANNELS_ = 128;
    constexpr static float VERTICAL_RESOLUTION_ = (UPPER_FOV_ - LOWER_FOV_) / CHANNELS_;
};

#endif //__LSCH128X1_ENCODER_H__