#ifndef __PCL_RVIZ_BRIDGE_H__
#define __PCL_RVIZ_BRIDGE_H__

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.h>
#include "jsoncpp/json/json.h"
#include "logging_utils.h"
#include "lsch128x1_encoder.h"
#include "pcl_preprocess.h"

class PclRvizBridgeNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

public:
    PclRvizBridgeNode() : Node("PclRvizBridgeNode")
    {}
    ~PclRvizBridgeNode() {}

    bool init(const char* json_file_path)
    {
        rclcpp::QoS qos = rclcpp::QoS(
            rclcpp::KeepLast(3)
        );
        qos.best_effort();
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_rviz/nuscenes_pointcloud", 10);
        // sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/pcl_rviz/nuscenes_odom", qos, std::bind(&PclRvizBridgeNode::odom_callback, this, std::placeholders::_1));
        RLOGI("json_file_path: %s", json_file_path);

        /** read json file */
        play_json_file(json_file_path);

        return true;
    }

    bool play_json_file(const char* json_file_path)
    {
        std::ifstream lidarJson(json_file_path);
        if (!lidarJson.good())
        {
            RLOGE("Can't open file: %s", json_file_path);
            return false;
        }
        RLOGI("loaded (%s)", json_file_path);
        std::shared_ptr<PclPreprocess> pps = std::make_shared<PclPreprocess>();
        uint8_t* tmpbuf = new uint8_t[LSCH128X1Encoder::CH128X1_FRAME_SIZE_];
        pps->init_filter();
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sensor(new pcl::PointCloud<pcl::PointXYZI>());

        Json::Reader reader;
        Json::Value root_obj;
        reader.parse(lidarJson, root_obj);
        Json::Value& data_list_obj = root_obj["data"];
        RLOGI("lidar json obj size: %d", data_list_obj.size());
        for(uint32_t i=0; i<data_list_obj.size(); i++)
        {
            Json::Value& data_i = data_list_obj[i];
            std::string ch128x1_file = data_i["LIDAR_TOP"]["ch128x1_file"].asString();
            uint64_t timestamp_us = data_i["LIDAR_TOP"]["timestamp_us"].asUInt64();
            Json::Value& obj_quat = data_i["LIDAR_TOP"]["quat_wxyz_ego2world"];
            Json::Value& obj_trans = data_i["LIDAR_TOP"]["trans_xyz_ego2world"];
            float quat_wxyz_ego2world[4] = {0.0f};
            float trans_xyz_ego2world[3] = {0.0f};
            for(uint32_t j=0; j<obj_quat.size(); j++)
            {
                quat_wxyz_ego2world[j] = obj_quat[j].asFloat();
            }
            for(uint32_t j=0; j<obj_trans.size(); j++)
            {
                trans_xyz_ego2world[j] = obj_trans[j].asFloat();
            }
            RLOGI("lidar block[%d] timestamp_us (%ld), ch128x1_file: %s\nquat: (%.4f, %.4f, %.4f, %.4f), trans: (%.4f, %.4f, %.4f)", \
                    i, timestamp_us, ch128x1_file.c_str(), \
                    quat_wxyz_ego2world[0], quat_wxyz_ego2world[1], quat_wxyz_ego2world[2], quat_wxyz_ego2world[3], \
                    trans_xyz_ego2world[0], trans_xyz_ego2world[1], trans_xyz_ego2world[2]);
            /** open ch128x1 file, load frames and decode into pcl::PointCloud */
            cloud_sensor->clear();

            FILE * pFile;
            pFile = fopen(ch128x1_file.c_str(), "rb");

            do{
                memset(tmpbuf, 0, LSCH128X1Encoder::CH128X1_FRAME_SIZE_);
                size_t num_bytes = fread(tmpbuf, sizeof(uint8_t), LSCH128X1Encoder::CH128X1_FRAME_SIZE_, pFile);
                if(num_bytes == LSCH128X1Encoder::CH128X1_FRAME_SIZE_)
                {
                    /** decode difop frame */
                    std::vector<uint8_t> difop_frame(tmpbuf, tmpbuf+LSCH128X1Encoder::CH128X1_FRAME_SIZE_);
                    LSCH128X1Encoder::ParseDifopFrame(difop_frame);

                    uint32_t msop_count = 0;
                    while(num_bytes == LSCH128X1Encoder::CH128X1_FRAME_SIZE_)
                    {
                        memset(tmpbuf, 0, LSCH128X1Encoder::CH128X1_FRAME_SIZE_);
                        num_bytes = fread(tmpbuf, sizeof(uint8_t), LSCH128X1Encoder::CH128X1_FRAME_SIZE_, pFile);
                        if(num_bytes != LSCH128X1Encoder::CH128X1_FRAME_SIZE_)
                        {
                            RLOGI("file %s reach end, total %d msop frames.", ch128x1_file.c_str(), msop_count);
                            break;
                        }
                        std::vector<uint8_t> msop_frame(tmpbuf, tmpbuf+LSCH128X1Encoder::CH128X1_FRAME_SIZE_);
                        msop_count ++;
                        /** decode msop frame, store points into cloud_sensor */
                        LSCH128X1Encoder::ParseMsopFrame(msop_frame, cloud_sensor);
                    }
                }
                else
                {
                    RLOGE("read file %s error.", ch128x1_file.c_str());
                    break;
                }
            }while(false);

            RLOGI("file %s points count %d.", ch128x1_file.c_str(), cloud_sensor->size());
            fclose(pFile);

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ego(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_world(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZI>());

            pps -> Filter(cloud_sensor, cloud_filter);
            // pps -> TransformLidar2Ego(cloud_sensor, cloud_ego);
            pps -> GetTfLidar2World(quat_wxyz_ego2world, trans_xyz_ego2world);
            pps -> TransformLidar2World(cloud_filter, cloud_world);

            sensor_msgs::msg::PointCloud2 pc2_msg;
            pcl::toROSMsg(*cloud_world, pc2_msg);
            pc2_msg.header.frame_id = "world";
            pub_pointcloud_->publish(pc2_msg);

            RLOGI("publish pointcloud (%d)", i);
            usleep(100*1000);
        }

        delete tmpbuf;
        return true;
    }

};

#endif //__PCL_RVIZ_BRIDGE_H__