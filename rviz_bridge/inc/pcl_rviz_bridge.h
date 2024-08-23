#ifndef __PCL_RVIZ_BRIDGE_H__
#define __PCL_RVIZ_BRIDGE_H__

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "jsoncpp/json/json.h"
#include "logging_utils.h"
#include "basic_utils.h"
#include "lsch128x1_encoder.h"
#include "pcl_preprocess.h"

class PclRvizBridgeNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_annotation_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_detection_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_ego_vehicle_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ego_traj_;
    nav_msgs::msg::Path path_ego_traj_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    float ego_xyz_origin[3] = {0.0f};
    bool ego_origin_set = false;
    int play_period_ms_ = 500;

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

        pub_annotation_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pcl_rviz/nuscenes_annotation", 10);
        pub_detection_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pcl_rviz/nuscenes_detection", 10);
        pub_ego_vehicle_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pcl_rviz/ego_vehicle", 10);
        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcl_rviz/nuscenes_pointcloud", 10);
        pub_ego_traj_ = this->create_publisher<nav_msgs::msg::Path>("/pcl_rviz/ego_trajectory", 10);

        // sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/pcl_rviz/nuscenes_odom", qos, std::bind(&PclRvizBridgeNode::odom_callback, this, std::placeholders::_1));
        RLOGI("json_file_path: %s", json_file_path);
        ego_origin_set = false;

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
                trans_xyz_ego2world[j] = obj_trans[j].asFloat() - ego_xyz_origin[j];
            }

            if(!ego_origin_set)
            {
                ego_xyz_origin[0] = trans_xyz_ego2world[0];
                ego_xyz_origin[1] = trans_xyz_ego2world[1];
                // ego_xyz_origin[2] = trans_xyz_ego2world[2];
                ego_xyz_origin[2] = 0.0f;
                trans_xyz_ego2world[0] = 0.0f;
                trans_xyz_ego2world[1] = 0.0f;
                ego_origin_set = true;
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
            // pps -> TransformLidar2Ego(cloud_filter, cloud_ego);
            pps -> GetTfLidar2World(quat_wxyz_ego2world, trans_xyz_ego2world);
            pps -> TransformLidar2World(cloud_filter, cloud_world);

            /** pub pointcloud */
            sensor_msgs::msg::PointCloud2 pc2_msg;
            pcl::toROSMsg(*cloud_world, pc2_msg);
            // pcl::toROSMsg(*cloud_filter, pc2_msg);
            pc2_msg.header.frame_id = "world";
            pub_pointcloud_->publish(pc2_msg);

            /** pub ego vehicle */
            visualization_msgs::msg::MarkerArray ego_markers;
            visualization_msgs::msg::Marker ego_vehicle;
            ego_vehicle.header = pc2_msg.header;
            ego_vehicle.type = visualization_msgs::msg::Marker::CUBE;
            ego_vehicle.id = 0;
            ego_vehicle.action = visualization_msgs::msg::Marker::ADD;
            ego_vehicle.lifetime = rclcpp::Duration(0, 0); //last 100ms

            ego_vehicle.pose.position.x = trans_xyz_ego2world[0];
            ego_vehicle.pose.position.y = trans_xyz_ego2world[1];
            ego_vehicle.pose.position.z = trans_xyz_ego2world[2];

            ego_vehicle.pose.orientation.w = quat_wxyz_ego2world[0];
            ego_vehicle.pose.orientation.x = quat_wxyz_ego2world[1];
            ego_vehicle.pose.orientation.y = quat_wxyz_ego2world[2];
            ego_vehicle.pose.orientation.z = quat_wxyz_ego2world[3];

            ego_vehicle.scale.x = 5.0;
            ego_vehicle.scale.y = 2.0;
            ego_vehicle.scale.z = 1.5;

            /** choose vehicle color */
            ego_vehicle.color.a = 1.0;
            ego_vehicle.color.r = 1.0;
            ego_vehicle.color.g = 1.0;
            ego_vehicle.color.b = 1.0;
            ego_markers.markers.push_back(ego_vehicle);
            pub_ego_vehicle_->publish(ego_markers);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = pc2_msg.header;
            pose_stamped.pose.orientation = ego_vehicle.pose.orientation;
            pose_stamped.pose.position = ego_vehicle.pose.position;
            path_ego_traj_.header = pc2_msg.header;
            path_ego_traj_.poses.push_back(pose_stamped);
            pub_ego_traj_->publish(path_ego_traj_);

            /** pub objects, annotaion_list in world-coord, transform into first ego-coord */
            Json::Value& annotation_list = data_i["LIDAR_TOP"]["annotation_list"];
            visualization_msgs::msg::MarkerArray annotation_markers;
            for(uint32_t k=0; k<annotation_list.size(); k++)
            {
                Json::Value& annotation_i = annotation_list[k];
                const std::string category_name = annotation_i["category_name"].asString();
                Json::Value& quat_i = annotation_i["rotation"];
                Json::Value& trans_i = annotation_i["translation"];
                Json::Value& size_i = annotation_i["size"];
                float quat_wxyz[4] = {0.0f};
                float trans_xyz[3] = {0.0f};
                float size_xyz[3] = {0.0f};
                for(uint32_t j=0; j<quat_i.size(); j++)
                {
                    quat_wxyz[j] = quat_i[j].asFloat();
                }
                for(uint32_t j=0; j<trans_i.size(); j++)
                {
                    /** use first position as origin */
                    trans_xyz[j] = trans_i[j].asFloat() - ego_xyz_origin[j];
                }
                for(uint32_t j=0; j<size_i.size(); j++)
                {
                    size_xyz[j] = size_i[j].asFloat();
                }

                visualization_msgs::msg::Marker marker;
                marker.header = pc2_msg.header;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.id = k;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.lifetime = rclcpp::Duration(0, play_period_ms_*1e6); //last 100ms

                marker.pose.position.x = trans_xyz[0];
                marker.pose.position.y = trans_xyz[1];
                marker.pose.position.z = trans_xyz[2];

                marker.pose.orientation.w = quat_wxyz[0];
                marker.pose.orientation.x = quat_wxyz[1];
                marker.pose.orientation.y = quat_wxyz[2];
                marker.pose.orientation.z = quat_wxyz[3];

                marker.scale.x = size_xyz[1];
                marker.scale.y = size_xyz[0];
                marker.scale.z = size_xyz[2];

                /** choose vehicle color */
                marker.color.a = 0.3;

                if(strncmp(category_name.c_str(), "human", 5)==0)
                {
                    /** pedestrian: red */
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }
                else if(strncmp(category_name.c_str(), "vehicle.car", 11)==0)
                {
                    /** car: blue */
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
                else if(strncmp(category_name.c_str(), "vehicle.motorcycle", 18)==0)
                {
                    /** motorcycle: white */
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                }
                else if(strncmp(category_name.c_str(), "vehicle.bus", 11)==0)
                {
                    /** bus: green */
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else if(strncmp(category_name.c_str(), "vehicle.bicycle", 15)==0)
                {
                    /** bicycle: cyan */
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                }
                else if(strncmp(category_name.c_str(), "vehicle.truck", 13)==0)
                {
                    /** truck: yellow */
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else if(strncmp(category_name.c_str(), "movable_object.trafficcone", 26)==0)
                {
                    /** trafficcone: purple */
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
                else 
                {
                    /** black as others */
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                }

                annotation_markers.markers.push_back(marker);
            }
            pub_annotation_markers_->publish(annotation_markers);

            /** pub objects, centerpoint detection in world-coord, transform into first ego-coord */
            Json::Value& detection_list = data_i["LIDAR_TOP"]["obj_box3d_list"];
            visualization_msgs::msg::MarkerArray detection_markers;
            for(uint32_t k=0; k<detection_list.size(); k++)
            {
                Json::Value& detection_i = detection_list[k];
                const int cls_i = detection_i["cls"].asInt64();
                const float score_i = detection_i["score"].asFloat();

                float yaw_rad_i = detection_i["theta"].asFloat();
                const float velx_i = detection_i["velX"].asFloat();
                const float vely_i = detection_i["velY"].asFloat();
                // bool using_track_angle = false;
                // if(fabs(velx_i)>0.1f || fabs(vely_i)>0.1f)
                // {
                //     /** use track angle instead. */
                //     yaw_rad_i = atan2(velx_i, vely_i);
                //     using_track_angle = true;
                // }

                float trans_xyz_lidar[3] = {0.0f};
                float trans_xyz_world[3] = {0.0f};
                float size_xyz[3] = {0.0f};
                trans_xyz_lidar[0] = detection_i["x"].asFloat();
                trans_xyz_lidar[1] = detection_i["y"].asFloat();
                trans_xyz_lidar[2] = detection_i["z"].asFloat();

                // pps -> PoseLidar2World(trans_xyz_lidar, quat_wxyz_ego2world, trans_xyz_ego2world, trans_xyz_world);
                pps -> PoseEgo2World(trans_xyz_lidar, quat_wxyz_ego2world, trans_xyz_ego2world, trans_xyz_world);
                size_xyz[0] = detection_i["h"].asFloat();
                size_xyz[1] = detection_i["l"].asFloat();
                size_xyz[2] = detection_i["w"].asFloat();

                visualization_msgs::msg::Marker marker;
                marker.header = pc2_msg.header;
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.id = k;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.lifetime = rclcpp::Duration(0, play_period_ms_*1e6); //last 100ms

                marker.pose.position.x = trans_xyz_world[0];
                marker.pose.position.y = trans_xyz_world[1];
                marker.pose.position.z = trans_xyz_world[2];
                // marker.pose.position.x = trans_xyz_lidar[0];
                // marker.pose.position.y = trans_xyz_lidar[1];
                // marker.pose.position.z = trans_xyz_lidar[2];

                const float rpy_i[3] = {0.0f, 0.0f, yaw_rad_i-0.5f*M_PI}; //0.5f*M_PI
                float quat_ego_i[4] = {0};
                float quat_world_i[4] = {0};
                BasicUtils::QuatFromEuler(rpy_i, quat_ego_i);
                // pps -> OrientationLidar2World(quat_ego_i, quat_wxyz_ego2world, quat_world_i);
                pps -> OrientationEgo2World(quat_ego_i, quat_wxyz_ego2world, quat_world_i);

                marker.pose.orientation.w = quat_world_i[0];
                marker.pose.orientation.x = quat_world_i[1];
                marker.pose.orientation.y = quat_world_i[2];
                marker.pose.orientation.z = quat_world_i[3];

                marker.scale.x = size_xyz[0];
                marker.scale.y = size_xyz[1];
                marker.scale.z = size_xyz[2];

                /** choose vehicle color */
                marker.color.a = 0.6;

                // 0: car,         
                // 1: truck,       2: construction_vehicle
                // 3: bus,         4: trailer,     
                // 5: barrier
                // 6: motorcycle   7: bicycle      
                // 8: pedestrian   9: traffic_cone
                if(cls_i==8 || cls_i==9)
                {
                    if(score_i < 0.60f)
                    {
                        continue;
                    }

                    /** pedestrian: red */
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }
                else if(cls_i==0)
                {
                    if(score_i < 0.40f)
                    {
                        continue;
                    }

                    /** car: blue */
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;

                    // if(using_track_angle)
                    // {
                    //     marker.color.g = 1.0;
                    // }
                }
                else if(cls_i==3 || cls_i==4)
                {
                    if(score_i < 0.20f)
                    {
                        continue;
                    }

                    /** bus: green */
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    continue;
                }
                else if(cls_i==6 || cls_i==7)
                {
                    if(score_i < 0.20f)
                    {
                        continue;
                    }

                    /** bicycle: cyan */
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                    continue;
                }
                else if(cls_i==1 || cls_i==2)
                {
                    if(score_i < 0.20f)
                    {
                        continue;
                    }

                    /** truck: yellow */
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    continue;
                }
                else 
                {
                    if(score_i < 0.40f)
                    {
                        continue;
                    }

                    /** black as others */
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                    continue;
                }

                detection_markers.markers.push_back(marker);
            }
            pub_detection_markers_->publish(detection_markers);

            RLOGI("publish pointcloud (%d) with (%d) annotation (%d) detection", i, annotation_list.size(), detection_list.size());
            usleep(play_period_ms_*1000);
        }

        delete tmpbuf;
        return true;
    }

};

#endif //__PCL_RVIZ_BRIDGE_H__