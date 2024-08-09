#ifndef __NUSCENES_ADAPTER_H__
#define __NUSCENES_ADAPTER_H__

#include <iostream>
#include <fstream>
#include <map>
#include <sys/stat.h>
#include <fcntl.h>
#include "jsoncpp/json/json.h"
#include "yaml-cpp/yaml.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/point_representation.h>
#include "logging_utils.h"
#include "lsch128x1_encoder.h"
#include "pcl_preprocess.h"

#define RUN_TRT_MODEL true
#if RUN_TRT_MODEL
#include "centerpoint_wrapper.h"
#endif

#define RETRIEVE_ANNOTATION_BBOXES true


struct NuScenesConfig
{
    std::string name;
    std::string dir;
    Json::Value scene;
    Json::Value sample;
    Json::Value sampleData;
    Json::Value sampleAnnotation;
    Json::Value instance;
    Json::Value category;
    Json::Value egoPose;
    Json::Value nullValue;

    struct LidarSweep 
    {
        uint64_t timestamp_us;
        std::string filename;
        std::string token;
        std::string sample_token;
        Json::Value obj_quat;
        Json::Value obj_trans;
        Json::Value obj_annotation_list;
        // float quat_wxyz_ego2world[4];
        // float trans_xyz_ego2world[3];

        void PrintInfo() 
        {
            RLOGI("LidarSweep (%s): timestamp %ld, filename: %s.", token.c_str(), timestamp_us, filename.c_str());
        }
    };

    Json::Value& GetSampleById(const std::string sample_id) 
    {
        Json::Value::iterator it = std::find_if(sample.begin(), sample.end(), [&sample_id](Json::Value& v) 
        {
            return v["token"] == sample_id;
        });
        if (it != sample.end()) 
        {
            return *it;
        }
        return nullValue;
    }

    Json::Value& GetNextSample(const Json::Value& sample) 
    {
        if (sample.isNull()) 
        {
            return nullValue;
        }
        return GetSampleById(sample["next"].asString());
    }

    Json::Value GetLidarSamples(const Json::Value& sample)
    {
        std::map<uint64_t, Json::Value> m;
        std::string sample_token = sample["token"].asString();
        for (const Json::Value& data : sampleData) 
        {
            if (data["sample_token"].asString() == sample_token && 
                    data["filename"].asString().find("LIDAR_TOP") != std::string::npos) 
            {
                m[data["timestamp"].asUInt64()] = data;
            }
        }
        Json::Value lidarSamples = Json::Value(Json::arrayValue);
        for (auto v : m) 
        {
            lidarSamples.append(v.second);
        }
        return lidarSamples;
    }

    std::vector<LidarSweep> GetSceneSweeps(const Json::Value& scene) 
    {
        std::vector<LidarSweep> sweeps;
        std::string first_sample_token = scene["first_sample_token"].asString();
        std::string last_sample_token = scene["last_sample_token"].asString();
        
        Json::Value& sample = GetSampleById(first_sample_token);
        uint32_t ls_count = 0;
        while (!sample.isNull()) 
        {
            auto lidarSamples = GetLidarSamples(sample);
            for (auto& ls : lidarSamples) 
            {
                /** find quaternion && translation in ego_pose.json */
                Json::Value obj_quat;
                Json::Value obj_trans;
                for(uint32_t i=0; i<egoPose.size(); i++)
                {
                    Json::Value& ego_pose_i = egoPose[i];
                    if(ls["token"].asString() == ego_pose_i["token"].asString())
                    {
                        ls_count ++;
                        obj_quat = ego_pose_i["rotation"];
                        obj_trans = ego_pose_i["translation"];
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
                        // RLOGI("find ego_pose[%d] quat: (%.4f, %.4f, %.4f, %.4f), trans: (%.4f, %.4f, %.4f)", \
                        //     count, quat_wxyz_ego2world[0], quat_wxyz_ego2world[1], quat_wxyz_ego2world[2], quat_wxyz_ego2world[3], \
                        //     trans_xyz_ego2world[0], trans_xyz_ego2world[1], trans_xyz_ego2world[2]);
                        break;
                    }
                }

                Json::Value obj_annotation_list = Json::Value(Json::arrayValue);
#if RETRIEVE_ANNOTATION_BBOXES
                uint32_t bbox_count = 0;
                /** retrieve annotations from sample_annotation.json */
                for(uint32_t i=0; i<sampleAnnotation.size(); i++)
                {
                    const Json::Value& annotation_i = sampleAnnotation[i];
                    Json::Value annotation_i_copy;
                    if(ls["sample_token"].asString() == annotation_i["sample_token"].asString())
                    {
                        annotation_i_copy["annotation_token"] = annotation_i["token"];
                        annotation_i_copy["translation"] = annotation_i["translation"];
                        annotation_i_copy["size"] = annotation_i["size"];
                        annotation_i_copy["rotation"] = annotation_i["rotation"];

                        for(uint32_t j=0; j<instance.size(); j++)
                        {
                            Json::Value& instance_j = instance[j];
                            // RLOGI("instance token: %s, token: %s", annotation_i["instance_token"].asString().c_str(), instance_j["token"].asString().c_str());
                            if(annotation_i["instance_token"].asString() == instance_j["token"].asString())
                            {
                                // RLOGI("lidar sweeep (%d) annotation (%d), match instance_token (%s)", ls_count, bbox_count, instance_j["token"].asString().c_str());
                                for(uint32_t k=0; k<category.size(); k++)
                                {
                                    Json::Value& category_k = category[k];
                                    if(instance_j["category_token"].asString() == category_k["token"].asString())
                                    {
                                        annotation_i_copy["category_name"] = category_k["name"];
                                        // RLOGI("annotation [%d] name: %s", i, category_k["name"].asString().c_str());
                                        break;
                                    }
                                }
                                break;
                            }
                        }
                        // RLOGI("lidar sweeep (%d) append annotation (%d)",  ls_count, bbox_count);
                        obj_annotation_list[bbox_count++] = annotation_i_copy;
                    }
                }
                // RLOGI("lidarsweep [%d] bbox_count: %d", ls_count, obj_annotation_list.size());
#endif
                sweeps.push_back(LidarSweep{
                    ls["timestamp"].asUInt64(),
                    dir + "/" + ls["filename"].asString(),
                    ls["token"].asString(),
                    ls["sample_token"].asString(),
                    obj_quat,
                    obj_trans,
                    obj_annotation_list
                });
            }
            if (sample["token"].asString() == last_sample_token) 
            {
                break;
            } 
            sample = GetNextSample(sample);
        }
        return sweeps;
    }

    void PrintNuscenesInfo()
    {
        RLOGI("nuScenes dataset (%s) in dir (%s), scene.size: %d, sample.size: %d, sampleData.size: %d.", \
                name.c_str(), dir.c_str(), scene.size(), sample.size(), sampleData.size());
        for(uint32_t i=0; i<scene.size(); i++)
        {
            Json::Value scene_i = scene[i];
            RLOGI("scene[%d] name: %s, nbr_samples: %ld.", i, scene_i["name"].asString().c_str(), scene_i["nbr_samples"].asInt64());
        }
    }

    std::string DataPath()
    {
        return dir + "/" + name; 
    }

    static bool LoadNuscenesJson(const std::string& config_dir, NuScenesConfig& nuScenesConfig)
    {
        YAML::Node config = YAML::LoadFile(config_dir);
        nuScenesConfig.name = config["nu_scenes_dataset"].as<std::string>();
        nuScenesConfig.dir = config["nu_scenes_dir"].as<std::string>();
        RLOGI("nu_scenes_dataset: %s, nu_scenes_dir: %s.", nuScenesConfig.name.c_str(), nuScenesConfig.dir.c_str());

        std::string scene_json_file = nuScenesConfig.DataPath() + "/scene.json";
        std::ifstream configScene(scene_json_file);
        if (!configScene.good())
        {
            RLOGE("Can't open file: %s", scene_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", scene_json_file.c_str());

        std::string sample_json_file = nuScenesConfig.DataPath() + "/sample.json";
        std::ifstream configSample(sample_json_file);
        if (!configSample.good()) 
        {
            RLOGE("Can't open file: %s", sample_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", sample_json_file.c_str());

        std::string sample_data_json_file = nuScenesConfig.DataPath() + "/sample_data.json";
        std::ifstream configSampleData(sample_data_json_file);
        if (!configSampleData.good()) 
        {
            RLOGE("Can't open file: %s", sample_data_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", sample_data_json_file.c_str());

        std::string sample_annotation_json_file = nuScenesConfig.DataPath() + "/sample_annotation.json";
        std::ifstream configSampleAnnotation(sample_annotation_json_file);
        if (!configSampleAnnotation.good()) 
        {
            RLOGE("Can't open file: %s", sample_annotation_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", sample_annotation_json_file.c_str());

        std::string instance_json_file = nuScenesConfig.DataPath() + "/instance.json";
        std::ifstream configInstance(instance_json_file);
        if (!configInstance.good()) 
        {
            RLOGE("Can't open file: %s", instance_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", instance_json_file.c_str());

        std::string category_json_file = nuScenesConfig.DataPath() + "/category.json";
        std::ifstream configCategory(category_json_file);
        if (!configCategory.good()) 
        {
            RLOGE("Can't open file: %s", category_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", category_json_file.c_str());

        std::string ego_pose_json_file = nuScenesConfig.DataPath() + "/ego_pose.json";
        std::ifstream configEgoPose(ego_pose_json_file);
        if (!configEgoPose.good()) 
        {
            RLOGE("Can't open file: %s", ego_pose_json_file.c_str());
            return false;
        }
        RLOGI("loaded (%s)", ego_pose_json_file.c_str());

        Json::Reader reader;
        reader.parse(configScene,      nuScenesConfig.scene);
        reader.parse(configSample,     nuScenesConfig.sample);
        reader.parse(configSampleData, nuScenesConfig.sampleData);
        reader.parse(configEgoPose, nuScenesConfig.egoPose);
        reader.parse(configSampleAnnotation, nuScenesConfig.sampleAnnotation);
        reader.parse(configInstance, nuScenesConfig.instance);
        reader.parse(configCategory, nuScenesConfig.category);

        RLOGI("LoadNuscenesJson success.");
        return true;
    }

};


class NuscenesAdapter
{
public:
    NuscenesAdapter()
    {
    }

    ~NuscenesAdapter()
    {
    }

    bool LoadConfig(const char* yaml_config_file, int set_scene_idx=-1)
    {
        RLOGI("load yaml config %s", yaml_config_file);
        if(!NuScenesConfig::LoadNuscenesJson(yaml_config_file, ns_config_))
        {
            RLOGI("LoadNuscenesJson failed!");
            return false;
        }

        ns_config_.PrintNuscenesInfo();
        for(uint32_t i=0; i<ns_config_.scene.size(); i++)
        {
            if(set_scene_idx>=0 && set_scene_idx!=i)
            {
                continue;
            }
            Json::Value scene_i = ns_config_.scene[i];
            std::vector<NuScenesConfig::LidarSweep> LidarSweeps = ns_config_.GetSceneSweeps(scene_i);
            allscene_pcd_files_.push_back(LidarSweeps);
            RLOGI("scene[%d] name: %s, LidarSweeps.size: %d.", i, scene_i["name"].asString().c_str(), LidarSweeps.size());
        }
        return true;
    }

    bool LoadPCDByScene(const int scene_idx, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& pcs)
    {
        if(scene_idx >= allscene_pcd_files_.size())
        {
            RLOGE("idx (%d) overflow max (%ld)", scene_idx, allscene_pcd_files_.size());
            return false;
        }

        std::vector<NuScenesConfig::LidarSweep>& LidarSweeps = allscene_pcd_files_[scene_idx];
        for(uint32_t i=0; i<LidarSweeps.size(); i++)
        {
            NuScenesConfig::LidarSweep& ls_i = LidarSweeps[i];
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            ReadPCDFile(ls_i.filename, cloud_ptr);
            pcs.push_back(cloud_ptr);
            LOGPF("load file (%s), point size: %ld", ls_i.filename.c_str(), cloud_ptr->size());
        }
        RLOGI("read (%d) LidarSweeps from scene (%d).", pcs.size(), scene_idx);
        return true;
    }

    bool LoadCameraJsonThenAddLidar(const int scene_idx, const char* camera_json_file)
    {
        std::ifstream cameraJson(camera_json_file);
        if (!cameraJson.good()) 
        {
            RLOGE("Can't open file: %s", camera_json_file);
            return false;
        }
        RLOGI("loaded (%s)", camera_json_file);

        std::string output_dir = PCD_CH128X1_DIR_ + "scene_" + std::to_string(scene_idx) + "/";
        gMakeDir(output_dir);

        Json::Reader reader;
        Json::Value root_obj;
        reader.parse(cameraJson, root_obj);
        Json::Value& data_list_obj = root_obj["data"];
        RLOGI("camera json obj size: %d", data_list_obj.size());

        std::vector<NuScenesConfig::LidarSweep>& LidarSweeps = allscene_pcd_files_[scene_idx];
        std::vector<uint64_t> camera_avgts_us;

        for(uint32_t i=0; i<data_list_obj.size(); i++)
        {
            Json::Value& data_i = data_list_obj[i];
            uint64_t tps[6] = {  data_i["CAM_FRONT"]["timestamp"].asUInt64(), data_i["CAM_FRONT_RIGHT"]["timestamp"].asUInt64(), data_i["CAM_BACK_RIGHT"]["timestamp"].asUInt64(), \
                                 data_i["CAM_BACK"]["timestamp"].asUInt64(), data_i["CAM_BACK_LEFT"]["timestamp"].asUInt64(), data_i["CAM_FRONT_LEFT"]["timestamp"].asUInt64()   };
            uint64_t avgts = 0;
            uint64_t mints = std::numeric_limits<uint64_t>::max();
            uint64_t maxts = 0;
            for(uint32_t j=0; j<6; j++)
            {
                avgts += tps[j];
                if(tps[j] < mints)
                {
                    mints = tps[j];
                }
                else if(tps[j] > maxts)
                {
                    maxts = tps[j];
                }
            }
            avgts /= 6;
            camera_avgts_us.push_back(avgts);
            RLOGI("camera block[%d] avgts: %ld us, diff: (%ld) us.", i, avgts, (maxts-mints));

            uint64_t mints_diff = std::numeric_limits<uint64_t>::max();
            int32_t mints_idx = -1;
            for(uint32_t i=0; i<LidarSweeps.size(); i++)
            {
                NuScenesConfig::LidarSweep& ls_i = LidarSweeps[i];
                int64_t ts_diff = ls_i.timestamp_us - avgts;
                uint64_t uts_diff = std::abs(ts_diff);
                if(uts_diff < mints_diff)
                {
                    mints_diff = uts_diff;
                    mints_idx = i;
                }
            }
            if(mints_idx >= 0)
            {
                NuScenesConfig::LidarSweep& ls_i = LidarSweeps[mints_idx];
                /** encode with LSCH128X1 protocol */
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                ReadPCDFile(ls_i.filename, cloud_ptr);
                DIFOP_Frame_t difop_frame = LSCH128X1Encoder::MakeDifopFrame(ls_i.timestamp_us);
                MSOP_Frames_t msop_frames = LSCH128X1Encoder::MakeMsopFrames(cloud_ptr, ls_i.timestamp_us);

                /** save msop and difop frame to file */
                std::string pcd_base_filename = ls_i.filename.substr(ls_i.filename.find_last_of("//") + 1);
                std::string pcd_ch128x1_path = output_dir + pcd_base_filename + ".ch128x1";

                FILE * pFile;
                size_t total_bytes = 0;
                pFile = fopen (pcd_ch128x1_path.c_str(), "wb");
                size_t num_bytes = fwrite(difop_frame.data(), sizeof(uint8_t), difop_frame.size(), pFile);
                total_bytes += num_bytes;
                for(uint32_t j=0; j<msop_frames.size(); j++)
                {
                    num_bytes = fwrite(msop_frames[j].data(), sizeof(uint8_t), msop_frames[j].size(), pFile);
                    total_bytes += num_bytes;
                }
                fclose(pFile);
                RLOGI("pcd_ch128x1_path: %s bytes written: %ld, raw bytes: %ld", \
                    pcd_ch128x1_path.c_str(), total_bytes, LSCH128X1Encoder::CH128X1_FRAME_SIZE_*(msop_frames.size() + 1));

                data_i["LIDAR_TOP"]["timestamp_us"] = (Json::UInt64)(ls_i.timestamp_us);
                data_i["LIDAR_TOP"]["pcd_file"] = ls_i.filename;
                data_i["LIDAR_TOP"]["ch128x1_file"] = pcd_ch128x1_path.c_str();
                data_i["LIDAR_TOP"]["token"] = ls_i.token;
                data_i["LIDAR_TOP"]["sample_token"] = ls_i.sample_token;
                // float quat_wxyz_ego2world[4] = {0.0f};
                // float trans_xyz_ego2world[3] = {0.0f};
                data_i["LIDAR_TOP"]["quat_wxyz_ego2world"] = ls_i.obj_quat;
                data_i["LIDAR_TOP"]["trans_xyz_ego2world"] = ls_i.obj_trans;
                data_i["LIDAR_TOP"]["annotation_list"] = ls_i.obj_annotation_list;
                RLOGI("camera block[%d] match lidar data (diff=%ldus):\n%s\n", i, mints_diff, data_i["LIDAR_TOP"].toStyledString().c_str());
            }
            else
            {
                RLOGE("camera block[%d] match no lidar data.");
            }
        }

        std::string strJson = root_obj.toStyledString();
        std::string camera_json_file_path(camera_json_file);

        std::string json_base_filename = camera_json_file_path.substr(camera_json_file_path.find_last_of("//") + 1);
        std::string json_file_path = PCD_CH128X1_DIR_ + json_base_filename;
        std::ofstream json_ofs;
        json_ofs.open(json_file_path.c_str(), std::ofstream::out);
        json_ofs << strJson << std::endl;
        json_ofs.close();
        // RLOGI("new json string: \n%s", strJson.c_str());
        return true;
    }

    bool LoadLidarJsonThenViz(const char* lidar_json_file, const char* trt_model_path)
    {
        std::ifstream lidarJson(lidar_json_file);
        if (!lidarJson.good())
        {
            RLOGE("Can't open file: %s", lidar_json_file);
            return false;
        }
        RLOGI("loaded (%s)", lidar_json_file);
#if RUN_TRT_MODEL
        if(!load_centerpoint_model(trt_model_path))
        {
            RLOGE("Can't load model: %s", trt_model_path);
            return false;
        }
#endif
        float* points_xyzi_array = new float[4 * PCD_POINTS_NUM_MAX_];

        uint8_t* tmpbuf = new uint8_t[LSCH128X1Encoder::CH128X1_FRAME_SIZE_];
        std::shared_ptr<PclPreprocess> pps = std::make_shared<PclPreprocess>();
        pps->init_filter();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_sensor(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("CH128X1_VIZ"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(1.0, "coord_zero");
        viewer->setCameraPosition(
            0, 0, 30,
            0, 0, 0,
            0, 1, 0); // 0, 1, 0

        /** color: x, y, z, or intensity, larger is redder */
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr color_handler = boost::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> >(cloud_sensor, "intensity");
        viewer->addPointCloud<pcl::PointXYZI>(cloud_sensor, *color_handler, "CH128X1_VIZ", 0);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CH128X1_VIZ");

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

            /** copy points into array, check if pointcloud data interleaved ? */
            memset(points_xyzi_array, 0, 4*PCD_POINTS_NUM_MAX_*sizeof(float));
            // cloud_filter->copyToFloatArray(points_xyzi_array);
            for(uint32_t i=0; i<cloud_filter->size(); i++)
            {
                pcl::PointXYZI& p = cloud_filter->at(i);
                points_xyzi_array[4*i] = p.x;
                points_xyzi_array[4*i+1] = p.y;
                points_xyzi_array[4*i+2] = p.z;
                points_xyzi_array[4*i+3] = p.intensity;
            }
            /** model infer */

            /** pcl visualize */
            viewer->updatePointCloud<pcl::PointXYZI>(cloud_world, *color_handler, "CH128X1_VIZ");
            // usleep(100*1000);
            viewer->spinOnce(100);
        }

        delete points_xyzi_array;
        delete tmpbuf;
        return true;
    }

    void test()
    {
        // LSCH128X1Encoder::test();
        // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcs;
        // LoadPCDByScene(1, pcs);
        // LoadCameraJsonThenAddLidar(1, "/home/hugoliu/alaska/dataset/nuscenes/data_set_noa_sgrbg12/nusc_dataset_noa_sgrbg12.json");
        LoadLidarJsonThenViz("/dev/shm/nuscenes/mini/nusc_dataset_noa_sgrbg12.json", "/home/hugoliu/github/nuscenes/nuscenes_pcl_viz/centerpoint/model/centerpoint_fp16.trt");
    }

private:
    bool ReadPCDFile(const std::string& fname, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
    {
        if (fname.empty())
        {
            return false;
        }
        std::ifstream file(fname, std::ios::in | std::ios::binary);
        if (!file.good()) 
        {
            RLOGE("Error during openning the file: %s.", fname.c_str());
            return false;
        }

        int cnt = 0;
        float max_i = 0;
        while (file) 
        {
            float x,y,z,i,r;
            file.read(reinterpret_cast<char*>(&x), sizeof(float));
            file.read(reinterpret_cast<char*>(&y), sizeof(float));
            file.read(reinterpret_cast<char*>(&z), sizeof(float));
            file.read(reinterpret_cast<char*>(&i), sizeof(float));
            file.read(reinterpret_cast<char*>(&r), sizeof(float));
            max_i = std::max(max_i, i);
            pcl::PointXYZI point{};
            point.x = x;
            point.y = y;
            point.z = z;
            point.intensity = i; //std::sqrt(x*x + y*y + z*z) < 10 ? 3 : 250 ;
            cloud->push_back(point);
        }
        RLOGI("load file (%s) points: (%d)", fname.c_str(), cloud->size());
        file.close();

#if 0
        for(uint32_t idx=0; idx<cloud->size(); idx++)
        {
            pcl::PointXYZI& p = cloud->at(idx);
            float fx = p.x;
            float fy = p.y;
            float fz = p.z;
            float fi = p.intensity / 100.0f;
            if(idx % 1000 == 0)
            {
                RLOGI("point[%d]: %.3f, %.3f, %.3f, %.3f", idx, fx, fy, fz, fi);
            }
        }
#endif
        return true;
    }

private:
    NuScenesConfig ns_config_;
    std::vector<std::vector<NuScenesConfig::LidarSweep>> allscene_pcd_files_;
    const std::string PCD_CH128X1_DIR_ = "/dev/shm/nuscenes/mini/";
    constexpr static uint32_t PCD_POINTS_NUM_MAX_ = 153600;
    
};

#endif //__NUSCENES_ADAPTER_H__
