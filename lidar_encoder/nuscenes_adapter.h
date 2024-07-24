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
#include "logging_utils.h"
#include "lsch128x1_encoder.h"

struct NuScenesConfig
{
    std::string name;
    std::string dir;
    Json::Value scene;
    Json::Value sample;
    Json::Value sampleData;
    Json::Value nullValue;

    struct LidarSweep 
    {
        uint64_t timestamp_us;
        std::string filename;
        std::string token;
        std::string sample_token;
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
        while (!sample.isNull()) 
        {
            auto lidarSamples = GetLidarSamples(sample);
            for (auto& ls : lidarSamples) 
            {
                sweeps.push_back(LidarSweep{
                    ls["timestamp"].asUInt64(),
                    dir + "/" + ls["filename"].asString(),
                    ls["token"].asString(),
                    ls["sample_token"].asString()
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

        Json::Reader reader;
        reader.parse(configScene,      nuScenesConfig.scene);
        reader.parse(configSample,     nuScenesConfig.sample);
        reader.parse(configSampleData, nuScenesConfig.sampleData);

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

    bool LoadConfig(const char* yaml_config_file)
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
            Json::Value scene_i = ns_config_.scene[i];
            std::vector<NuScenesConfig::LidarSweep> pcds = ns_config_.GetSceneSweeps(scene_i);
            allscene_pcd_files_.push_back(pcds);
            RLOGI("scene[%d] name: %s, pcds.size: %d.", i, scene_i["name"].asString().c_str(), pcds.size());
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

        std::vector<NuScenesConfig::LidarSweep>& pcds = allscene_pcd_files_[scene_idx];
        for(uint32_t i=0; i<pcds.size(); i++)
        {
            NuScenesConfig::LidarSweep& pcd = pcds[i];
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
            ReadPCDFile(pcd.filename, cloud_ptr);
            pcs.push_back(cloud_ptr);
            LOGPF("load file (%s), point size: %ld", pcd.filename.c_str(), cloud_ptr->size());
        }
        RLOGI("read (%d) pcds from scene (%d).", pcs.size(), scene_idx);
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

        std::vector<NuScenesConfig::LidarSweep>& pcds = allscene_pcd_files_[scene_idx];
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
            for(uint32_t i=0; i<pcds.size(); i++)
            {
                NuScenesConfig::LidarSweep& pcd = pcds[i];
                int64_t ts_diff = pcd.timestamp_us - avgts;
                uint64_t uts_diff = std::abs(ts_diff);
                if(uts_diff < mints_diff)
                {
                    mints_diff = uts_diff;
                    mints_idx = i;
                }
            }
            if(mints_idx >= 0)
            {
                NuScenesConfig::LidarSweep& pcd = pcds[mints_idx];
                /** encode with LSCH128X1 protocol */
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
                ReadPCDFile(pcd.filename, cloud_ptr);
                DIFOP_Frame_t difop_frame = LSCH128X1Encoder::MakeDifopFrame(pcd.timestamp_us);
                MSOP_Frames_t msop_frames = LSCH128X1Encoder::MakeMsopFrames(cloud_ptr, pcd.timestamp_us);

                /** save msop and difop frame to file */
                std::string pcd_base_filename = pcd.filename.substr(pcd.filename.find_last_of("//") + 1);
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

                data_i["LIDAR_TOP"]["timestamp_us"] = (Json::UInt64)(pcd.timestamp_us);
                data_i["LIDAR_TOP"]["pcd_file"] = pcd.filename;
                data_i["LIDAR_TOP"]["ch128x1_file"] = pcd_ch128x1_path.c_str();
                data_i["LIDAR_TOP"]["token"] = pcd.token;
                data_i["LIDAR_TOP"]["sample_token"] = pcd.sample_token;
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

    bool LoadLidarJsonThenViz(const char* lidar_json_file)
    {
        std::ifstream lidarJson(lidar_json_file);
        if (!lidarJson.good())
        {
            RLOGE("Can't open file: %s", lidar_json_file);
            return false;
        }
        RLOGI("loaded (%s)", lidar_json_file);

        Json::Reader reader;
        Json::Value root_obj;
        reader.parse(lidarJson, root_obj);
        Json::Value& data_list_obj = root_obj["data"];
        RLOGI("lidar json obj size: %d", data_list_obj.size());
        for(uint32_t i=0; i<data_list_obj.size(); i++)
        {
            Json::Value& data_i = data_list_obj[i];
        }

        return true;
    }

    void test()
    {
        LSCH128X1Encoder::test();
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcs;
        // LoadPCDByScene(1, pcs);
        // LoadCameraJsonThenAddLidar(1, "/home/hugoliu/alaska/dataset/nuscenes/data_set_noa_sgrbg12/nusc_dataset_noa_sgrbg12.json");
        LoadLidarJsonThenViz("/dev/shm/nuscenes/mini/nusc_dataset_noa_sgrbg12.json");
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
        return true;
    }

private:
    NuScenesConfig ns_config_;
    std::vector<std::vector<NuScenesConfig::LidarSweep>> allscene_pcd_files_;
    const std::string PCD_CH128X1_DIR_ = "/dev/shm/nuscenes/mini/";
    
};

#endif //__NUSCENES_ADAPTER_H__
