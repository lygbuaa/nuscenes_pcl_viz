#ifndef __PCL_PREPROCESS_H__
#define __PCL_PREPROCESS_H__

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include "logging_utils.h"

class PclPreprocess
{
public:
    PclPreprocess()
    {
        tf_l2e_ = GetTfLidar2Ego();
    }

    ~PclPreprocess()
    {
    }

    void init_filter()
    {
        // Create the filtering object
        cr_filter_ = std::make_shared<pcl::ConditionalRemoval<pcl::PointXYZI>>();
        pcl::ConditionOr<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZI>());
        /** filter conditions in sensor coord */
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::LT, -2.0f)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x", pcl::ComparisonOps::GT, 2.0f)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::LT, -3.0f)));
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y", pcl::ComparisonOps::GT, 3.0f)));
        cr_filter_->setCondition (range_cond);
        cr_filter_->setKeepOrganized (false);

        // pcl::PassThrough<pcl::PointXYZI> pass;
        // pass.setInputCloud (cloud);
        // pass.setFilterFieldName ("z");
        // pass.setFilterLimits (0.0, 1.0);
        // pass.setNegative (true);
        // pass.filter (*cloud_filtered);
    }

    static std::string MatrixtoString(const Eigen::MatrixXf& mat)
    {
        std::stringstream ss;
        ss << mat;
        return ss.str();
    }

    static Eigen::Affine3f GetTfLidar2Ego()
    {
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        tf.translation() << TRANS_XYZ_LIDAR2EGO_[0], TRANS_XYZ_LIDAR2EGO_[1], TRANS_XYZ_LIDAR2EGO_[2];
        tf.rotate(Eigen::Quaternionf(QUAT_WXYZ_LIDAR2EGO_[0], QUAT_WXYZ_LIDAR2EGO_[1], QUAT_WXYZ_LIDAR2EGO_[2], QUAT_WXYZ_LIDAR2EGO_[3]));
        Eigen::Matrix4f mat = tf.matrix();
        // RLOGI("GetTfLidar2Ego:\n%s", MatrixtoString(mat).c_str());
        return tf;
    }

    Eigen::Affine3f GetTfEgo2World(const float* quat_wxyz_e2w, const float* trans_xyz_e2w)
    {
        Eigen::Affine3f tf = Eigen::Affine3f::Identity();
        // tf.translation() << trans_xyz_e2w[0], trans_xyz_e2w[1], trans_xyz_e2w[2];
        tf.rotate(Eigen::Quaternionf(quat_wxyz_e2w[0], quat_wxyz_e2w[1], quat_wxyz_e2w[2], quat_wxyz_e2w[3]));
        Eigen::Matrix4f mat = tf.matrix();
        // RLOGI("GetTfEgo2World:\n%s", MatrixtoString(mat).c_str());
        return tf;
    }

    Eigen::Affine3f GetTfLidar2World(const float* quat_wxyz_e2w, const float* trans_xyz_e2w)
    {
        Eigen::Affine3f tf_l2e = GetTfLidar2Ego();
        Eigen::Affine3f tf_e2w = GetTfEgo2World(quat_wxyz_e2w, trans_xyz_e2w);
        Eigen::Affine3f tf_l2w = tf_e2w * tf_l2e;
        Eigen::Matrix4f mat = tf_l2w.matrix();
        RLOGI("GetTfLidar2World:\n%s", MatrixtoString(mat).c_str());
        tf_l2w_ = tf_l2w;
        return tf_l2w;
    }

    void TransformLidar2Ego(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
    {
        HANG_STOPWATCH();
        Eigen::Affine3f tf_l2e = GetTfLidar2Ego();
        pcl::transformPointCloud (*cloud_in, *cloud_out, tf_l2e);
    }

    void TransformLidar2World(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
    {
        HANG_STOPWATCH();
        // Eigen::Affine3f tf_l2w = GetTfLidar2World();
        pcl::transformPointCloud (*cloud_in, *cloud_out, tf_l2w_);
    }

    void Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
    {
        HANG_STOPWATCH();
        cr_filter_->setInputCloud(cloud_in);
        cr_filter_->filter(*cloud_out);
        RLOGI("filter input size (%d), output size (%d)", cloud_in->size(), cloud_out->size());
    }

    void test()
    {
        float quat_wxyz_e2w[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float trans_xyz_e2w[3] = {1.0f, 2.0f, 3.0f};
        // GetTfLidar2Ego();
        // GetTfEgo2World(quat_wxyz_e2w, trans_xyz_e2w);
        GetTfLidar2World(quat_wxyz_e2w, trans_xyz_e2w);
    }

private:
    constexpr static float QUAT_WXYZ_LIDAR2EGO_[4] = {0.706749235646644, -0.015300993788500868, 0.01739745181256607, -0.7070846669051719};
    constexpr static float TRANS_XYZ_LIDAR2EGO_[3] = {0.985793, 0.0, 1.84019};
    Eigen::Affine3f tf_l2e_;
    Eigen::Affine3f tf_l2w_;
    std::shared_ptr<pcl::ConditionalRemoval<pcl::PointXYZI>> cr_filter_;
};

#endif //__PCL_PREPROCESS_H__