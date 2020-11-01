/*
 * @Description: ICP 匹配模块
 * @Author: Qin Han
 * @Date: 2020-10-31 10:32:36
 */

#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class ICPRegistration : public RegistrationInterface {
  public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float max_dist, float trans_eps, float eculi_eps, int max_iter);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override; //继承自基类，override一定要重写

//为什么这两个private不写在一起:私有函数
  private:
    bool SettRegistrationParam(float max_dist, float trans_eps, float eculi_eps, int max_iter);

  private: //私有变量
    //pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
    //pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_; //为什么要加上中间的这个< >？？

};
}


#endif