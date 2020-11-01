/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/front_end/front_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr; //QS:什么意思？？

bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    response.succeed = _front_end_flow_ptr->SaveMap(); //保存地图
    _front_end_flow_ptr->PublishGlobalMap();
    return response.succeed;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node"); //节点初始化
    ros::NodeHandle nh; //节点句柄

    // 创建一个名为save_map的server，注册回调函数save_map_callback
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    //QS：创建的这个nh干嘛的
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100); //设置循环的频率
    while (ros::ok()) {
        ros::spinOnce(); //监听反馈函数

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}