//
// Created by kevinlad on 2020/4/8.
//

#ifndef SRC_RADAR_H
#define SRC_RADAR_H



#include <thread>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <huaxun_radar_msgs/RadarTargetArray.h>
//#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace radar{

    class Radar {

        using PointMsg = huaxun_radar_msgs::RadarPoint;
        using TrackMsg = huaxun_radar_msgs::RadarTrack;
        using TargetArrayMsg = huaxun_radar_msgs::RadarTargetArray;
    public:

        explicit Radar(char* mode);

        bool TestParse();

        bool TestSendRec();

        ~Radar();

    private:
        void Init();

        void ReceiveThread();

        PointMsg ParsePointFrame(const unsigned char *data);

        TrackMsg ParseTrackFrame(const unsigned char *data);

        void PrintFrameInfo(int can_index, unsigned int can_frame_id,int data_len, const unsigned char *data);

        void PublishMsg();

    private:

        std::shared_ptr<std::thread> receive_thread_ptr_;

        std::string radar_frame_ = "radar";
        std::string radar_targets_topic_ = "/radar_targets";
        std::string radar_pointcloud_raw_topic_ = "/radar_pointcloud_raw";
        std::string radar_pointcloud_topic_ = "/radar_pointcloud";

        bool print_frame = true;

        ros::NodeHandle nh_;
        ros::Publisher radar_pub_;
        ros::Publisher pointcloud_pub_;
        ros::Publisher pointcloud_raw_pub_;

//        VCI_BOARD_INFO pInfo1 [50]; // All usb-can devices info
//        VCI_CAN_OBJ rec_buffer_[3000];
//        VCI_INIT_CONFIG config_;

        int count=0; //数据列表中，用来存储列表序号。

        unsigned int num_ = 0; //account of USB-CAN Devices

        std::shared_ptr<TargetArrayMsg> last_radar_target_array_msg_;
        std::shared_ptr<TargetArrayMsg> radar_target_array_msg_;


    };

}

#endif //SRC_RADAR_H
