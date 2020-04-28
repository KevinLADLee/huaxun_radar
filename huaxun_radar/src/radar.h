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

#include <canlib.h>

#include <huaxun_radar_msgs/RadarTargetArray.h>

#include <sensor_msgs/PointCloud2.h>

namespace radar{


struct CanMsg{
  long id = 0;
  unsigned char msg[8];
  unsigned int dlc = 0;
  unsigned int flag = 0;
  unsigned long time = 0;
};

class Radar {

  using PointMsg = huaxun_radar_msgs::RadarPoint;
  using TrackMsg = huaxun_radar_msgs::RadarTrack;
  using TargetArrayMsg = huaxun_radar_msgs::RadarTargetArray;
 public:

  Radar();

  bool TestParse();

  ~Radar();

 private:
  void Init();


  int CanInit();

  int CanRead();

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

  canHandle can_handle_;
  canStatus can_status_;
  int channel_ = 0;


  int count=0; //数据列表中，用来存储列表序号。

//        unsigned int num_ = 0; //account of USB-CAN Devices

  std::shared_ptr<TargetArrayMsg> last_radar_target_array_msg_;
  std::shared_ptr<TargetArrayMsg> radar_target_array_msg_;


};

}

#endif //SRC_RADAR_H
