//
// Created by kevinlad on 2020/4/8.
//

#include "radar.h"

namespace radar{

Radar::Radar() {

  radar_frame_ = "radar";

  Init();

}

Radar::~Radar() {
  if(receive_thread_ptr_ != nullptr){
    receive_thread_ptr_->join();
  }
  canBusOff(can_handle_);
  canClose(can_handle_);
  canUnloadLibrary();
}

void Radar::Init() {

  // ROS Initialization
  radar_tracks_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_points", 1);
  radar_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_tracks", 1);

  if(CanInit() != 0){
    std::cerr << "CAN Init failed!" << std::endl;
    return;
  }

  std::cout << "Starting receiving thread..." << std::endl;
  receive_thread_ptr_ = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this));

}

int Radar::CanInit() {
  std::cout << "Reading messages on channel " << channel_ << std::endl;
  canInitializeLibrary();
  can_handle_ = canOpenChannel(channel_, canOPEN_EXCLUSIVE | canOPEN_REQUIRE_EXTENDED | canOPEN_ACCEPT_VIRTUAL);
  if(can_handle_ < 0){
    std::cerr << "canOpenChannel: " << channel_ << std::endl;
  }

//  stat = canSetNotify(can_handle_, notifyCallback, canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS | canNOTIFY_ENVVAR, (char*)0);

  can_status_ = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canSetBusParams failed!" << std::endl;
    return -1;
  }


  can_status_ = canAccept(can_handle_, 0x0F0L, canFILTER_SET_MASK_STD);
  if (can_status_ < 0) {
    std::cerr << "canAccept set mask failed!" << std::endl;
    return -1;
  }
  can_status_ = canAccept(can_handle_, 0x060L, canFILTER_SET_CODE_STD);
  if (can_status_ < 0) {
    std::cerr << "canAccept set code failed!" << std::endl;
    return -1;
  }
  can_status_ = canBusOn(can_handle_);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canBusOn failed!" << std::endl;
    return -1;
  }
  return 0;
}

void Radar::ReceiveThread() {
  int reclen=0;
  unsigned int i,j;
  int ind=0;
  bool head = false;
  bool first_head = false;
  bool tail = false;

  auto can_msg_ptr = std::make_shared<CanMsg>();

  while(!ros::isShuttingDown())
  {

    can_status_ = canReadWait(can_handle_, &can_msg_ptr->id, &can_msg_ptr->msg, &can_msg_ptr->dlc, &can_msg_ptr->flag, &can_msg_ptr->time, 60);

    if(can_status_ == canOK)
    {
      auto current_rec_time = ros::Time::now();
      if(print_frame) {
        PrintFrameInfo(channel_, can_msg_ptr->id, can_msg_ptr->dlc, can_msg_ptr->msg);
      }

      switch (can_msg_ptr->id){
        case 0x70A:
          head = true;
          first_head = true;
          timestamp_ = current_rec_time;
          break;
        case 0x70C:
          if(head && first_head) {
            radar_point_msg_vec_.push_back(CanMsgToRadarPointMsg(can_msg_ptr->msg));
          }
          break;
        case 0x70E:
          if(head && first_head) {
            radar_track_msg_vec_.push_back(CanMsgToRadarTrackMsg(can_msg_ptr->msg));
          }
          break;
        case 0x70F:
          tail = true;
          if(head && first_head) {
            head = false;
            PublishMsg();
            //reset
            radar_point_msg_vec_.clear();
            radar_track_msg_vec_.clear();
          }
          break;
      };

    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
  }
}


void Radar::PrintFrameInfo(int can_index, unsigned int can_frame_id, int data_len, const unsigned char *data) {
  std::cout << "CAN" << can_index+1 << " RX | ID: 0x" << std::hex << can_frame_id;
  for(int i = 0; i < data_len; i++)
  {
    std::cout << " ";
    std::cout.width(2);
    std::cout.fill('0');
    std::cout << std::hex << (unsigned int)data[i];
  }
  std::cout << std::endl;
}

void Radar::PublishMsg() {
  if(radar_track_msg_vec_.empty() && radar_point_msg_vec_.empty()){
    std::cerr << "Error: No msgs to publish" << std::endl;
    return;
  }

  pcl::PointCloud<PclRadarPointType> points;
  RadarPointMsgVecToPcl(radar_point_msg_vec_, points);
  sensor_msgs::PointCloud2 points_msg;
  pcl::toROSMsg(points, points_msg);
  points_msg.header.frame_id = radar_frame_;
  points_msg.header.stamp = timestamp_;
  radar_points_pub_.publish(points_msg);

  pcl::PointCloud<PclRadarTrackType> tracks;
  RadarTrackMsgVecToPcl(radar_track_msg_vec_, tracks);
  sensor_msgs::PointCloud2 tracks_msg;
  pcl::toROSMsg(tracks, tracks_msg);
  tracks_msg.header.frame_id = radar_frame_;
  tracks_msg.header.stamp = timestamp_;
  radar_tracks_pub_.publish(tracks_msg);


}



}