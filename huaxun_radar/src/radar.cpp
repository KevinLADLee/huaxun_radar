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
  receive_thread_ptr_->join();
  canBusOff(can_handle_);
  canClose(can_handle_);
  canUnloadLibrary();
}

void Radar::Init() {

  // ROS Initialization
  radar_pub_ = nh_.advertise<TargetArrayMsg>("/radar_targets", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud", 1);
  pointcloud_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud_raw", 1);

  if(CanInit() != 0){
    std::cerr << "USB-CAN Init failed!" << std::endl;
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
//    check("", can_handle);
  }

//  stat = canSetNotify(can_handle_, notifyCallback, canNOTIFY_RX | canNOTIFY_TX | canNOTIFY_ERROR | canNOTIFY_STATUS | canNOTIFY_ENVVAR, (char*)0);

  can_status_ = canSetBusParams(can_handle_, canBITRATE_500K, 0, 0, 0, 0, 0);
  if (can_status_ != canOK) {
    std::cerr << "CanInit: canSetBusParams failed!" << std::endl;
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
          radar_target_array_msg_ = std::make_shared<TargetArrayMsg>();
          radar_target_array_msg_->header.stamp = current_rec_time;
          break;
        case 0x70C:
          if(head && first_head) {
            auto point_msg = ParsePointFrame(can_msg_ptr->msg);
            radar_target_array_msg_->radar_point_array.push_back(point_msg);
          }
          break;
        case 0x70E:
          if(head && first_head) {
            auto track_msg = ParseTrackFrame(can_msg_ptr->msg);
            radar_target_array_msg_->radar_track_array.push_back(ParseTrackFrame(can_msg_ptr->msg));
          }
          break;
        case 0x70F:
          tail = true;
          if(head && first_head) {
            head = false;
            PublishMsg();
            //reset
            last_radar_target_array_msg_ = radar_target_array_msg_;
            radar_target_array_msg_.reset();
          }
          break;
      };

    }
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(10ms);
  }
}

Radar::PointMsg Radar::ParsePointFrame(const unsigned char *data) {
  auto point_msg = Radar::PointMsg();
  point_msg.target_id       = (uint8_t)(data[0]);
  point_msg.target_peak_val = (uint16_t)(data[1] + ((data[2] & 0x1f) << 8));
  auto raw_target_distance  = (float)((data[2] >> 5) + (data[3] << 3));
  point_msg.car_speed       = (uint8_t)data[4];
  auto raw_target_angle     = (float)(data[5] + ((data[6] & 0x0f) << 8));
  auto raw_target_speed     = (float)((data[6] >> 4) + (data[7] << 4));

  point_msg.target_distance = raw_target_distance / 10.0f;
  point_msg.target_angle    = (raw_target_angle * 0.1f) - 180.0f;
  point_msg.target_speed    = (raw_target_speed * 0.025f) - 50.0f;
  return point_msg;
}

Radar::TrackMsg Radar::ParseTrackFrame(const unsigned char *data) {
  auto track_msg = Radar::TrackMsg();
  track_msg.target_id = (uint8_t)(data[0] & 0x3f);
  auto raw_v_x = (float)(((data[2] & 0x07) << 10) + (data[1] << 2) + (data[0] >> 6));
  auto raw_v_y = (float)((data[3] << 5) + (data[2] >> 3));
  auto raw_r_x = (float)((data[5] << 8) + data[4]);
  auto raw_r_y = (float)((data[7] << 8) + data[6]);

  track_msg.x_velocity = raw_v_x / 50.0f - 50.0f;
  track_msg.y_velocity = raw_v_y / 50.0f - 50.0f;
  track_msg.x_distance = raw_r_x / 50.0f - 200.0f;
  track_msg.y_distance = raw_r_y / 50.0f - 10.0f;

  return track_msg;
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
  if(radar_target_array_msg_ == nullptr){
    std::cerr << "Error: radar_target_array_msg_ is null." << std::endl;
    return;
  }
  radar_pub_.publish(*radar_target_array_msg_);
  auto msg_timestamp = radar_target_array_msg_->header.stamp;

  sensor_msgs::PointCloud2 pointcloud_raw_msg;
  pcl::PointCloud<pcl::PointXYZ> cloud_raw;
  cloud_raw.width = radar_target_array_msg_->radar_point_array.size();
  cloud_raw.height = 1;
  cloud_raw.is_dense = false;
  cloud_raw.points.resize(cloud_raw.width * cloud_raw.height);
  for(unsigned int m = 0; m < cloud_raw.width; m++){
    const auto &p = radar_target_array_msg_->radar_point_array[m];
    cloud_raw.points[m].x = std::sin(p.target_angle / 180.0 * M_PI) * p.target_distance;
    cloud_raw.points[m].y = std::cos(p.target_angle / 180.0 * M_PI) * p.target_distance;
    cloud_raw.points[m].z = 0;
  }
  pcl::toROSMsg(cloud_raw, pointcloud_raw_msg);
  pointcloud_raw_msg.header.stamp = msg_timestamp;
  pointcloud_raw_msg.header.frame_id = radar_frame_;
  pointcloud_raw_pub_.publish(pointcloud_raw_msg);

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = radar_target_array_msg_->radar_track_array.size();
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);
  for(unsigned int m = 0; m < cloud.width; m++){
    const auto &p = radar_target_array_msg_->radar_track_array[m];
    cloud.points[m].x = p.x_distance;
    cloud.points[m].y = p.y_distance;
    cloud.points[m].z = 0;
  }
  pcl::toROSMsg(cloud, pointcloud_msg);
  pointcloud_msg.header.stamp = msg_timestamp;
  pointcloud_msg.header.frame_id = radar_frame_;
  pointcloud_pub_.publish(pointcloud_msg);


}

bool Radar::TestParse() {

  std::cout << "TestParse!" << std::endl;
  std::cout << "*****************" << std::endl;


  // Test Parse Point frame
  uint8_t data[8];
  data[7] = 0x7D;
  data[6] = 0x06;
  data[5] = 0xFA;
  data[4] = 0x00;
  data[3] = 0x25;
  data[2] = 0xB4;
  data[1] = 0xDB;
  data[0] = 0x01;
  auto point_msg = ParsePointFrame(data);


  std::cout << "Point msg: " << std::endl;
  std::cout << "id: " << (unsigned int)point_msg.target_id << std::endl;
  std::cout << "peak_val " << std::dec << (unsigned int)point_msg.target_peak_val << std::endl;
  std::cout << "target_distance: " << point_msg.target_distance << std::endl;
  std::cout << "Car Speed: " << (unsigned int)point_msg.car_speed << std::endl;
  std::cout << "target angle: " << point_msg.target_angle << std::endl;
  std::cout << "target velocity: " << point_msg.target_speed << std::endl;


  // Test Parse Track msg
  uint8_t data_track[8];
  data_track[7] = 0x01;
  data_track[6] = 0x0c;
  data_track[5] = 0x27;
  data_track[4] = 0x32;
  data_track[3] = 0x4e;
  data_track[2] = 0x22;
  data_track[1] = 0x70;
  data_track[0] = 0x42;

  auto track_msg = ParseTrackFrame(data_track);
  std::cout << "*****************" << std::endl;
  std::cout << "Track msg:" << std::endl;
  std::cout << "id: " << (unsigned int)track_msg.target_id << std::endl;
  std::cout << "v_x: " << track_msg.x_velocity << std::endl;
  std::cout << "v_y: " << track_msg.y_velocity << std::endl;
  std::cout << "r_x: " << track_msg.x_distance << std::endl;
  std::cout << "r_y: " << track_msg.y_distance << std::endl;


}

}