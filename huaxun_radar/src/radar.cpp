//
// Created by kevinlad on 2020/4/8.
//

#include "radar.h"
#include "controlcan.h"

namespace radar{

Radar::Radar() {
  Init();
}

Radar::Radar(char* test) {
    TestParse();
}

Radar::~Radar() {
  receive_thread_ptr_->join();
  std::cout << "Stop receiving thread success!" << std::endl;
  std::cout << "Closing USB-CAN device!" << std::endl;
  usleep(100000); // delay100ms
  VCI_ResetCAN(VCI_USBCAN2, 0, 0); //Reset CAN1
  usleep(100000); // delay100ms
  VCI_ResetCAN(VCI_USBCAN2, 0, 1); //Reset CAN2
  usleep(100000); // delay100ms
  VCI_CloseDevice(VCI_USBCAN2,0) ;// Close Device
  std::cout << "Close USB-CAN device success!" << std::endl;
}

void Radar::Init() {

  // ROS Initialization
  radar_pub_ = nh_.advertise<TargetArrayMsg>("/radar_targets", 10);
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud", 1);
  pointcloud_raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_pointcloud_raw", 1);

  if(!InitCan()){
    std::cerr << "USB-CAN Init failed!" << std::endl;
    return;
  }

  std::cout << "Starting receiving thread..." << std::endl;
  receive_thread_ptr_ = std::make_shared<std::thread>(std::bind(&Radar::ReceiveThread, this));

}

bool Radar::InitCan() {
  VCI_BOARD_INFO pInfo1 [50];
  unsigned int num = VCI_FindUsbDevice2(pInfo1);
  std::cout << ">>Find " << num << " USB-CAN devices" << std::endl;

  if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//Open Device
  {
    std::cout << ">>Open deivce success!" << std::endl;
  }else
  {
    std::cerr << ">>Open deivce error!" << std::endl;
    return false;
//    exit(1);
    //TODO: Safe exit
  }

  //TODO: rosparam support
  VCI_INIT_CONFIG config;
  config.AccCode=0;
  config.AccMask=0xFFFFFFFF;
  config.Filter=1;// Receive All frames
  config.Timing0=0x00;  // 500 kbps
  config.Timing1=0x1C;
  config.Mode=0;// Normal mode

  if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
  {
    std::cerr << ">>Init CAN1 error"<< std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
    return false;
  }

  if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
  {
    std::cerr << ">>Start CAN1 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
    return false;
  }

  if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
  {
    std::cerr << ">>Init can2 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
    return false;
  }
  if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
  {
    std::cerr << ">>Start can2 error" << std::endl;
    VCI_CloseDevice(VCI_USBCAN2,0);
    return false;
  }
}

void Radar::ReceiveThread() {
  int reclen=0;
  unsigned int i,j;
  int ind=0;
  bool head = false;
  bool first_head = false;
  bool tail = false;

  VCI_CAN_OBJ rec_buffer[3000];

  while(!ros::isShuttingDown())
  {
    if((reclen = VCI_Receive(VCI_USBCAN2,0,ind,rec_buffer,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
    {

      auto current_rec_time = ros::Time::now();

      for(j=0; j<reclen; j++)
      {
        if(print_frame) {
          PrintFrameInfo(ind, rec_buffer[j].ID, rec_buffer[j].DataLen,rec_buffer[j].Data);
        }

        switch (rec_buffer[j].ID){
          case 0x70A:
            head = true;
            first_head = true;
            radar_target_array_msg_ = std::make_shared<TargetArrayMsg>();
            radar_target_array_msg_->header.stamp = current_rec_time;
            break;
          case 0x70C:
            if(head && first_head) {
              auto point_msg = ParsePointFrame(rec_buffer[j].Data);
              radar_target_array_msg_->radar_point_array.push_back(point_msg);
            }
            break;
          case 0x70E:
            if(head && first_head) {
              auto track_msg = ParseTrackFrame(rec_buffer[j].Data);
              radar_target_array_msg_->radar_track_array.push_back(ParseTrackFrame(rec_buffer[j].Data));
            }
            break;
          case 0x70F:
            tail = true;
            head = false;
            PublishMsg();
            //reset
            last_radar_target_array_msg_ = radar_target_array_msg_;
            radar_target_array_msg_.reset();
            break;
        };
      }
    }
    ind=!ind; // Switch CAN1 / CAN2
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(30ms);
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
  radar_pub_.publish(*radar_target_array_msg_);
  auto msg_timestamp = radar_target_array_msg_->header.stamp;

  sensor_msgs::PointCloud2 pointcloud_raw_msg;
  pointcloud_raw_msg.header.stamp = msg_timestamp;
  pointcloud_raw_msg.header.frame_id = radar_frame_;
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
  pointcloud_raw_pub_.publish(pointcloud_raw_msg);

  sensor_msgs::PointCloud2 pointcloud_msg;
  pointcloud_msg.header.stamp = msg_timestamp;
  pointcloud_msg.header.frame_id = radar_frame_;
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