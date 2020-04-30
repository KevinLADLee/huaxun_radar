
#ifndef SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_
#define SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_

struct RadarTrackMsg{
  RadarTrackMsg() = default;
  std::uint8_t id = 0;
  float x = 0;
  float y = 0;
  float vx = 0;
  float vy = 0;
};

using RadarTrackMsgVec = std::vector<RadarTrackMsg>;

struct RadarPointMsg{
  RadarPointMsg() = default;
  std::uint8_t id = 0;
  std::uint16_t peak = 0;
  std::uint8_t car_speed = 0;
  float x = 0;
  float y = 0;
  float vx = 0;
  float vy = 0;
};

using RadarPointMsgVec = std::vector<RadarPointMsg>;


struct PclRadarPointType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
  float vz;
  std::uint8_t id;
  std::uint16_t peak;
  std::uint8_t car_speed;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclRadarPointType,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, vx, vx)
                                      (float, vy, vy)
                                      (float, vz, vz)
                                      (std::uint8_t, id, id)
                                      (std::uint16_t, peak, peak)
                                      (std::uint8_t, car_speed, car_speed)
)
struct PclRadarTrackType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
  float vz;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclRadarTrackType,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, vx, vx)
                                      (float, vy, vy)
                                      (float, vz, vz))

inline RadarPointMsg CanMsgToRadarPointMsg(const unsigned char *data) {
  auto point_msg = RadarPointMsg();
  point_msg.id       = (uint8_t)(data[0]);
  point_msg.peak = (uint16_t)(data[1] + ((data[2] & 0x1f) << 8));
  point_msg.car_speed       = (uint8_t)data[4];

  auto raw_target_distance  = (float)((data[2] >> 5) + (data[3] << 3));
  float target_distance = raw_target_distance / 10.0f;

  auto raw_target_angle     = (float)(data[5] + ((data[6] & 0x0f) << 8));
  float target_angle    = (raw_target_angle * 0.1f) - 180.0f;

  point_msg.x = std::sin(target_angle / 180.0 * M_PI) * target_distance;
  point_msg.y = std::cos(target_angle / 180.0 * M_PI) * target_distance;

  auto raw_target_speed     = (float)((data[6] >> 4) + (data[7] << 4));
  float target_speed    = (raw_target_speed * 0.025f) - 50.0f;
  point_msg.vx = std::sin(target_angle / 180.0 * M_PI) * target_speed;
  point_msg.vy = std::cos(target_angle / 180.0 * M_PI) * target_speed;

  return point_msg;
}

inline RadarTrackMsg CanMsgToRadarTrackMsg(const unsigned char *data) {
  auto track_msg = RadarTrackMsg();
  track_msg.id = (uint8_t)(data[0] & 0x3f);
  auto raw_v_x = (float)(((data[2] & 0x07) << 10) + (data[1] << 2) + (data[0] >> 6));
  auto raw_v_y = (float)((data[3] << 5) + (data[2] >> 3));
  auto raw_r_x = (float)((data[5] << 8) + data[4]);
  auto raw_r_y = (float)((data[7] << 8) + data[6]);
  track_msg.vx = raw_v_x / 50.0f - 50.0f;
  track_msg.vy = raw_v_y / 50.0f - 50.0f;
  track_msg.x = raw_r_x / 50.0f - 200.0f;
  track_msg.y = raw_r_y / 50.0f - 10.0f;
  return track_msg;
}

inline void RadarPointMsgVecToPcl(const RadarPointMsgVec &radar_point_vec, pcl::PointCloud<PclRadarPointType> points){
  auto point_size = radar_point_vec.size();
  points.points.resize(point_size);
  points.width = point_size;
  points.height = 1;
  for(unsigned int i = 0; i < point_size; i++){
    auto &p = radar_point_vec[i];
    points.points[i].x = p.x;
    points.points[i].y = p.y;
    points.points[i].z = 0;
    points.points[i].vx = p.vx;
    points.points[i].vy = p.vy;
    points.points[i].vz = 0;
    points.points[i].id = p.id;
    points.points[i].peak = p.peak;
  }
}

inline void RadarTrackMsgVecToPcl(const RadarTrackMsgVec &radar_track_vec, pcl::PointCloud<PclRadarTrackType> &tracks){
  auto track_size = radar_track_vec.size();
  tracks.resize(track_size);
  tracks.width = track_size;
  tracks.height = 1;
  for(unsigned int i = 0; i < track_size; i++){
    auto &t = radar_track_vec[i];
    tracks.points[i].x = t.x;
    tracks.points[i].y = t.y;
    tracks.points[i].z = 0;
    tracks.points[i].vx = t.vx;
    tracks.points[i].vy = t.vy;
    tracks.points[i].vz = 0;
  }
}


//bool Radar::TestParse() {
//
//  std::cout << "TestParse!" << std::endl;
//  std::cout << "*****************" << std::endl;
//
//
//  // Test Parse Point frame
//  uint8_t data[8];
//  data[7] = 0x7D;
//  data[6] = 0x06;
//  data[5] = 0xFA;
//  data[4] = 0x00;
//  data[3] = 0x25;
//  data[2] = 0xB4;
//  data[1] = 0xDB;
//  data[0] = 0x01;
//  auto point_msg = ParsePointFrame(data);
//
//
//  std::cout << "Point msg: " << std::endl;
//  std::cout << "id: " << (unsigned int)point_msg.target_id << std::endl;
//  std::cout << "peak_val " << std::dec << (unsigned int)point_msg.target_peak_val << std::endl;
//  std::cout << "target_distance: " << point_msg.target_distance << std::endl;
//  std::cout << "Car Speed: " << (unsigned int)point_msg.car_speed << std::endl;
//  std::cout << "target angle: " << point_msg.target_angle << std::endl;
//  std::cout << "target velocity: " << point_msg.target_speed << std::endl;
//
//
//  // Test Parse Track msg
//  uint8_t data_track[8];
//  data_track[7] = 0x01;
//  data_track[6] = 0x0c;
//  data_track[5] = 0x27;
//  data_track[4] = 0x32;
//  data_track[3] = 0x4e;
//  data_track[2] = 0x22;
//  data_track[1] = 0x70;
//  data_track[0] = 0x42;
//
//  auto track_msg = ParseTrackFrame(data_track);
//  std::cout << "*****************" << std::endl;
//  std::cout << "Track msg:" << std::endl;
//  std::cout << "id: " << (unsigned int)track_msg.target_id << std::endl;
//  std::cout << "v_x: " << track_msg.x_velocity << std::endl;
//  std::cout << "v_y: " << track_msg.y_velocity << std::endl;
//  std::cout << "r_x: " << track_msg.x_distance << std::endl;
//  std::cout << "r_y: " << track_msg.y_distance << std::endl;
//
//
//}

#endif //SRC_HUAXUN_RADAR_HUAXUN_RADAR_SRC_PROTOCOL_H_
