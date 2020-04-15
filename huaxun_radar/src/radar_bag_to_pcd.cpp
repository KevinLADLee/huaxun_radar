//
// Created by kevinlad on 2020/4/12.
//

#include "radar_bag_to_pcd.h"

#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <huaxun_radar_msgs/RadarTargetArray.h>
#include <sensor_msgs/point_cloud_conversion.h>

//typedef sensor_msgs::PointCloud2 PointCloud;
//typedef PointCloud::Ptr PointCloudPtr;
//typedef PointCloud::ConstPtr PointCloudConstPtr;

struct PclRadarPointType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
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
                                 (std::uint8_t, id, id)
                                 (std::uint16_t, peak, peak)
                                 (std::uint8_t, car_speed, car_speed)
                                 )
struct PclRadarTrackType{
  PCL_ADD_POINT4D;
  float vx;
  float vy;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PclRadarTrackType,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, vx, vx)
                                      (float, vy, vy))


/* ---[ */
int main (int argc, char** argv)
{
    ros::init (argc, argv, "radar_bag_to_pcd");
    if (argc < 4)
    {
        std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <topic> <output_directory> [<target_frame>]" << std::endl;
        std::cerr << "Example: " << argv[0] << " data.bag /laser_tilt_cloud ./pointclouds /base_link" << std::endl;
        return (-1);
    }

    // TF
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_it;

    try
    {
        bag.open (argv[1], rosbag::bagmode::Read);
    }
    catch (const rosbag::BagException&)
    {
        std::cerr << "Error opening file " << argv[1] << std::endl;
        return (-1);
    }

    // check that target topic exists in the bag file:
    rosbag::View topic_list_view(bag);
    std::string target_topic;
    std::map<std::string, std::string> topic_list;
    for(rosbag::ConnectionInfo const *ci: topic_list_view.getConnections() )
    {
        topic_list[ci->topic] = ci->datatype;
        if (ci->topic == argv[2])
        {
            if (ci->datatype == std::string("huaxun_radar_msgs/RadarTargetArray"))
            {
                target_topic = std::string (argv[2]);
                view.addQuery (bag, rosbag::TopicQuery (target_topic));
            }
            else
            {
                std::cerr << "Provided topic '" << argv[2] << "' is in the bag file, but is not of type sensor_msgs/PointCloud (type: " << ci->datatype << ")" << std::endl;
            }
        }
    }
    if (target_topic.empty())
    {
        std::cerr << "Could not find a sensor_msgs/PointCloud type on topic '" << argv[2] << "' in bag file " << argv[1] << std::endl;
        std::cerr << "Topics found in the bag file:" << std::endl;
        for (std::map<std::string, std::string>::iterator it=topic_list.begin(); it!=topic_list.end(); ++it)
            std::cout << "    " << it->first << " (" << it->second << ")" << std::endl;
        return (-1);
    }

    view.addQuery (bag, rosbag::TypeQuery ("tf/tfMessage"));
    view.addQuery (bag, rosbag::TypeQuery ("tf2_msgs/TFMessage"));
    view_it = view.begin ();

    std::string output_dir = std::string (argv[3]);
    boost::filesystem::path outpath (output_dir);
    if (!boost::filesystem::exists (outpath))
    {
        if (!boost::filesystem::create_directories (outpath))
        {
            std::cerr << "Error creating directory " << output_dir << std::endl;
            return (-1);
        }
        std::cerr << "Creating directory " << output_dir << std::endl;
    }

    // Add the PointCloud handler
    std::cerr << "Saving recorded sensor_msgs::PointCloud messages on topic " << target_topic << " to " << output_dir << std::endl;

    sensor_msgs::PointCloud2 points_msg;
    sensor_msgs::PointCloud2 tracks_msg;
    ros::Duration r (0.001);
    // Loop over the whole bag file
    while (view_it != view.end ())
    {
        // Handle TF messages first
        tf::tfMessage::ConstPtr tf = view_it->instantiate<tf::tfMessage> ();
        if (tf != NULL)
        {
            tf_broadcaster.sendTransform (tf->transforms);
            ros::spinOnce ();
            r.sleep ();
        }
        else
        {
            // Get the PointCloud2 message
          huaxun_radar_msgs::RadarTargetArrayConstPtr radar_targets_ptr_ = view_it->instantiate<huaxun_radar_msgs::RadarTargetArray> ();

          if (radar_targets_ptr_ == NULL)
          {
            ++view_it;
            continue;
          }

          pcl::PointCloud<PclRadarPointType> points;
          auto point_size = radar_targets_ptr_->radar_point_array.size();
          points.points.resize(point_size);
          points.width = point_size;
          points.height = 1;
          for(unsigned int i = 0; i < point_size; i++){
            auto &p = radar_targets_ptr_->radar_point_array[i];
            points.points[i].x = -std::sin(p.target_angle / 180.0 * M_PI) * p.target_distance;
            points.points[i].y = std::cos(p.target_angle / 180.0 * M_PI) * p.target_distance;
            points.points[i].z = 0;
            points.points[i].vx = -std::sin(p.target_angle / 180.0 * M_PI) * p.target_speed;
            points.points[i].vy = std::cos(std::abs(p.target_angle / 180.0 )* M_PI) * p.target_speed;
            points.points[i].id = p.target_id;
            points.points[i].peak = p.target_peak_val;
          }

          pcl::toROSMsg(points, points_msg);
          points_msg.header = radar_targets_ptr_->header;


          std::cerr << "Got " << points_msg.width * points_msg.height << " data points in frame " << points_msg.header.frame_id << " on topic " << view_it->getTopic() << " with the following fields: " << pcl::getFieldsList (points_msg) << std::endl;

          std::stringstream ss;
          ss << output_dir << "/radar_points/" << points_msg.header.stamp << ".pcd";
          std::cerr << "Data saved to " << ss.str () << std::endl;
          pcl::io::savePCDFile (ss.str (), points_msg, Eigen::Vector4f::Zero (),
                                Eigen::Quaternionf::Identity (), true);
        }
      // Increment the iterator
      ++view_it;
    }

    return (0);
}