#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/fill_image.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>

#include <boost/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

void increment(int* value)
{
  ++(*value);
}

class Anaglyph3D
{
private:
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter left_sub_, right_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_;
  image_transport::Publisher pub_anaglyph_;
  
  sensor_msgs::ImageConstPtr last_left_, last_right_;
  boost::mutex image_mutex_;
  sensor_msgs::Image img_ana_;
  
  ros::WallTimer check_synced_timer_;
  int left_received_, right_received_, all_received_;

public:
  Anaglyph3D(ros::NodeHandle& nh)
    : it_(nh), sync_(2), left_received_(0),
      right_received_(0), all_received_(0)
  {
    ros::NodeHandle local_nh("~");
    
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s", left_topic.c_str(), right_topic.c_str());

    left_sub_.subscribe(it_, left_topic, 3);
    right_sub_.subscribe(it_, right_topic, 3);
    sync_.connectInput(left_sub_, right_sub_);
    sync_.registerCallback(boost::bind(&Anaglyph3D::imageCB, this, _1, _2));

    pub_anaglyph_ = it_.advertise(stereo_ns + "anaglyph3D", 1);

    // Complain every 30s if the topics appear unsynchronized
    left_sub_.registerCallback(boost::bind(increment, &left_received_));
    right_sub_.registerCallback(boost::bind(increment, &right_received_));
    sync_.registerCallback(boost::bind(increment, &all_received_));
    check_synced_timer_ = nh.createWallTimer(ros::WallDuration(30.0),
                                             boost::bind(&Anaglyph3D::checkInputsSynchronized, this));
  }

  void imageCB(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right)
  {
    {
      boost::lock_guard<boost::mutex> guard(image_mutex_);
    
      // Hang on to message pointers for sake of mouse_cb
      last_left_ = left;
      last_right_ = right;
    }

    if (last_left_->encoding == enc::BGR8) {
      uint8_t* data_anaglyph = (uint8_t*) calloc(3*last_left_->height*last_left_->width, sizeof(uint8_t));
      for(int row = 0; row < last_left_->height; row++)
        for(int col = 0; col < last_left_->width; col++) {
          data_anaglyph[row*last_right_->step + 3*col + 0] = last_right_->data[row*last_right_->step + 3*col + 0];
          data_anaglyph[row*last_right_->step + 3*col + 1] = last_right_->data[row*last_right_->step + 3*col + 1];
          data_anaglyph[row*last_left_->step + 3*col + 2] = last_left_->data[row*last_left_->step + 3*col + 2];
        }

      fillImage(img_ana_, "bgr8", last_left_->height, last_left_->width, last_left_->step, const_cast<uint8_t*>(data_anaglyph));
      pub_anaglyph_.publish(img_ana_);
      free(data_anaglyph);
    }


  }

  void checkInputsSynchronized()
  {
    int threshold = 2 * all_received_;
    if (left_received_ > threshold || right_received_ > threshold) {
      ROS_WARN("[anaglyph] Low number of synchronized left/right/disparity triplets received.\n"
               "Left images received: %d\n"
               "Right images received: %d\n"
               "Synchronized doublets: %d\n"
               "Possible issues:\n"
               "\t* stereo_image_proc is not running.\n"
               "\t* The cameras are not synchronized.\n"
               "\t* The network is too slow. One or more images are dropped from each triplet.",
               left_received_, right_received_, all_received_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "anaglyph", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("[stereo_view] stereo has not been remapped! Example command-line usage:\n"
             "\t$ rosrun image_view stereo_view stereo:=narrow_stereo image:=image_color");
  }
  if (ros::names::remap("image") == "/image_raw") {
    ROS_WARN("[stereo_view] There is a delay between when the camera drivers publish the raw "
             "images and when stereo_image_proc publishes the computed point cloud. stereo_view "
             "may fail to synchronize these topics.");
  }
  
  Anaglyph3D ana(nh);
  
  ros::spin();
  return 0;
}
