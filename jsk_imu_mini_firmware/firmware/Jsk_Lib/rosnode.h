#ifndef __ROS_NODE_H_
#define __ROS_NODE_H_

#include <vector>

/* ros */
#include <ros.h>
/* inlcude ros message types*/
#include <jsk_imu_mini_msgs/Potentio.h>

class RosNode
{
public:
  RosNode(ros::NodeHandle* nh)
  {
    /* ros node initialize */
    nh_ = nh;
    /* ros publisher initialize */
    potentio_pub = new ros::Publisher("potentio/raw", &potentio_msg);
    // publish the msgs
    nh_->advertise(*potentio_pub);
    potentio_msg.potentio_length = 4;
    potentio_msg.potentio = (uint16_t*)malloc(sizeof(uint16_t) * 4);

    /* ros subscriber initialize */
  }

  void publish(uint16_t arr[])
  {
    potentio_msg.stamp = nh_->now();
    for (int i = 0; i < 4; i++)
      potentio_msg.potentio[i] = arr[i];
    potentio_pub->publish(&potentio_msg);
  }

  ~RosNode()
  {
  }

  /* ros node handle */
  ros::NodeHandle* nh_;
  /* ros publisher */
  ros::Publisher* potentio_pub;
  /*  ros msgs */
  jsk_imu_mini_msgs::Potentio potentio_msg;

private:
};
#endif
