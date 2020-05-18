#ifndef _ROS_diff_drive_base1_WheelControl_h
#define _ROS_diff_drive_base1_WheelControl_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace diff_drive_base1
{

  class WheelControl : public ros::Msg
  {
    public:
      typedef float _left_wheel_speed_type;
      _left_wheel_speed_type left_wheel_speed;
      typedef float _right_wheel_speed_type;
      _right_wheel_speed_type right_wheel_speed;

    WheelControl():
      left_wheel_speed(0),
      right_wheel_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_speed;
      u_left_wheel_speed.real = this->left_wheel_speed;
      *(outbuffer + offset + 0) = (u_left_wheel_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel_speed);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_speed;
      u_right_wheel_speed.real = this->right_wheel_speed;
      *(outbuffer + offset + 0) = (u_right_wheel_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_speed;
      u_left_wheel_speed.base = 0;
      u_left_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel_speed = u_left_wheel_speed.real;
      offset += sizeof(this->left_wheel_speed);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_speed;
      u_right_wheel_speed.base = 0;
      u_right_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel_speed = u_right_wheel_speed.real;
      offset += sizeof(this->right_wheel_speed);
     return offset;
    }

    const char * getType(){ return "diff_drive_base1/WheelControl"; };
    const char * getMD5(){ return "d71110aac9fc7c10468c9311afc9a766"; };

  };

}
#endif
