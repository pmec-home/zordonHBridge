#ifndef _ROS_zordon_msgs_OdomCompacted_h
#define _ROS_zordon_msgs_OdomCompacted_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace zordon_msgs
{

  class OdomCompacted : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _yaw_type;
      _yaw_type yaw;
      typedef float _wl_type;
      _wl_type wl;
      typedef float _wr_type;
      _wr_type wr;

    OdomCompacted():
      x(0),
      y(0),
      yaw(0),
      wl(0),
      wr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_wl;
      u_wl.real = this->wl;
      *(outbuffer + offset + 0) = (u_wl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wl);
      union {
        float real;
        uint32_t base;
      } u_wr;
      u_wr.real = this->wr;
      *(outbuffer + offset + 0) = (u_wr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_wl;
      u_wl.base = 0;
      u_wl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wl = u_wl.real;
      offset += sizeof(this->wl);
      union {
        float real;
        uint32_t base;
      } u_wr;
      u_wr.base = 0;
      u_wr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wr = u_wr.real;
      offset += sizeof(this->wr);
     return offset;
    }

    const char * getType(){ return "zordon_msgs/OdomCompacted"; };
    const char * getMD5(){ return "4cf1fd3481a647c6f326ff92e1c8071d"; };

  };

}
#endif
