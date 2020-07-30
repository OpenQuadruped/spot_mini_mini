#ifndef _ROS_intera_motion_msgs_MotionStatus_h
#define _ROS_intera_motion_msgs_MotionStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace intera_motion_msgs
{

  class MotionStatus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _motion_status_type;
      _motion_status_type motion_status;
      typedef const char* _current_trajectory_type;
      _current_trajectory_type current_trajectory;
      typedef uint32_t _current_waypoint_type;
      _current_waypoint_type current_waypoint;
      typedef uint32_t _motion_request_type;
      _motion_request_type motion_request;
      enum { MOTION_IDLE = idle };
      enum { MOTION_PENDING = pending };
      enum { MOTION_RUNNING = running };
      enum { MOTION_STOPPING = stopping };
      enum { MOTION_DONE = done };
      enum { MOTION_PREEMPTED = preempted };
      enum { MOTION_ERROR = error };

    MotionStatus():
      header(),
      motion_status(""),
      current_trajectory(""),
      current_waypoint(0),
      motion_request(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_motion_status = strlen(this->motion_status);
      varToArr(outbuffer + offset, length_motion_status);
      offset += 4;
      memcpy(outbuffer + offset, this->motion_status, length_motion_status);
      offset += length_motion_status;
      uint32_t length_current_trajectory = strlen(this->current_trajectory);
      varToArr(outbuffer + offset, length_current_trajectory);
      offset += 4;
      memcpy(outbuffer + offset, this->current_trajectory, length_current_trajectory);
      offset += length_current_trajectory;
      *(outbuffer + offset + 0) = (this->current_waypoint >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current_waypoint >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current_waypoint >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current_waypoint >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_waypoint);
      *(outbuffer + offset + 0) = (this->motion_request >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motion_request >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motion_request >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motion_request >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motion_request);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_motion_status;
      arrToVar(length_motion_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motion_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motion_status-1]=0;
      this->motion_status = (char *)(inbuffer + offset-1);
      offset += length_motion_status;
      uint32_t length_current_trajectory;
      arrToVar(length_current_trajectory, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_current_trajectory; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_current_trajectory-1]=0;
      this->current_trajectory = (char *)(inbuffer + offset-1);
      offset += length_current_trajectory;
      this->current_waypoint =  ((uint32_t) (*(inbuffer + offset)));
      this->current_waypoint |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_waypoint |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->current_waypoint |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->current_waypoint);
      this->motion_request =  ((uint32_t) (*(inbuffer + offset)));
      this->motion_request |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->motion_request |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->motion_request |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->motion_request);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/MotionStatus"; };
    const char * getMD5(){ return "178c2a9a52d756f9d73396be4ec1a07c"; };

  };

}
#endif
