#ifndef _ROS_intera_motion_msgs_EndpointTrackingError_h
#define _ROS_intera_motion_msgs_EndpointTrackingError_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace intera_motion_msgs
{

  class EndpointTrackingError : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _trajectory_id_type;
      _trajectory_id_type trajectory_id;
      typedef double _trajectory_time_type;
      _trajectory_time_type trajectory_time;
      typedef const char* _active_endpoint_type;
      _active_endpoint_type active_endpoint;
      typedef geometry_msgs::Pose _pose_error_type;
      _pose_error_type pose_error;
      typedef geometry_msgs::Twist _twist_error_type;
      _twist_error_type twist_error;
      typedef geometry_msgs::Pose _pose_command_type;
      _pose_command_type pose_command;
      typedef geometry_msgs::Twist _twist_command_type;
      _twist_command_type twist_command;

    EndpointTrackingError():
      header(),
      trajectory_id(0),
      trajectory_time(0),
      active_endpoint(""),
      pose_error(),
      twist_error(),
      pose_command(),
      twist_command()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->trajectory_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->trajectory_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->trajectory_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->trajectory_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->trajectory_id);
      union {
        double real;
        uint64_t base;
      } u_trajectory_time;
      u_trajectory_time.real = this->trajectory_time;
      *(outbuffer + offset + 0) = (u_trajectory_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_trajectory_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_trajectory_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_trajectory_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_trajectory_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_trajectory_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_trajectory_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_trajectory_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->trajectory_time);
      uint32_t length_active_endpoint = strlen(this->active_endpoint);
      varToArr(outbuffer + offset, length_active_endpoint);
      offset += 4;
      memcpy(outbuffer + offset, this->active_endpoint, length_active_endpoint);
      offset += length_active_endpoint;
      offset += this->pose_error.serialize(outbuffer + offset);
      offset += this->twist_error.serialize(outbuffer + offset);
      offset += this->pose_command.serialize(outbuffer + offset);
      offset += this->twist_command.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->trajectory_id =  ((uint32_t) (*(inbuffer + offset)));
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->trajectory_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->trajectory_id);
      union {
        double real;
        uint64_t base;
      } u_trajectory_time;
      u_trajectory_time.base = 0;
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_trajectory_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->trajectory_time = u_trajectory_time.real;
      offset += sizeof(this->trajectory_time);
      uint32_t length_active_endpoint;
      arrToVar(length_active_endpoint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_active_endpoint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_active_endpoint-1]=0;
      this->active_endpoint = (char *)(inbuffer + offset-1);
      offset += length_active_endpoint;
      offset += this->pose_error.deserialize(inbuffer + offset);
      offset += this->twist_error.deserialize(inbuffer + offset);
      offset += this->pose_command.deserialize(inbuffer + offset);
      offset += this->twist_command.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/EndpointTrackingError"; };
    const char * getMD5(){ return "c266cbf60fc5f871b5909394c05032fc"; };

  };

}
#endif
