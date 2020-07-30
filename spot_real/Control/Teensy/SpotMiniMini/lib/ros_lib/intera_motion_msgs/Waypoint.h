#ifndef _ROS_intera_motion_msgs_Waypoint_h
#define _ROS_intera_motion_msgs_Waypoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "intera_motion_msgs/WaypointOptions.h"

namespace intera_motion_msgs
{

  class Waypoint : public ros::Msg
  {
    public:
      uint32_t joint_positions_length;
      typedef double _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;
      typedef const char* _active_endpoint_type;
      _active_endpoint_type active_endpoint;
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;
      typedef intera_motion_msgs::WaypointOptions _options_type;
      _options_type options;

    Waypoint():
      joint_positions_length(0), joint_positions(NULL),
      active_endpoint(""),
      pose(),
      options()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_positions_length);
      for( uint32_t i = 0; i < joint_positions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_positionsi;
      u_joint_positionsi.real = this->joint_positions[i];
      *(outbuffer + offset + 0) = (u_joint_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_positionsi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_positionsi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_positionsi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_positionsi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_positionsi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_positions[i]);
      }
      uint32_t length_active_endpoint = strlen(this->active_endpoint);
      varToArr(outbuffer + offset, length_active_endpoint);
      offset += 4;
      memcpy(outbuffer + offset, this->active_endpoint, length_active_endpoint);
      offset += length_active_endpoint;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->options.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_positions_length);
      if(joint_positions_lengthT > joint_positions_length)
        this->joint_positions = (double*)realloc(this->joint_positions, joint_positions_lengthT * sizeof(double));
      joint_positions_length = joint_positions_lengthT;
      for( uint32_t i = 0; i < joint_positions_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_positions;
      u_st_joint_positions.base = 0;
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_positions.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_positions = u_st_joint_positions.real;
      offset += sizeof(this->st_joint_positions);
        memcpy( &(this->joint_positions[i]), &(this->st_joint_positions), sizeof(double));
      }
      uint32_t length_active_endpoint;
      arrToVar(length_active_endpoint, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_active_endpoint; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_active_endpoint-1]=0;
      this->active_endpoint = (char *)(inbuffer + offset-1);
      offset += length_active_endpoint;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->options.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/Waypoint"; };
    const char * getMD5(){ return "8284b290b22204acc5e4d8000467b033"; };

  };

}
#endif
