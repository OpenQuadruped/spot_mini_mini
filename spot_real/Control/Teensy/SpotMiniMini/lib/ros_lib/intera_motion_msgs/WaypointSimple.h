#ifndef _ROS_intera_motion_msgs_WaypointSimple_h
#define _ROS_intera_motion_msgs_WaypointSimple_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace intera_motion_msgs
{

  class WaypointSimple : public ros::Msg
  {
    public:
      uint32_t joint_positions_length;
      typedef double _joint_positions_type;
      _joint_positions_type st_joint_positions;
      _joint_positions_type * joint_positions;
      typedef const char* _active_endpoint_type;
      _active_endpoint_type active_endpoint;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef int32_t _segment_index_type;
      _segment_index_type segment_index;
      typedef double _time_type;
      _time_type time;

    WaypointSimple():
      joint_positions_length(0), joint_positions(NULL),
      active_endpoint(""),
      pose(),
      segment_index(0),
      time(0)
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
      union {
        int32_t real;
        uint32_t base;
      } u_segment_index;
      u_segment_index.real = this->segment_index;
      *(outbuffer + offset + 0) = (u_segment_index.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_segment_index.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_segment_index.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_segment_index.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->segment_index);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
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
      union {
        int32_t real;
        uint32_t base;
      } u_segment_index;
      u_segment_index.base = 0;
      u_segment_index.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_segment_index.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_segment_index.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_segment_index.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->segment_index = u_segment_index.real;
      offset += sizeof(this->segment_index);
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/WaypointSimple"; };
    const char * getMD5(){ return "f29bcd94cca5f378ef52eb965645d7ce"; };

  };

}
#endif
