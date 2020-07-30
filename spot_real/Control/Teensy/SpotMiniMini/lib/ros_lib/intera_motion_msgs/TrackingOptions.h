#ifndef _ROS_intera_motion_msgs_TrackingOptions_h
#define _ROS_intera_motion_msgs_TrackingOptions_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_motion_msgs
{

  class TrackingOptions : public ros::Msg
  {
    public:
      typedef bool _use_min_time_rate_type;
      _use_min_time_rate_type use_min_time_rate;
      typedef double _min_time_rate_type;
      _min_time_rate_type min_time_rate;
      typedef bool _use_max_time_rate_type;
      _use_max_time_rate_type use_max_time_rate;
      typedef double _max_time_rate_type;
      _max_time_rate_type max_time_rate;
      uint32_t goal_joint_tolerance_length;
      typedef double _goal_joint_tolerance_type;
      _goal_joint_tolerance_type st_goal_joint_tolerance;
      _goal_joint_tolerance_type * goal_joint_tolerance;
      typedef bool _use_settling_time_at_goal_type;
      _use_settling_time_at_goal_type use_settling_time_at_goal;
      typedef double _settling_time_at_goal_type;
      _settling_time_at_goal_type settling_time_at_goal;

    TrackingOptions():
      use_min_time_rate(0),
      min_time_rate(0),
      use_max_time_rate(0),
      max_time_rate(0),
      goal_joint_tolerance_length(0), goal_joint_tolerance(NULL),
      use_settling_time_at_goal(0),
      settling_time_at_goal(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_min_time_rate;
      u_use_min_time_rate.real = this->use_min_time_rate;
      *(outbuffer + offset + 0) = (u_use_min_time_rate.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_min_time_rate);
      union {
        double real;
        uint64_t base;
      } u_min_time_rate;
      u_min_time_rate.real = this->min_time_rate;
      *(outbuffer + offset + 0) = (u_min_time_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_time_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_time_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_time_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_min_time_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_min_time_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_min_time_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_min_time_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->min_time_rate);
      union {
        bool real;
        uint8_t base;
      } u_use_max_time_rate;
      u_use_max_time_rate.real = this->use_max_time_rate;
      *(outbuffer + offset + 0) = (u_use_max_time_rate.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_max_time_rate);
      union {
        double real;
        uint64_t base;
      } u_max_time_rate;
      u_max_time_rate.real = this->max_time_rate;
      *(outbuffer + offset + 0) = (u_max_time_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_time_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_time_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_time_rate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_time_rate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_time_rate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_time_rate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_time_rate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_time_rate);
      *(outbuffer + offset + 0) = (this->goal_joint_tolerance_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_joint_tolerance_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_joint_tolerance_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_joint_tolerance_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_joint_tolerance_length);
      for( uint32_t i = 0; i < goal_joint_tolerance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_goal_joint_tolerancei;
      u_goal_joint_tolerancei.real = this->goal_joint_tolerance[i];
      *(outbuffer + offset + 0) = (u_goal_joint_tolerancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_goal_joint_tolerancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_goal_joint_tolerancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_goal_joint_tolerancei.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_goal_joint_tolerancei.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_goal_joint_tolerancei.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_goal_joint_tolerancei.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_goal_joint_tolerancei.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->goal_joint_tolerance[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_use_settling_time_at_goal;
      u_use_settling_time_at_goal.real = this->use_settling_time_at_goal;
      *(outbuffer + offset + 0) = (u_use_settling_time_at_goal.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_settling_time_at_goal);
      union {
        double real;
        uint64_t base;
      } u_settling_time_at_goal;
      u_settling_time_at_goal.real = this->settling_time_at_goal;
      *(outbuffer + offset + 0) = (u_settling_time_at_goal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_settling_time_at_goal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_settling_time_at_goal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_settling_time_at_goal.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_settling_time_at_goal.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_settling_time_at_goal.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_settling_time_at_goal.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_settling_time_at_goal.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->settling_time_at_goal);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_min_time_rate;
      u_use_min_time_rate.base = 0;
      u_use_min_time_rate.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_min_time_rate = u_use_min_time_rate.real;
      offset += sizeof(this->use_min_time_rate);
      union {
        double real;
        uint64_t base;
      } u_min_time_rate;
      u_min_time_rate.base = 0;
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_min_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->min_time_rate = u_min_time_rate.real;
      offset += sizeof(this->min_time_rate);
      union {
        bool real;
        uint8_t base;
      } u_use_max_time_rate;
      u_use_max_time_rate.base = 0;
      u_use_max_time_rate.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_max_time_rate = u_use_max_time_rate.real;
      offset += sizeof(this->use_max_time_rate);
      union {
        double real;
        uint64_t base;
      } u_max_time_rate;
      u_max_time_rate.base = 0;
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_time_rate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_time_rate = u_max_time_rate.real;
      offset += sizeof(this->max_time_rate);
      uint32_t goal_joint_tolerance_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goal_joint_tolerance_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goal_joint_tolerance_length);
      if(goal_joint_tolerance_lengthT > goal_joint_tolerance_length)
        this->goal_joint_tolerance = (double*)realloc(this->goal_joint_tolerance, goal_joint_tolerance_lengthT * sizeof(double));
      goal_joint_tolerance_length = goal_joint_tolerance_lengthT;
      for( uint32_t i = 0; i < goal_joint_tolerance_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_goal_joint_tolerance;
      u_st_goal_joint_tolerance.base = 0;
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_goal_joint_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_goal_joint_tolerance = u_st_goal_joint_tolerance.real;
      offset += sizeof(this->st_goal_joint_tolerance);
        memcpy( &(this->goal_joint_tolerance[i]), &(this->st_goal_joint_tolerance), sizeof(double));
      }
      union {
        bool real;
        uint8_t base;
      } u_use_settling_time_at_goal;
      u_use_settling_time_at_goal.base = 0;
      u_use_settling_time_at_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_settling_time_at_goal = u_use_settling_time_at_goal.real;
      offset += sizeof(this->use_settling_time_at_goal);
      union {
        double real;
        uint64_t base;
      } u_settling_time_at_goal;
      u_settling_time_at_goal.base = 0;
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_settling_time_at_goal.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->settling_time_at_goal = u_settling_time_at_goal.real;
      offset += sizeof(this->settling_time_at_goal);
     return offset;
    }

    const char * getType(){ return "intera_motion_msgs/TrackingOptions"; };
    const char * getMD5(){ return "e848e8a266b514c3bde707d0e1859055"; };

  };

}
#endif
