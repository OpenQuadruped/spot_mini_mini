#ifndef _ROS_SERVICE_SolvePositionIK_h
#define _ROS_SERVICE_SolvePositionIK_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

namespace intera_core_msgs
{

static const char SOLVEPOSITIONIK[] = "intera_core_msgs/SolvePositionIK";

  class SolvePositionIKRequest : public ros::Msg
  {
    public:
      uint32_t pose_stamp_length;
      typedef geometry_msgs::PoseStamped _pose_stamp_type;
      _pose_stamp_type st_pose_stamp;
      _pose_stamp_type * pose_stamp;
      uint32_t seed_angles_length;
      typedef sensor_msgs::JointState _seed_angles_type;
      _seed_angles_type st_seed_angles;
      _seed_angles_type * seed_angles;
      typedef int8_t _seed_mode_type;
      _seed_mode_type seed_mode;
      uint32_t use_nullspace_goal_length;
      typedef bool _use_nullspace_goal_type;
      _use_nullspace_goal_type st_use_nullspace_goal;
      _use_nullspace_goal_type * use_nullspace_goal;
      uint32_t nullspace_goal_length;
      typedef sensor_msgs::JointState _nullspace_goal_type;
      _nullspace_goal_type st_nullspace_goal;
      _nullspace_goal_type * nullspace_goal;
      uint32_t nullspace_gain_length;
      typedef double _nullspace_gain_type;
      _nullspace_gain_type st_nullspace_gain;
      _nullspace_gain_type * nullspace_gain;
      uint32_t tip_names_length;
      typedef char* _tip_names_type;
      _tip_names_type st_tip_names;
      _tip_names_type * tip_names;
      enum { SEED_AUTO =  0 };
      enum { SEED_USER =  1 };
      enum { SEED_CURRENT =  2 };
      enum { SEED_NS_MAP =  3 };

    SolvePositionIKRequest():
      pose_stamp_length(0), pose_stamp(NULL),
      seed_angles_length(0), seed_angles(NULL),
      seed_mode(0),
      use_nullspace_goal_length(0), use_nullspace_goal(NULL),
      nullspace_goal_length(0), nullspace_goal(NULL),
      nullspace_gain_length(0), nullspace_gain(NULL),
      tip_names_length(0), tip_names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->pose_stamp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->pose_stamp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->pose_stamp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->pose_stamp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_stamp_length);
      for( uint32_t i = 0; i < pose_stamp_length; i++){
      offset += this->pose_stamp[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->seed_angles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seed_angles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seed_angles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seed_angles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seed_angles_length);
      for( uint32_t i = 0; i < seed_angles_length; i++){
      offset += this->seed_angles[i].serialize(outbuffer + offset);
      }
      union {
        int8_t real;
        uint8_t base;
      } u_seed_mode;
      u_seed_mode.real = this->seed_mode;
      *(outbuffer + offset + 0) = (u_seed_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->seed_mode);
      *(outbuffer + offset + 0) = (this->use_nullspace_goal_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->use_nullspace_goal_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->use_nullspace_goal_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->use_nullspace_goal_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->use_nullspace_goal_length);
      for( uint32_t i = 0; i < use_nullspace_goal_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_use_nullspace_goali;
      u_use_nullspace_goali.real = this->use_nullspace_goal[i];
      *(outbuffer + offset + 0) = (u_use_nullspace_goali.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_nullspace_goal[i]);
      }
      *(outbuffer + offset + 0) = (this->nullspace_goal_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nullspace_goal_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nullspace_goal_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nullspace_goal_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nullspace_goal_length);
      for( uint32_t i = 0; i < nullspace_goal_length; i++){
      offset += this->nullspace_goal[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->nullspace_gain_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->nullspace_gain_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->nullspace_gain_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->nullspace_gain_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nullspace_gain_length);
      for( uint32_t i = 0; i < nullspace_gain_length; i++){
      union {
        double real;
        uint64_t base;
      } u_nullspace_gaini;
      u_nullspace_gaini.real = this->nullspace_gain[i];
      *(outbuffer + offset + 0) = (u_nullspace_gaini.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nullspace_gaini.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nullspace_gaini.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nullspace_gaini.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nullspace_gaini.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nullspace_gaini.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nullspace_gaini.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nullspace_gaini.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nullspace_gain[i]);
      }
      *(outbuffer + offset + 0) = (this->tip_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tip_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->tip_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->tip_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tip_names_length);
      for( uint32_t i = 0; i < tip_names_length; i++){
      uint32_t length_tip_namesi = strlen(this->tip_names[i]);
      varToArr(outbuffer + offset, length_tip_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->tip_names[i], length_tip_namesi);
      offset += length_tip_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t pose_stamp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      pose_stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      pose_stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      pose_stamp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->pose_stamp_length);
      if(pose_stamp_lengthT > pose_stamp_length)
        this->pose_stamp = (geometry_msgs::PoseStamped*)realloc(this->pose_stamp, pose_stamp_lengthT * sizeof(geometry_msgs::PoseStamped));
      pose_stamp_length = pose_stamp_lengthT;
      for( uint32_t i = 0; i < pose_stamp_length; i++){
      offset += this->st_pose_stamp.deserialize(inbuffer + offset);
        memcpy( &(this->pose_stamp[i]), &(this->st_pose_stamp), sizeof(geometry_msgs::PoseStamped));
      }
      uint32_t seed_angles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      seed_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      seed_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      seed_angles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->seed_angles_length);
      if(seed_angles_lengthT > seed_angles_length)
        this->seed_angles = (sensor_msgs::JointState*)realloc(this->seed_angles, seed_angles_lengthT * sizeof(sensor_msgs::JointState));
      seed_angles_length = seed_angles_lengthT;
      for( uint32_t i = 0; i < seed_angles_length; i++){
      offset += this->st_seed_angles.deserialize(inbuffer + offset);
        memcpy( &(this->seed_angles[i]), &(this->st_seed_angles), sizeof(sensor_msgs::JointState));
      }
      union {
        int8_t real;
        uint8_t base;
      } u_seed_mode;
      u_seed_mode.base = 0;
      u_seed_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->seed_mode = u_seed_mode.real;
      offset += sizeof(this->seed_mode);
      uint32_t use_nullspace_goal_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      use_nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      use_nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      use_nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->use_nullspace_goal_length);
      if(use_nullspace_goal_lengthT > use_nullspace_goal_length)
        this->use_nullspace_goal = (bool*)realloc(this->use_nullspace_goal, use_nullspace_goal_lengthT * sizeof(bool));
      use_nullspace_goal_length = use_nullspace_goal_lengthT;
      for( uint32_t i = 0; i < use_nullspace_goal_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_use_nullspace_goal;
      u_st_use_nullspace_goal.base = 0;
      u_st_use_nullspace_goal.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_use_nullspace_goal = u_st_use_nullspace_goal.real;
      offset += sizeof(this->st_use_nullspace_goal);
        memcpy( &(this->use_nullspace_goal[i]), &(this->st_use_nullspace_goal), sizeof(bool));
      }
      uint32_t nullspace_goal_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nullspace_goal_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nullspace_goal_length);
      if(nullspace_goal_lengthT > nullspace_goal_length)
        this->nullspace_goal = (sensor_msgs::JointState*)realloc(this->nullspace_goal, nullspace_goal_lengthT * sizeof(sensor_msgs::JointState));
      nullspace_goal_length = nullspace_goal_lengthT;
      for( uint32_t i = 0; i < nullspace_goal_length; i++){
      offset += this->st_nullspace_goal.deserialize(inbuffer + offset);
        memcpy( &(this->nullspace_goal[i]), &(this->st_nullspace_goal), sizeof(sensor_msgs::JointState));
      }
      uint32_t nullspace_gain_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      nullspace_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      nullspace_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      nullspace_gain_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->nullspace_gain_length);
      if(nullspace_gain_lengthT > nullspace_gain_length)
        this->nullspace_gain = (double*)realloc(this->nullspace_gain, nullspace_gain_lengthT * sizeof(double));
      nullspace_gain_length = nullspace_gain_lengthT;
      for( uint32_t i = 0; i < nullspace_gain_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_nullspace_gain;
      u_st_nullspace_gain.base = 0;
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_nullspace_gain.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_nullspace_gain = u_st_nullspace_gain.real;
      offset += sizeof(this->st_nullspace_gain);
        memcpy( &(this->nullspace_gain[i]), &(this->st_nullspace_gain), sizeof(double));
      }
      uint32_t tip_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      tip_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      tip_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      tip_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->tip_names_length);
      if(tip_names_lengthT > tip_names_length)
        this->tip_names = (char**)realloc(this->tip_names, tip_names_lengthT * sizeof(char*));
      tip_names_length = tip_names_lengthT;
      for( uint32_t i = 0; i < tip_names_length; i++){
      uint32_t length_st_tip_names;
      arrToVar(length_st_tip_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_tip_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_tip_names-1]=0;
      this->st_tip_names = (char *)(inbuffer + offset-1);
      offset += length_st_tip_names;
        memcpy( &(this->tip_names[i]), &(this->st_tip_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return SOLVEPOSITIONIK; };
    const char * getMD5(){ return "b09dd99695bb18639bfea7c92d0a89ca"; };

  };

  class SolvePositionIKResponse : public ros::Msg
  {
    public:
      uint32_t joints_length;
      typedef sensor_msgs::JointState _joints_type;
      _joints_type st_joints;
      _joints_type * joints;
      uint32_t result_type_length;
      typedef int8_t _result_type_type;
      _result_type_type st_result_type;
      _result_type_type * result_type;
      enum { IK_FAILED =  -1 };
      enum { IK_IN_COLLISION =  -2 };
      enum { IK_ENDPOINT_DOES_NOT_EXIST =  -3 };

    SolvePositionIKResponse():
      joints_length(0), joints(NULL),
      result_type_length(0), result_type(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joints_length);
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->joints[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->result_type_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->result_type_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->result_type_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->result_type_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->result_type_length);
      for( uint32_t i = 0; i < result_type_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_result_typei;
      u_result_typei.real = this->result_type[i];
      *(outbuffer + offset + 0) = (u_result_typei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result_type[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joints_length);
      if(joints_lengthT > joints_length)
        this->joints = (sensor_msgs::JointState*)realloc(this->joints, joints_lengthT * sizeof(sensor_msgs::JointState));
      joints_length = joints_lengthT;
      for( uint32_t i = 0; i < joints_length; i++){
      offset += this->st_joints.deserialize(inbuffer + offset);
        memcpy( &(this->joints[i]), &(this->st_joints), sizeof(sensor_msgs::JointState));
      }
      uint32_t result_type_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      result_type_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      result_type_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      result_type_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->result_type_length);
      if(result_type_lengthT > result_type_length)
        this->result_type = (int8_t*)realloc(this->result_type, result_type_lengthT * sizeof(int8_t));
      result_type_length = result_type_lengthT;
      for( uint32_t i = 0; i < result_type_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_result_type;
      u_st_result_type.base = 0;
      u_st_result_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_result_type = u_st_result_type.real;
      offset += sizeof(this->st_result_type);
        memcpy( &(this->result_type[i]), &(this->st_result_type), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return SOLVEPOSITIONIK; };
    const char * getMD5(){ return "d47b4ca70898ba1b2f99ffdf9b81d911"; };

  };

  class SolvePositionIK {
    public:
    typedef SolvePositionIKRequest Request;
    typedef SolvePositionIKResponse Response;
  };

}
#endif
