#ifndef _ROS_SERVICE_SolvePositionFK_h
#define _ROS_SERVICE_SolvePositionFK_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

namespace intera_core_msgs
{

static const char SOLVEPOSITIONFK[] = "intera_core_msgs/SolvePositionFK";

  class SolvePositionFKRequest : public ros::Msg
  {
    public:
      uint32_t configuration_length;
      typedef sensor_msgs::JointState _configuration_type;
      _configuration_type st_configuration;
      _configuration_type * configuration;
      uint32_t tip_names_length;
      typedef char* _tip_names_type;
      _tip_names_type st_tip_names;
      _tip_names_type * tip_names;

    SolvePositionFKRequest():
      configuration_length(0), configuration(NULL),
      tip_names_length(0), tip_names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->configuration_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->configuration_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->configuration_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->configuration_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->configuration_length);
      for( uint32_t i = 0; i < configuration_length; i++){
      offset += this->configuration[i].serialize(outbuffer + offset);
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
      uint32_t configuration_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      configuration_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      configuration_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      configuration_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->configuration_length);
      if(configuration_lengthT > configuration_length)
        this->configuration = (sensor_msgs::JointState*)realloc(this->configuration, configuration_lengthT * sizeof(sensor_msgs::JointState));
      configuration_length = configuration_lengthT;
      for( uint32_t i = 0; i < configuration_length; i++){
      offset += this->st_configuration.deserialize(inbuffer + offset);
        memcpy( &(this->configuration[i]), &(this->st_configuration), sizeof(sensor_msgs::JointState));
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

    const char * getType(){ return SOLVEPOSITIONFK; };
    const char * getMD5(){ return "14c88cbabc4e4d6c183969e91f5e56ca"; };

  };

  class SolvePositionFKResponse : public ros::Msg
  {
    public:
      uint32_t pose_stamp_length;
      typedef geometry_msgs::PoseStamped _pose_stamp_type;
      _pose_stamp_type st_pose_stamp;
      _pose_stamp_type * pose_stamp;
      uint32_t isValid_length;
      typedef bool _isValid_type;
      _isValid_type st_isValid;
      _isValid_type * isValid;
      uint32_t inCollision_length;
      typedef bool _inCollision_type;
      _inCollision_type st_inCollision;
      _inCollision_type * inCollision;

    SolvePositionFKResponse():
      pose_stamp_length(0), pose_stamp(NULL),
      isValid_length(0), isValid(NULL),
      inCollision_length(0), inCollision(NULL)
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
      *(outbuffer + offset + 0) = (this->isValid_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->isValid_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->isValid_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->isValid_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->isValid_length);
      for( uint32_t i = 0; i < isValid_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_isValidi;
      u_isValidi.real = this->isValid[i];
      *(outbuffer + offset + 0) = (u_isValidi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isValid[i]);
      }
      *(outbuffer + offset + 0) = (this->inCollision_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->inCollision_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->inCollision_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->inCollision_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->inCollision_length);
      for( uint32_t i = 0; i < inCollision_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_inCollisioni;
      u_inCollisioni.real = this->inCollision[i];
      *(outbuffer + offset + 0) = (u_inCollisioni.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->inCollision[i]);
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
      uint32_t isValid_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      isValid_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      isValid_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      isValid_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->isValid_length);
      if(isValid_lengthT > isValid_length)
        this->isValid = (bool*)realloc(this->isValid, isValid_lengthT * sizeof(bool));
      isValid_length = isValid_lengthT;
      for( uint32_t i = 0; i < isValid_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_isValid;
      u_st_isValid.base = 0;
      u_st_isValid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_isValid = u_st_isValid.real;
      offset += sizeof(this->st_isValid);
        memcpy( &(this->isValid[i]), &(this->st_isValid), sizeof(bool));
      }
      uint32_t inCollision_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      inCollision_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      inCollision_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      inCollision_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->inCollision_length);
      if(inCollision_lengthT > inCollision_length)
        this->inCollision = (bool*)realloc(this->inCollision, inCollision_lengthT * sizeof(bool));
      inCollision_length = inCollision_lengthT;
      for( uint32_t i = 0; i < inCollision_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_inCollision;
      u_st_inCollision.base = 0;
      u_st_inCollision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_inCollision = u_st_inCollision.real;
      offset += sizeof(this->st_inCollision);
        memcpy( &(this->inCollision[i]), &(this->st_inCollision), sizeof(bool));
      }
     return offset;
    }

    const char * getType(){ return SOLVEPOSITIONFK; };
    const char * getMD5(){ return "907cf9ee4b255127ce59627076bd1e85"; };

  };

  class SolvePositionFK {
    public:
    typedef SolvePositionFKRequest Request;
    typedef SolvePositionFKResponse Response;
  };

}
#endif
