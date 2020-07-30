#ifndef _ROS_intera_core_msgs_HomingState_h
#define _ROS_intera_core_msgs_HomingState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class HomingState : public ros::Msg
  {
    public:
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t state_length;
      typedef int32_t _state_type;
      _state_type st_state;
      _state_type * state;
      enum { HOMED = 0 };
      enum { HOMING = 1 };
      enum { NOT_HOMED = 2 };

    HomingState():
      name_length(0), name(NULL),
      state_length(0), state(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state_length);
      for( uint32_t i = 0; i < state_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_statei;
      u_statei.real = this->state[i];
      *(outbuffer + offset + 0) = (u_statei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_statei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_statei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_statei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->state[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->state_length);
      if(state_lengthT > state_length)
        this->state = (int32_t*)realloc(this->state, state_lengthT * sizeof(int32_t));
      state_length = state_lengthT;
      for( uint32_t i = 0; i < state_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_state;
      u_st_state.base = 0;
      u_st_state.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_state.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_state.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_state.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_state = u_st_state.real;
      offset += sizeof(this->st_state);
        memcpy( &(this->state[i]), &(this->st_state), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/HomingState"; };
    const char * getMD5(){ return "eacb82bc7d74638daa749d9caab52b99"; };

  };

}
#endif
