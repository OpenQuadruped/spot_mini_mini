#ifndef _ROS_baxter_core_msgs_EndEffectorState_h
#define _ROS_baxter_core_msgs_EndEffectorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace baxter_core_msgs
{

  class EndEffectorState : public ros::Msg
  {
    public:
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef uint32_t _id_type;
      _id_type id;
      typedef uint8_t _enabled_type;
      _enabled_type enabled;
      typedef uint8_t _calibrated_type;
      _calibrated_type calibrated;
      typedef uint8_t _ready_type;
      _ready_type ready;
      typedef uint8_t _moving_type;
      _moving_type moving;
      typedef uint8_t _gripping_type;
      _gripping_type gripping;
      typedef uint8_t _missed_type;
      _missed_type missed;
      typedef uint8_t _error_type;
      _error_type error;
      typedef uint8_t _reverse_type;
      _reverse_type reverse;
      typedef float _position_type;
      _position_type position;
      typedef float _force_type;
      _force_type force;
      typedef const char* _state_type;
      _state_type state;
      typedef const char* _command_type;
      _command_type command;
      typedef const char* _command_sender_type;
      _command_sender_type command_sender;
      typedef uint32_t _command_sequence_type;
      _command_sequence_type command_sequence;
      enum { STATE_FALSE =  0 };
      enum { STATE_TRUE =  1 };
      enum { STATE_UNKNOWN =  2 };
      enum { POSITION_CLOSED =  0.0 };
      enum { POSITION_OPEN =  100.0 };
      enum { FORCE_MIN =  0.0 };
      enum { FORCE_MAX =  100.0 };

    EndEffectorState():
      timestamp(),
      id(0),
      enabled(0),
      calibrated(0),
      ready(0),
      moving(0),
      gripping(0),
      missed(0),
      error(0),
      reverse(0),
      position(0),
      force(0),
      state(""),
      command(""),
      command_sender(""),
      command_sequence(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      *(outbuffer + offset + 0) = (this->enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      *(outbuffer + offset + 0) = (this->calibrated >> (8 * 0)) & 0xFF;
      offset += sizeof(this->calibrated);
      *(outbuffer + offset + 0) = (this->ready >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ready);
      *(outbuffer + offset + 0) = (this->moving >> (8 * 0)) & 0xFF;
      offset += sizeof(this->moving);
      *(outbuffer + offset + 0) = (this->gripping >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gripping);
      *(outbuffer + offset + 0) = (this->missed >> (8 * 0)) & 0xFF;
      offset += sizeof(this->missed);
      *(outbuffer + offset + 0) = (this->error >> (8 * 0)) & 0xFF;
      offset += sizeof(this->error);
      *(outbuffer + offset + 0) = (this->reverse >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reverse);
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_force;
      u_force.real = this->force;
      *(outbuffer + offset + 0) = (u_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->force);
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      uint32_t length_command_sender = strlen(this->command_sender);
      varToArr(outbuffer + offset, length_command_sender);
      offset += 4;
      memcpy(outbuffer + offset, this->command_sender, length_command_sender);
      offset += length_command_sender;
      *(outbuffer + offset + 0) = (this->command_sequence >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->command_sequence >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->command_sequence >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->command_sequence >> (8 * 3)) & 0xFF;
      offset += sizeof(this->command_sequence);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      this->enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->enabled);
      this->calibrated =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->calibrated);
      this->ready =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ready);
      this->moving =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->moving);
      this->gripping =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gripping);
      this->missed =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->missed);
      this->error =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->error);
      this->reverse =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reverse);
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        float real;
        uint32_t base;
      } u_force;
      u_force.base = 0;
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->force = u_force.real;
      offset += sizeof(this->force);
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      uint32_t length_command_sender;
      arrToVar(length_command_sender, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command_sender; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command_sender-1]=0;
      this->command_sender = (char *)(inbuffer + offset-1);
      offset += length_command_sender;
      this->command_sequence =  ((uint32_t) (*(inbuffer + offset)));
      this->command_sequence |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->command_sequence |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->command_sequence |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->command_sequence);
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/EndEffectorState"; };
    const char * getMD5(){ return "ade777f069d738595bc19e246b8ec7a0"; };

  };

}
#endif
