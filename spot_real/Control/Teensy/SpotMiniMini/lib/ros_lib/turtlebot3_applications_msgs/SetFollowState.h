#ifndef _ROS_SERVICE_SetFollowState_h
#define _ROS_SERVICE_SetFollowState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace turtlebot3_applications_msgs
{

static const char SETFOLLOWSTATE[] = "turtlebot3_applications_msgs/SetFollowState";

  class SetFollowStateRequest : public ros::Msg
  {
    public:
      typedef uint8_t _state_type;
      _state_type state;
      enum { STOPPED =  0 };
      enum { FOLLOW =  1 };
      enum { OK =  0 };
      enum { ERROR =  1 };

    SetFollowStateRequest():
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return SETFOLLOWSTATE; };
    const char * getMD5(){ return "bf26f08bd02c8f904098849ef5e63d81"; };

  };

  class SetFollowStateResponse : public ros::Msg
  {
    public:
      typedef uint8_t _result_type;
      _result_type result;

    SetFollowStateResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->result >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->result =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->result);
     return offset;
    }

    const char * getType(){ return SETFOLLOWSTATE; };
    const char * getMD5(){ return "25458147911545c320c4c0a299eff763"; };

  };

  class SetFollowState {
    public:
    typedef SetFollowStateRequest Request;
    typedef SetFollowStateResponse Response;
  };

}
#endif
