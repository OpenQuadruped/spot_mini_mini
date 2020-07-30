#ifndef _ROS_flexbe_msgs_BehaviorSynthesisGoal_h
#define _ROS_flexbe_msgs_BehaviorSynthesisGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/SynthesisRequest.h"

namespace flexbe_msgs
{

  class BehaviorSynthesisGoal : public ros::Msg
  {
    public:
      typedef flexbe_msgs::SynthesisRequest _request_type;
      _request_type request;

    BehaviorSynthesisGoal():
      request()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->request.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->request.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorSynthesisGoal"; };
    const char * getMD5(){ return "64ccf8fdad6091a950ca099bc67e6595"; };

  };

}
#endif
