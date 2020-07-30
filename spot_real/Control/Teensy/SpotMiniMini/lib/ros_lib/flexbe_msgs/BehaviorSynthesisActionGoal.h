#ifndef _ROS_flexbe_msgs_BehaviorSynthesisActionGoal_h
#define _ROS_flexbe_msgs_BehaviorSynthesisActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "flexbe_msgs/BehaviorSynthesisGoal.h"

namespace flexbe_msgs
{

  class BehaviorSynthesisActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef flexbe_msgs::BehaviorSynthesisGoal _goal_type;
      _goal_type goal;

    BehaviorSynthesisActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/BehaviorSynthesisActionGoal"; };
    const char * getMD5(){ return "ef3bccd0f7dc68c4fe76a2cb791126b0"; };

  };

}
#endif
