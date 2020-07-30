#ifndef _ROS_SERVICE_IOComponentCommandSrv_h
#define _ROS_SERVICE_IOComponentCommandSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "intera_core_msgs/IOStatus.h"
#include "ros/time.h"
#include "intera_core_msgs/IOComponentCommand.h"

namespace intera_core_msgs
{

static const char IOCOMPONENTCOMMANDSRV[] = "intera_core_msgs/IOComponentCommandSrv";

  class IOComponentCommandSrvRequest : public ros::Msg
  {
    public:
      typedef intera_core_msgs::IOComponentCommand _command_type;
      _command_type command;
      typedef float _timeout_type;
      _timeout_type timeout;

    IOComponentCommandSrvRequest():
      command(),
      timeout(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeout.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timeout.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timeout.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeout.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_timeout.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_timeout.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
     return offset;
    }

    const char * getType(){ return IOCOMPONENTCOMMANDSRV; };
    const char * getMD5(){ return "f1576f0935f7d90abe14dbedf574be71"; };

  };

  class IOComponentCommandSrvResponse : public ros::Msg
  {
    public:
      typedef ros::Time _time_type;
      _time_type time;
      typedef const char* _op_type;
      _op_type op;
      typedef intera_core_msgs::IOStatus _status_type;
      _status_type status;
      typedef const char* _response_type;
      _response_type response;

    IOComponentCommandSrvResponse():
      time(),
      op(""),
      status(),
      response("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      uint32_t length_op = strlen(this->op);
      varToArr(outbuffer + offset, length_op);
      offset += 4;
      memcpy(outbuffer + offset, this->op, length_op);
      offset += length_op;
      offset += this->status.serialize(outbuffer + offset);
      uint32_t length_response = strlen(this->response);
      varToArr(outbuffer + offset, length_response);
      offset += 4;
      memcpy(outbuffer + offset, this->response, length_response);
      offset += length_response;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
      uint32_t length_op;
      arrToVar(length_op, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_op; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_op-1]=0;
      this->op = (char *)(inbuffer + offset-1);
      offset += length_op;
      offset += this->status.deserialize(inbuffer + offset);
      uint32_t length_response;
      arrToVar(length_response, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_response-1]=0;
      this->response = (char *)(inbuffer + offset-1);
      offset += length_response;
     return offset;
    }

    const char * getType(){ return IOCOMPONENTCOMMANDSRV; };
    const char * getMD5(){ return "201c021e76b3e871e24d4b9fd38ffd49"; };

  };

  class IOComponentCommandSrv {
    public:
    typedef IOComponentCommandSrvRequest Request;
    typedef IOComponentCommandSrvResponse Response;
  };

}
#endif
