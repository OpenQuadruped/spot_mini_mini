#ifndef _ROS_intera_core_msgs_EndpointNamesArray_h
#define _ROS_intera_core_msgs_EndpointNamesArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace intera_core_msgs
{

  class EndpointNamesArray : public ros::Msg
  {
    public:
      uint32_t endpoint_names_length;
      typedef char* _endpoint_names_type;
      _endpoint_names_type st_endpoint_names;
      _endpoint_names_type * endpoint_names;

    EndpointNamesArray():
      endpoint_names_length(0), endpoint_names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->endpoint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->endpoint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->endpoint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->endpoint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->endpoint_names_length);
      for( uint32_t i = 0; i < endpoint_names_length; i++){
      uint32_t length_endpoint_namesi = strlen(this->endpoint_names[i]);
      varToArr(outbuffer + offset, length_endpoint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->endpoint_names[i], length_endpoint_namesi);
      offset += length_endpoint_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t endpoint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      endpoint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      endpoint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      endpoint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->endpoint_names_length);
      if(endpoint_names_lengthT > endpoint_names_length)
        this->endpoint_names = (char**)realloc(this->endpoint_names, endpoint_names_lengthT * sizeof(char*));
      endpoint_names_length = endpoint_names_lengthT;
      for( uint32_t i = 0; i < endpoint_names_length; i++){
      uint32_t length_st_endpoint_names;
      arrToVar(length_st_endpoint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_endpoint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_endpoint_names-1]=0;
      this->st_endpoint_names = (char *)(inbuffer + offset-1);
      offset += length_st_endpoint_names;
        memcpy( &(this->endpoint_names[i]), &(this->st_endpoint_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "intera_core_msgs/EndpointNamesArray"; };
    const char * getMD5(){ return "6bf0a2d04056051f84da1291f261f35a"; };

  };

}
#endif
