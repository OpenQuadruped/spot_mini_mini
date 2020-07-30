#ifndef _ROS_baxter_maintenance_msgs_UpdateSources_h
#define _ROS_baxter_maintenance_msgs_UpdateSources_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "baxter_maintenance_msgs/UpdateSource.h"

namespace baxter_maintenance_msgs
{

  class UpdateSources : public ros::Msg
  {
    public:
      typedef const char* _uuid_type;
      _uuid_type uuid;
      uint32_t sources_length;
      typedef baxter_maintenance_msgs::UpdateSource _sources_type;
      _sources_type st_sources;
      _sources_type * sources;

    UpdateSources():
      uuid(""),
      sources_length(0), sources(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_uuid = strlen(this->uuid);
      varToArr(outbuffer + offset, length_uuid);
      offset += 4;
      memcpy(outbuffer + offset, this->uuid, length_uuid);
      offset += length_uuid;
      *(outbuffer + offset + 0) = (this->sources_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sources_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sources_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sources_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sources_length);
      for( uint32_t i = 0; i < sources_length; i++){
      offset += this->sources[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_uuid;
      arrToVar(length_uuid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_uuid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_uuid-1]=0;
      this->uuid = (char *)(inbuffer + offset-1);
      offset += length_uuid;
      uint32_t sources_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      sources_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      sources_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      sources_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->sources_length);
      if(sources_lengthT > sources_length)
        this->sources = (baxter_maintenance_msgs::UpdateSource*)realloc(this->sources, sources_lengthT * sizeof(baxter_maintenance_msgs::UpdateSource));
      sources_length = sources_lengthT;
      for( uint32_t i = 0; i < sources_length; i++){
      offset += this->st_sources.deserialize(inbuffer + offset);
        memcpy( &(this->sources[i]), &(this->st_sources), sizeof(baxter_maintenance_msgs::UpdateSource));
      }
     return offset;
    }

    const char * getType(){ return "baxter_maintenance_msgs/UpdateSources"; };
    const char * getMD5(){ return "b3b428bf55e80e83d378830c33b3405b"; };

  };

}
#endif
