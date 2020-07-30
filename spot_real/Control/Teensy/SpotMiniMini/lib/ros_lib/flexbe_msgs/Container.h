#ifndef _ROS_flexbe_msgs_Container_h
#define _ROS_flexbe_msgs_Container_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace flexbe_msgs
{

  class Container : public ros::Msg
  {
    public:
      typedef const char* _path_type;
      _path_type path;
      uint32_t children_length;
      typedef char* _children_type;
      _children_type st_children;
      _children_type * children;
      uint32_t outcomes_length;
      typedef char* _outcomes_type;
      _outcomes_type st_outcomes;
      _outcomes_type * outcomes;
      uint32_t transitions_length;
      typedef char* _transitions_type;
      _transitions_type st_transitions;
      _transitions_type * transitions;
      uint32_t autonomy_length;
      typedef int8_t _autonomy_type;
      _autonomy_type st_autonomy;
      _autonomy_type * autonomy;

    Container():
      path(""),
      children_length(0), children(NULL),
      outcomes_length(0), outcomes(NULL),
      transitions_length(0), transitions(NULL),
      autonomy_length(0), autonomy(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_path = strlen(this->path);
      varToArr(outbuffer + offset, length_path);
      offset += 4;
      memcpy(outbuffer + offset, this->path, length_path);
      offset += length_path;
      *(outbuffer + offset + 0) = (this->children_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->children_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->children_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->children_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->children_length);
      for( uint32_t i = 0; i < children_length; i++){
      uint32_t length_childreni = strlen(this->children[i]);
      varToArr(outbuffer + offset, length_childreni);
      offset += 4;
      memcpy(outbuffer + offset, this->children[i], length_childreni);
      offset += length_childreni;
      }
      *(outbuffer + offset + 0) = (this->outcomes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->outcomes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->outcomes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->outcomes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->outcomes_length);
      for( uint32_t i = 0; i < outcomes_length; i++){
      uint32_t length_outcomesi = strlen(this->outcomes[i]);
      varToArr(outbuffer + offset, length_outcomesi);
      offset += 4;
      memcpy(outbuffer + offset, this->outcomes[i], length_outcomesi);
      offset += length_outcomesi;
      }
      *(outbuffer + offset + 0) = (this->transitions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->transitions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->transitions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->transitions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transitions_length);
      for( uint32_t i = 0; i < transitions_length; i++){
      uint32_t length_transitionsi = strlen(this->transitions[i]);
      varToArr(outbuffer + offset, length_transitionsi);
      offset += 4;
      memcpy(outbuffer + offset, this->transitions[i], length_transitionsi);
      offset += length_transitionsi;
      }
      *(outbuffer + offset + 0) = (this->autonomy_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->autonomy_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->autonomy_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->autonomy_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->autonomy_length);
      for( uint32_t i = 0; i < autonomy_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_autonomyi;
      u_autonomyi.real = this->autonomy[i];
      *(outbuffer + offset + 0) = (u_autonomyi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->autonomy[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_path;
      arrToVar(length_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_path-1]=0;
      this->path = (char *)(inbuffer + offset-1);
      offset += length_path;
      uint32_t children_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      children_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->children_length);
      if(children_lengthT > children_length)
        this->children = (char**)realloc(this->children, children_lengthT * sizeof(char*));
      children_length = children_lengthT;
      for( uint32_t i = 0; i < children_length; i++){
      uint32_t length_st_children;
      arrToVar(length_st_children, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_children; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_children-1]=0;
      this->st_children = (char *)(inbuffer + offset-1);
      offset += length_st_children;
        memcpy( &(this->children[i]), &(this->st_children), sizeof(char*));
      }
      uint32_t outcomes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      outcomes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->outcomes_length);
      if(outcomes_lengthT > outcomes_length)
        this->outcomes = (char**)realloc(this->outcomes, outcomes_lengthT * sizeof(char*));
      outcomes_length = outcomes_lengthT;
      for( uint32_t i = 0; i < outcomes_length; i++){
      uint32_t length_st_outcomes;
      arrToVar(length_st_outcomes, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_outcomes; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_outcomes-1]=0;
      this->st_outcomes = (char *)(inbuffer + offset-1);
      offset += length_st_outcomes;
        memcpy( &(this->outcomes[i]), &(this->st_outcomes), sizeof(char*));
      }
      uint32_t transitions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      transitions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->transitions_length);
      if(transitions_lengthT > transitions_length)
        this->transitions = (char**)realloc(this->transitions, transitions_lengthT * sizeof(char*));
      transitions_length = transitions_lengthT;
      for( uint32_t i = 0; i < transitions_length; i++){
      uint32_t length_st_transitions;
      arrToVar(length_st_transitions, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_transitions; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_transitions-1]=0;
      this->st_transitions = (char *)(inbuffer + offset-1);
      offset += length_st_transitions;
        memcpy( &(this->transitions[i]), &(this->st_transitions), sizeof(char*));
      }
      uint32_t autonomy_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      autonomy_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      autonomy_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      autonomy_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->autonomy_length);
      if(autonomy_lengthT > autonomy_length)
        this->autonomy = (int8_t*)realloc(this->autonomy, autonomy_lengthT * sizeof(int8_t));
      autonomy_length = autonomy_lengthT;
      for( uint32_t i = 0; i < autonomy_length; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_st_autonomy;
      u_st_autonomy.base = 0;
      u_st_autonomy.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_autonomy = u_st_autonomy.real;
      offset += sizeof(this->st_autonomy);
        memcpy( &(this->autonomy[i]), &(this->st_autonomy), sizeof(int8_t));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/Container"; };
    const char * getMD5(){ return "627eacc0f462c8ee83d7105e17cf4119"; };

  };

}
#endif
