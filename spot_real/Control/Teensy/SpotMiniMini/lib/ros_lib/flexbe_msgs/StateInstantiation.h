#ifndef _ROS_flexbe_msgs_StateInstantiation_h
#define _ROS_flexbe_msgs_StateInstantiation_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "flexbe_msgs/OutcomeCondition.h"

namespace flexbe_msgs
{

  class StateInstantiation : public ros::Msg
  {
    public:
      typedef const char* _state_path_type;
      _state_path_type state_path;
      typedef const char* _state_class_type;
      _state_class_type state_class;
      typedef const char* _initial_state_name_type;
      _initial_state_name_type initial_state_name;
      uint32_t input_keys_length;
      typedef char* _input_keys_type;
      _input_keys_type st_input_keys;
      _input_keys_type * input_keys;
      uint32_t output_keys_length;
      typedef char* _output_keys_type;
      _output_keys_type st_output_keys;
      _output_keys_type * output_keys;
      uint32_t cond_outcome_length;
      typedef char* _cond_outcome_type;
      _cond_outcome_type st_cond_outcome;
      _cond_outcome_type * cond_outcome;
      uint32_t cond_transition_length;
      typedef flexbe_msgs::OutcomeCondition _cond_transition_type;
      _cond_transition_type st_cond_transition;
      _cond_transition_type * cond_transition;
      typedef const char* _behavior_class_type;
      _behavior_class_type behavior_class;
      uint32_t parameter_names_length;
      typedef char* _parameter_names_type;
      _parameter_names_type st_parameter_names;
      _parameter_names_type * parameter_names;
      uint32_t parameter_values_length;
      typedef char* _parameter_values_type;
      _parameter_values_type st_parameter_values;
      _parameter_values_type * parameter_values;
      float position[2];
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
      uint32_t userdata_keys_length;
      typedef char* _userdata_keys_type;
      _userdata_keys_type st_userdata_keys;
      _userdata_keys_type * userdata_keys;
      uint32_t userdata_remapping_length;
      typedef char* _userdata_remapping_type;
      _userdata_remapping_type st_userdata_remapping;
      _userdata_remapping_type * userdata_remapping;
      enum { CLASS_STATEMACHINE =  :STATEMACHINE };
      enum { CLASS_CONCURRENCY =  :CONCURRENCY };
      enum { CLASS_PRIORITY =  :PRIORITY };
      enum { CLASS_BEHAVIOR =  :BEHAVIOR };

    StateInstantiation():
      state_path(""),
      state_class(""),
      initial_state_name(""),
      input_keys_length(0), input_keys(NULL),
      output_keys_length(0), output_keys(NULL),
      cond_outcome_length(0), cond_outcome(NULL),
      cond_transition_length(0), cond_transition(NULL),
      behavior_class(""),
      parameter_names_length(0), parameter_names(NULL),
      parameter_values_length(0), parameter_values(NULL),
      position(),
      outcomes_length(0), outcomes(NULL),
      transitions_length(0), transitions(NULL),
      autonomy_length(0), autonomy(NULL),
      userdata_keys_length(0), userdata_keys(NULL),
      userdata_remapping_length(0), userdata_remapping(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_state_path = strlen(this->state_path);
      varToArr(outbuffer + offset, length_state_path);
      offset += 4;
      memcpy(outbuffer + offset, this->state_path, length_state_path);
      offset += length_state_path;
      uint32_t length_state_class = strlen(this->state_class);
      varToArr(outbuffer + offset, length_state_class);
      offset += 4;
      memcpy(outbuffer + offset, this->state_class, length_state_class);
      offset += length_state_class;
      uint32_t length_initial_state_name = strlen(this->initial_state_name);
      varToArr(outbuffer + offset, length_initial_state_name);
      offset += 4;
      memcpy(outbuffer + offset, this->initial_state_name, length_initial_state_name);
      offset += length_initial_state_name;
      *(outbuffer + offset + 0) = (this->input_keys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->input_keys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->input_keys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->input_keys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->input_keys_length);
      for( uint32_t i = 0; i < input_keys_length; i++){
      uint32_t length_input_keysi = strlen(this->input_keys[i]);
      varToArr(outbuffer + offset, length_input_keysi);
      offset += 4;
      memcpy(outbuffer + offset, this->input_keys[i], length_input_keysi);
      offset += length_input_keysi;
      }
      *(outbuffer + offset + 0) = (this->output_keys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->output_keys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->output_keys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->output_keys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->output_keys_length);
      for( uint32_t i = 0; i < output_keys_length; i++){
      uint32_t length_output_keysi = strlen(this->output_keys[i]);
      varToArr(outbuffer + offset, length_output_keysi);
      offset += 4;
      memcpy(outbuffer + offset, this->output_keys[i], length_output_keysi);
      offset += length_output_keysi;
      }
      *(outbuffer + offset + 0) = (this->cond_outcome_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cond_outcome_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cond_outcome_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cond_outcome_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cond_outcome_length);
      for( uint32_t i = 0; i < cond_outcome_length; i++){
      uint32_t length_cond_outcomei = strlen(this->cond_outcome[i]);
      varToArr(outbuffer + offset, length_cond_outcomei);
      offset += 4;
      memcpy(outbuffer + offset, this->cond_outcome[i], length_cond_outcomei);
      offset += length_cond_outcomei;
      }
      *(outbuffer + offset + 0) = (this->cond_transition_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->cond_transition_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->cond_transition_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->cond_transition_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cond_transition_length);
      for( uint32_t i = 0; i < cond_transition_length; i++){
      offset += this->cond_transition[i].serialize(outbuffer + offset);
      }
      uint32_t length_behavior_class = strlen(this->behavior_class);
      varToArr(outbuffer + offset, length_behavior_class);
      offset += 4;
      memcpy(outbuffer + offset, this->behavior_class, length_behavior_class);
      offset += length_behavior_class;
      *(outbuffer + offset + 0) = (this->parameter_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameter_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parameter_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parameter_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parameter_names_length);
      for( uint32_t i = 0; i < parameter_names_length; i++){
      uint32_t length_parameter_namesi = strlen(this->parameter_names[i]);
      varToArr(outbuffer + offset, length_parameter_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->parameter_names[i], length_parameter_namesi);
      offset += length_parameter_namesi;
      }
      *(outbuffer + offset + 0) = (this->parameter_values_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->parameter_values_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->parameter_values_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->parameter_values_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->parameter_values_length);
      for( uint32_t i = 0; i < parameter_values_length; i++){
      uint32_t length_parameter_valuesi = strlen(this->parameter_values[i]);
      varToArr(outbuffer + offset, length_parameter_valuesi);
      offset += 4;
      memcpy(outbuffer + offset, this->parameter_values[i], length_parameter_valuesi);
      offset += length_parameter_valuesi;
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
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
      *(outbuffer + offset + 0) = (this->userdata_keys_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->userdata_keys_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->userdata_keys_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->userdata_keys_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->userdata_keys_length);
      for( uint32_t i = 0; i < userdata_keys_length; i++){
      uint32_t length_userdata_keysi = strlen(this->userdata_keys[i]);
      varToArr(outbuffer + offset, length_userdata_keysi);
      offset += 4;
      memcpy(outbuffer + offset, this->userdata_keys[i], length_userdata_keysi);
      offset += length_userdata_keysi;
      }
      *(outbuffer + offset + 0) = (this->userdata_remapping_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->userdata_remapping_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->userdata_remapping_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->userdata_remapping_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->userdata_remapping_length);
      for( uint32_t i = 0; i < userdata_remapping_length; i++){
      uint32_t length_userdata_remappingi = strlen(this->userdata_remapping[i]);
      varToArr(outbuffer + offset, length_userdata_remappingi);
      offset += 4;
      memcpy(outbuffer + offset, this->userdata_remapping[i], length_userdata_remappingi);
      offset += length_userdata_remappingi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_state_path;
      arrToVar(length_state_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_path-1]=0;
      this->state_path = (char *)(inbuffer + offset-1);
      offset += length_state_path;
      uint32_t length_state_class;
      arrToVar(length_state_class, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state_class; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state_class-1]=0;
      this->state_class = (char *)(inbuffer + offset-1);
      offset += length_state_class;
      uint32_t length_initial_state_name;
      arrToVar(length_initial_state_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_initial_state_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_initial_state_name-1]=0;
      this->initial_state_name = (char *)(inbuffer + offset-1);
      offset += length_initial_state_name;
      uint32_t input_keys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      input_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->input_keys_length);
      if(input_keys_lengthT > input_keys_length)
        this->input_keys = (char**)realloc(this->input_keys, input_keys_lengthT * sizeof(char*));
      input_keys_length = input_keys_lengthT;
      for( uint32_t i = 0; i < input_keys_length; i++){
      uint32_t length_st_input_keys;
      arrToVar(length_st_input_keys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_input_keys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_input_keys-1]=0;
      this->st_input_keys = (char *)(inbuffer + offset-1);
      offset += length_st_input_keys;
        memcpy( &(this->input_keys[i]), &(this->st_input_keys), sizeof(char*));
      }
      uint32_t output_keys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      output_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      output_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      output_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->output_keys_length);
      if(output_keys_lengthT > output_keys_length)
        this->output_keys = (char**)realloc(this->output_keys, output_keys_lengthT * sizeof(char*));
      output_keys_length = output_keys_lengthT;
      for( uint32_t i = 0; i < output_keys_length; i++){
      uint32_t length_st_output_keys;
      arrToVar(length_st_output_keys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_output_keys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_output_keys-1]=0;
      this->st_output_keys = (char *)(inbuffer + offset-1);
      offset += length_st_output_keys;
        memcpy( &(this->output_keys[i]), &(this->st_output_keys), sizeof(char*));
      }
      uint32_t cond_outcome_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cond_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cond_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cond_outcome_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cond_outcome_length);
      if(cond_outcome_lengthT > cond_outcome_length)
        this->cond_outcome = (char**)realloc(this->cond_outcome, cond_outcome_lengthT * sizeof(char*));
      cond_outcome_length = cond_outcome_lengthT;
      for( uint32_t i = 0; i < cond_outcome_length; i++){
      uint32_t length_st_cond_outcome;
      arrToVar(length_st_cond_outcome, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_cond_outcome; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_cond_outcome-1]=0;
      this->st_cond_outcome = (char *)(inbuffer + offset-1);
      offset += length_st_cond_outcome;
        memcpy( &(this->cond_outcome[i]), &(this->st_cond_outcome), sizeof(char*));
      }
      uint32_t cond_transition_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      cond_transition_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      cond_transition_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      cond_transition_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->cond_transition_length);
      if(cond_transition_lengthT > cond_transition_length)
        this->cond_transition = (flexbe_msgs::OutcomeCondition*)realloc(this->cond_transition, cond_transition_lengthT * sizeof(flexbe_msgs::OutcomeCondition));
      cond_transition_length = cond_transition_lengthT;
      for( uint32_t i = 0; i < cond_transition_length; i++){
      offset += this->st_cond_transition.deserialize(inbuffer + offset);
        memcpy( &(this->cond_transition[i]), &(this->st_cond_transition), sizeof(flexbe_msgs::OutcomeCondition));
      }
      uint32_t length_behavior_class;
      arrToVar(length_behavior_class, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_behavior_class; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_behavior_class-1]=0;
      this->behavior_class = (char *)(inbuffer + offset-1);
      offset += length_behavior_class;
      uint32_t parameter_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameter_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameter_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameter_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameter_names_length);
      if(parameter_names_lengthT > parameter_names_length)
        this->parameter_names = (char**)realloc(this->parameter_names, parameter_names_lengthT * sizeof(char*));
      parameter_names_length = parameter_names_lengthT;
      for( uint32_t i = 0; i < parameter_names_length; i++){
      uint32_t length_st_parameter_names;
      arrToVar(length_st_parameter_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_parameter_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_parameter_names-1]=0;
      this->st_parameter_names = (char *)(inbuffer + offset-1);
      offset += length_st_parameter_names;
        memcpy( &(this->parameter_names[i]), &(this->st_parameter_names), sizeof(char*));
      }
      uint32_t parameter_values_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      parameter_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      parameter_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      parameter_values_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->parameter_values_length);
      if(parameter_values_lengthT > parameter_values_length)
        this->parameter_values = (char**)realloc(this->parameter_values, parameter_values_lengthT * sizeof(char*));
      parameter_values_length = parameter_values_lengthT;
      for( uint32_t i = 0; i < parameter_values_length; i++){
      uint32_t length_st_parameter_values;
      arrToVar(length_st_parameter_values, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_parameter_values; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_parameter_values-1]=0;
      this->st_parameter_values = (char *)(inbuffer + offset-1);
      offset += length_st_parameter_values;
        memcpy( &(this->parameter_values[i]), &(this->st_parameter_values), sizeof(char*));
      }
      for( uint32_t i = 0; i < 2; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.base = 0;
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positioni.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position[i] = u_positioni.real;
      offset += sizeof(this->position[i]);
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
      uint32_t userdata_keys_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      userdata_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      userdata_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      userdata_keys_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->userdata_keys_length);
      if(userdata_keys_lengthT > userdata_keys_length)
        this->userdata_keys = (char**)realloc(this->userdata_keys, userdata_keys_lengthT * sizeof(char*));
      userdata_keys_length = userdata_keys_lengthT;
      for( uint32_t i = 0; i < userdata_keys_length; i++){
      uint32_t length_st_userdata_keys;
      arrToVar(length_st_userdata_keys, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_userdata_keys; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_userdata_keys-1]=0;
      this->st_userdata_keys = (char *)(inbuffer + offset-1);
      offset += length_st_userdata_keys;
        memcpy( &(this->userdata_keys[i]), &(this->st_userdata_keys), sizeof(char*));
      }
      uint32_t userdata_remapping_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      userdata_remapping_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      userdata_remapping_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      userdata_remapping_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->userdata_remapping_length);
      if(userdata_remapping_lengthT > userdata_remapping_length)
        this->userdata_remapping = (char**)realloc(this->userdata_remapping, userdata_remapping_lengthT * sizeof(char*));
      userdata_remapping_length = userdata_remapping_lengthT;
      for( uint32_t i = 0; i < userdata_remapping_length; i++){
      uint32_t length_st_userdata_remapping;
      arrToVar(length_st_userdata_remapping, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_userdata_remapping; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_userdata_remapping-1]=0;
      this->st_userdata_remapping = (char *)(inbuffer + offset-1);
      offset += length_st_userdata_remapping;
        memcpy( &(this->userdata_remapping[i]), &(this->st_userdata_remapping), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "flexbe_msgs/StateInstantiation"; };
    const char * getMD5(){ return "1c6026e288cfff7ab6c8308ee1db66f1"; };

  };

}
#endif
