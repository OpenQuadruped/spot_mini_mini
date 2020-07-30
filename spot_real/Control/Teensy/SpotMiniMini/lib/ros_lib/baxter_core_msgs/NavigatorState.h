#ifndef _ROS_baxter_core_msgs_NavigatorState_h
#define _ROS_baxter_core_msgs_NavigatorState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baxter_core_msgs
{

  class NavigatorState : public ros::Msg
  {
    public:
      uint32_t button_names_length;
      typedef char* _button_names_type;
      _button_names_type st_button_names;
      _button_names_type * button_names;
      uint32_t buttons_length;
      typedef bool _buttons_type;
      _buttons_type st_buttons;
      _buttons_type * buttons;
      typedef uint8_t _wheel_type;
      _wheel_type wheel;
      uint32_t light_names_length;
      typedef char* _light_names_type;
      _light_names_type st_light_names;
      _light_names_type * light_names;
      uint32_t lights_length;
      typedef bool _lights_type;
      _lights_type st_lights;
      _lights_type * lights;

    NavigatorState():
      button_names_length(0), button_names(NULL),
      buttons_length(0), buttons(NULL),
      wheel(0),
      light_names_length(0), light_names(NULL),
      lights_length(0), lights(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->button_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->button_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->button_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->button_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->button_names_length);
      for( uint32_t i = 0; i < button_names_length; i++){
      uint32_t length_button_namesi = strlen(this->button_names[i]);
      varToArr(outbuffer + offset, length_button_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->button_names[i], length_button_namesi);
      offset += length_button_namesi;
      }
      *(outbuffer + offset + 0) = (this->buttons_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->buttons_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->buttons_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->buttons_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buttons_length);
      for( uint32_t i = 0; i < buttons_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_buttonsi;
      u_buttonsi.real = this->buttons[i];
      *(outbuffer + offset + 0) = (u_buttonsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buttons[i]);
      }
      *(outbuffer + offset + 0) = (this->wheel >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wheel);
      *(outbuffer + offset + 0) = (this->light_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->light_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->light_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->light_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->light_names_length);
      for( uint32_t i = 0; i < light_names_length; i++){
      uint32_t length_light_namesi = strlen(this->light_names[i]);
      varToArr(outbuffer + offset, length_light_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->light_names[i], length_light_namesi);
      offset += length_light_namesi;
      }
      *(outbuffer + offset + 0) = (this->lights_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->lights_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->lights_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->lights_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lights_length);
      for( uint32_t i = 0; i < lights_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_lightsi;
      u_lightsi.real = this->lights[i];
      *(outbuffer + offset + 0) = (u_lightsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lights[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t button_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      button_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      button_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      button_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->button_names_length);
      if(button_names_lengthT > button_names_length)
        this->button_names = (char**)realloc(this->button_names, button_names_lengthT * sizeof(char*));
      button_names_length = button_names_lengthT;
      for( uint32_t i = 0; i < button_names_length; i++){
      uint32_t length_st_button_names;
      arrToVar(length_st_button_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_button_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_button_names-1]=0;
      this->st_button_names = (char *)(inbuffer + offset-1);
      offset += length_st_button_names;
        memcpy( &(this->button_names[i]), &(this->st_button_names), sizeof(char*));
      }
      uint32_t buttons_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      buttons_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->buttons_length);
      if(buttons_lengthT > buttons_length)
        this->buttons = (bool*)realloc(this->buttons, buttons_lengthT * sizeof(bool));
      buttons_length = buttons_lengthT;
      for( uint32_t i = 0; i < buttons_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_buttons;
      u_st_buttons.base = 0;
      u_st_buttons.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_buttons = u_st_buttons.real;
      offset += sizeof(this->st_buttons);
        memcpy( &(this->buttons[i]), &(this->st_buttons), sizeof(bool));
      }
      this->wheel =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->wheel);
      uint32_t light_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      light_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      light_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      light_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->light_names_length);
      if(light_names_lengthT > light_names_length)
        this->light_names = (char**)realloc(this->light_names, light_names_lengthT * sizeof(char*));
      light_names_length = light_names_lengthT;
      for( uint32_t i = 0; i < light_names_length; i++){
      uint32_t length_st_light_names;
      arrToVar(length_st_light_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_light_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_light_names-1]=0;
      this->st_light_names = (char *)(inbuffer + offset-1);
      offset += length_st_light_names;
        memcpy( &(this->light_names[i]), &(this->st_light_names), sizeof(char*));
      }
      uint32_t lights_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      lights_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      lights_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      lights_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->lights_length);
      if(lights_lengthT > lights_length)
        this->lights = (bool*)realloc(this->lights, lights_lengthT * sizeof(bool));
      lights_length = lights_lengthT;
      for( uint32_t i = 0; i < lights_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_lights;
      u_st_lights.base = 0;
      u_st_lights.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_lights = u_st_lights.real;
      offset += sizeof(this->st_lights);
        memcpy( &(this->lights[i]), &(this->st_lights), sizeof(bool));
      }
     return offset;
    }

    const char * getType(){ return "baxter_core_msgs/NavigatorState"; };
    const char * getMD5(){ return "680d121a1f16a32647298b292492fffd"; };

  };

}
#endif
