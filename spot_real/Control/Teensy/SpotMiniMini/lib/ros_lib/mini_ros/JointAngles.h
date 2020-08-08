#ifndef _ROS_mini_ros_JointAngles_h
#define _ROS_mini_ros_JointAngles_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mini_ros
{

  class JointAngles : public ros::Msg
  {
    public:
      typedef float _fls_type;
      _fls_type fls;
      typedef float _fle_type;
      _fle_type fle;
      typedef float _flw_type;
      _flw_type flw;
      typedef float _frs_type;
      _frs_type frs;
      typedef float _fre_type;
      _fre_type fre;
      typedef float _frw_type;
      _frw_type frw;
      typedef float _bls_type;
      _bls_type bls;
      typedef float _ble_type;
      _ble_type ble;
      typedef float _blw_type;
      _blw_type blw;
      typedef float _brs_type;
      _brs_type brs;
      typedef float _bre_type;
      _bre_type bre;
      typedef float _brw_type;
      _brw_type brw;
      typedef bool _step_or_view_type;
      _step_or_view_type step_or_view;

    JointAngles():
      fls(0),
      fle(0),
      flw(0),
      frs(0),
      fre(0),
      frw(0),
      bls(0),
      ble(0),
      blw(0),
      brs(0),
      bre(0),
      brw(0),
      step_or_view(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_fls;
      u_fls.real = this->fls;
      *(outbuffer + offset + 0) = (u_fls.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fls.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fls.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fls.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fls);
      union {
        float real;
        uint32_t base;
      } u_fle;
      u_fle.real = this->fle;
      *(outbuffer + offset + 0) = (u_fle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fle);
      union {
        float real;
        uint32_t base;
      } u_flw;
      u_flw.real = this->flw;
      *(outbuffer + offset + 0) = (u_flw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_flw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_flw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_flw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->flw);
      union {
        float real;
        uint32_t base;
      } u_frs;
      u_frs.real = this->frs;
      *(outbuffer + offset + 0) = (u_frs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frs);
      union {
        float real;
        uint32_t base;
      } u_fre;
      u_fre.real = this->fre;
      *(outbuffer + offset + 0) = (u_fre.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fre.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fre.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fre.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fre);
      union {
        float real;
        uint32_t base;
      } u_frw;
      u_frw.real = this->frw;
      *(outbuffer + offset + 0) = (u_frw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_frw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_frw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_frw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->frw);
      union {
        float real;
        uint32_t base;
      } u_bls;
      u_bls.real = this->bls;
      *(outbuffer + offset + 0) = (u_bls.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bls.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bls.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bls.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bls);
      union {
        float real;
        uint32_t base;
      } u_ble;
      u_ble.real = this->ble;
      *(outbuffer + offset + 0) = (u_ble.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ble.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ble.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ble.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ble);
      union {
        float real;
        uint32_t base;
      } u_blw;
      u_blw.real = this->blw;
      *(outbuffer + offset + 0) = (u_blw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_blw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_blw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_blw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->blw);
      union {
        float real;
        uint32_t base;
      } u_brs;
      u_brs.real = this->brs;
      *(outbuffer + offset + 0) = (u_brs.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brs.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_brs.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_brs.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->brs);
      union {
        float real;
        uint32_t base;
      } u_bre;
      u_bre.real = this->bre;
      *(outbuffer + offset + 0) = (u_bre.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bre.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bre.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bre.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bre);
      union {
        float real;
        uint32_t base;
      } u_brw;
      u_brw.real = this->brw;
      *(outbuffer + offset + 0) = (u_brw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_brw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_brw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->brw);
      union {
        bool real;
        uint8_t base;
      } u_step_or_view;
      u_step_or_view.real = this->step_or_view;
      *(outbuffer + offset + 0) = (u_step_or_view.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->step_or_view);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_fls;
      u_fls.base = 0;
      u_fls.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fls.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fls.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fls.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fls = u_fls.real;
      offset += sizeof(this->fls);
      union {
        float real;
        uint32_t base;
      } u_fle;
      u_fle.base = 0;
      u_fle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fle = u_fle.real;
      offset += sizeof(this->fle);
      union {
        float real;
        uint32_t base;
      } u_flw;
      u_flw.base = 0;
      u_flw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_flw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_flw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_flw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->flw = u_flw.real;
      offset += sizeof(this->flw);
      union {
        float real;
        uint32_t base;
      } u_frs;
      u_frs.base = 0;
      u_frs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frs = u_frs.real;
      offset += sizeof(this->frs);
      union {
        float real;
        uint32_t base;
      } u_fre;
      u_fre.base = 0;
      u_fre.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fre.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fre.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fre.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fre = u_fre.real;
      offset += sizeof(this->fre);
      union {
        float real;
        uint32_t base;
      } u_frw;
      u_frw.base = 0;
      u_frw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_frw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_frw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_frw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->frw = u_frw.real;
      offset += sizeof(this->frw);
      union {
        float real;
        uint32_t base;
      } u_bls;
      u_bls.base = 0;
      u_bls.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bls.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bls.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bls.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bls = u_bls.real;
      offset += sizeof(this->bls);
      union {
        float real;
        uint32_t base;
      } u_ble;
      u_ble.base = 0;
      u_ble.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ble.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ble.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ble.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ble = u_ble.real;
      offset += sizeof(this->ble);
      union {
        float real;
        uint32_t base;
      } u_blw;
      u_blw.base = 0;
      u_blw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_blw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_blw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_blw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->blw = u_blw.real;
      offset += sizeof(this->blw);
      union {
        float real;
        uint32_t base;
      } u_brs;
      u_brs.base = 0;
      u_brs.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brs.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_brs.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_brs.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->brs = u_brs.real;
      offset += sizeof(this->brs);
      union {
        float real;
        uint32_t base;
      } u_bre;
      u_bre.base = 0;
      u_bre.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bre.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bre.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bre.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bre = u_bre.real;
      offset += sizeof(this->bre);
      union {
        float real;
        uint32_t base;
      } u_brw;
      u_brw.base = 0;
      u_brw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_brw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_brw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->brw = u_brw.real;
      offset += sizeof(this->brw);
      union {
        bool real;
        uint8_t base;
      } u_step_or_view;
      u_step_or_view.base = 0;
      u_step_or_view.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->step_or_view = u_step_or_view.real;
      offset += sizeof(this->step_or_view);
     return offset;
    }

    const char * getType(){ return "mini_ros/JointAngles"; };
    const char * getMD5(){ return "54bfcd55046a61d9df3827d322506389"; };

  };

}
#endif
