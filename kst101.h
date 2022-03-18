#ifndef __KST101_H__
#define __KST101_H__

#pragma once

#include "libapt.h"

// KST101 - PREFIX : ST

typedef APT_STATE ST_STATE;
#define ST_STATE_LABELS APT_STATE_LABELS

// page 122
enum class ST_ACTUATOR_TYPE : uint8_t {
  ACTUATOR_TYPE_ZST_LEGACY_6MM = 0x20,
  ACTUATOR_TYPE_ZST_LEGACY_13MM = 0x21,
  ACTUATOR_TYPE_ZST_LEGACY_25MM = 0x22,
  ACTUATOR_TYPE_ZST_NEW_6MM = 0x30,
  ACTUATOR_TYPE_ZST_NEW_13MM = 0x31,
  ACTUATOR_TYPE_ZST_NEW_25MM = 0x32,
  ACTUATOR_TYPE_ZFS_NEW_6MM = 0x40,
  ACTUATOR_TYPE_ZFS_NEW_13MM = 0x41,
  ACTUATOR_TYPE_ZFS_NEW_25MM = 0x42,
  ACTUATOR_TYPE_DRV013_25MM = 0x50,
  ACTUATOR_TYPE_DRV014_50MM = 0x51,
  ACTUATOR_TYPE_N_TYPES
};
#define ST_ACTUATOR_TYPE_LABELS {"ACTUATOR_TYPE_INVALID_00",    "ACTUATOR_TYPE_INVALID_01",     "ACTUATOR_TYPE_INVALID_02",     "ACTUATOR_TYPE_INVALID_03","ACTUATOR_TYPE_INVALID_04","ACTUATOR_TYPE_INVALID_05","ACTUATOR_TYPE_INVALID_06","ACTUATOR_TYPE_INVALID_07","ACTUATOR_TYPE_INVALID_08","ACTUATOR_TYPE_INVALID_09","ACTUATOR_TYPE_INVALID_0A","ACTUATOR_TYPE_INVALID_0B","ACTUATOR_TYPE_INVALID_0C","ACTUATOR_TYPE_INVALID_0D","ACTUATOR_TYPE_INVALID_0E","ACTUATOR_TYPE_INVALID_0F",\
                                 "ACTUATOR_TYPE_INVALID_10",    "ACTUATOR_TYPE_INVALID_11",     "ACTUATOR_TYPE_INVALID_12",     "ACTUATOR_TYPE_INVALID_13","ACTUATOR_TYPE_INVALID_14","ACTUATOR_TYPE_INVALID_15","ACTUATOR_TYPE_INVALID_16","ACTUATOR_TYPE_INVALID_17","ACTUATOR_TYPE_INVALID_18","ACTUATOR_TYPE_INVALID_19","ACTUATOR_TYPE_INVALID_1A","ACTUATOR_TYPE_INVALID_1B","ACTUATOR_TYPE_INVALID_1C","ACTUATOR_TYPE_INVALID_1D","ACTUATOR_TYPE_INVALID_1E","ACTUATOR_TYPE_INVALID_1F",\
                                 "ACTUATOR_TYPE_ZST_LEGACY_6MM","ACTUATOR_TYPE_ZST_LEGACY_13MM","ACTUATOR_TYPE_ZST_LEGACY_25MM","ACTUATOR_TYPE_INVALID_23","ACTUATOR_TYPE_INVALID_24","ACTUATOR_TYPE_INVALID_25","ACTUATOR_TYPE_INVALID_26","ACTUATOR_TYPE_INVALID_27","ACTUATOR_TYPE_INVALID_28","ACTUATOR_TYPE_INVALID_29","ACTUATOR_TYPE_INVALID_2A","ACTUATOR_TYPE_INVALID_2B","ACTUATOR_TYPE_INVALID_2C","ACTUATOR_TYPE_INVALID_2D","ACTUATOR_TYPE_INVALID_2E","ACTUATOR_TYPE_INVALID_2F",\
                                 "ACTUATOR_TYPE_ZST_NEW_6MM",   "ACTUATOR_TYPE_ZST_NEW_13MM",   "ACTUATOR_TYPE_ZST_NEW_25MM",   "ACTUATOR_TYPE_INVALID_33","ACTUATOR_TYPE_INVALID_34","ACTUATOR_TYPE_INVALID_35","ACTUATOR_TYPE_INVALID_36","ACTUATOR_TYPE_INVALID_37","ACTUATOR_TYPE_INVALID_38","ACTUATOR_TYPE_INVALID_39","ACTUATOR_TYPE_INVALID_3A","ACTUATOR_TYPE_INVALID_3B","ACTUATOR_TYPE_INVALID_3C","ACTUATOR_TYPE_INVALID_3D","ACTUATOR_TYPE_INVALID_3E","ACTUATOR_TYPE_INVALID_3F",\
                                 "ACTUATOR_TYPE_ZFS_NEW_6MM",   "ACTUATOR_TYPE_ZFS_NEW_13MM",   "ACTUATOR_TYPE_ZFS_NEW_25MM",   "ACTUATOR_TYPE_INVALID_43","ACTUATOR_TYPE_INVALID_44","ACTUATOR_TYPE_INVALID_45","ACTUATOR_TYPE_INVALID_46","ACTUATOR_TYPE_INVALID_47","ACTUATOR_TYPE_INVALID_48","ACTUATOR_TYPE_INVALID_49","ACTUATOR_TYPE_INVALID_4A","ACTUATOR_TYPE_INVALID_4B","ACTUATOR_TYPE_INVALID_4C","ACTUATOR_TYPE_INVALID_4D","ACTUATOR_TYPE_INVALID_4E","ACTUATOR_TYPE_INVALID_4F",\
                                 "ACTUATOR_DRV013_25MM",        "ACTUATOR_DRV014_50MM"}

#define ST_TICKS_PER_MM 2184528.0

namespace aptserial {
  // ==================================================================================================================
  const std::string actuatorTypeToString(const ST_ACTUATOR_TYPE _actType);
  const bool stringToActuatorType(const std::string _actTypeString, ST_ACTUATOR_TYPE& _actType);
  // ==================================================================================================================


  class KST101 : public APTDevice { // Piezo Controller
    struct stChannelValue { // page 81
      uint16_t channel = 0x0000;
      uint32_t value = 0x00000000;
    } __attribute__((packed));
    struct stVelocityParameters { // page 66
      uint16_t channel = 0x0000;
      uint32_t minVelocity = 0x0000;
      uint32_t acceleration = 0x0000;
      uint32_t maxVelocity = 0x0000;
    } __attribute__((packed));
    struct stMotorStatus { // page 123
      uint16_t channel = 0x0000;
      uint32_t position = 0x0000;
      uint16_t Velocity = 0x0000;
      uint16_t motorCurrent = 0x0000;
      uint32_t statusFlags = 0x0000;
    } __attribute__((packed));
  private:
    uint32_t m_minVelocity, m_acceleration, m_maxVelocity;
    int32_t m_position_tick;
    uint32_t m_backlash;
    ST_STATE m_state;
  public:
    KST101(const std::string _deviceFileName);
    ~KST101() = default;
    KST101( const KST101& ) = delete; // disallows copy construction: KST101 a = b
    KST101& operator= ( const KST101& ) = delete; // disallows copy assignment: a = b;
    void identifyDevice() {
      return APTDevice::identifyDevice(APT_CHANNEL::CHANNEL_1);
    }


    void setActuatorType(const ST_ACTUATOR_TYPE _actType);


    void setVelocityParameters(const uint32_t _minVelocity, const uint32_t _acceleration, const uint32_t _maxVelocity);
    void getVelocityParameters(uint32_t& _minVelocity, uint32_t& _acceleration, uint32_t& _maxVelocity);
    void updateVelocityParameters();
    uint32_t getMinVelocity() {
      return m_minVelocity;
    }
    uint32_t getAcceleration() {
      return m_acceleration;
    }
    uint32_t getMaxVelocity() {
      return m_maxVelocity;
    }


    void setMovementParameters(const uint32_t _backlash);
    void getMovementParameters(uint32_t& _backlash);
    void updateMovementParameters();
    uint32_t getBacklash() {
      return m_backlash;
    }


    void setPositionCounter(const int32_t _position_tick);
    void resetPositionCounter() {
      setPositionCounter(0);
    }
    void getPositionCounter(int32_t& _position_tick);
    void updatePositionCounter();


    int32_t getPosition() {
      return m_position_tick;
    }
    float getPosition_mm() {
      return (float)m_position_tick/ST_TICKS_PER_MM;
    }


    void setPositionRelative();
    void setPositionRelative_mm() {
      setPositionRelative();
    }
    void setPositionRelative(const int32_t _position_tick);
    void setPositionRelative_mm(const float _position_mm) {
      setPositionRelative((int32_t)(_position_mm*ST_TICKS_PER_MM));
    }


    void setPositionAbsolute(const int32_t _position_tick);
    void setPositionAbsolute_mm(const float _position_mm) {
      setPositionAbsolute((int32_t)(_position_mm*ST_TICKS_PER_MM));
    }


    void moveHome();


    void stopMoving();

    
    void setChannelEnableState(const ST_STATE _state);
    void updateChannelEnableState();
    ST_STATE getChannelEnableState() {
      return m_state;
    }
  };
  // ==================================================================================================================
}

#endif /* __KST101_H__ */