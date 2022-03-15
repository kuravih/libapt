#ifndef __KPZ101_H__
#define __KPZ101_H__

#pragma once

#include "libapt.h"

typedef APT_STATE PZ_STATE;
#define PZ_STATE_LABELS APT_STATE_LABELS

// page 184
enum class PZ_VOLTAGE_RANGE : uint8_t {
  VOLTAGE_RANGE_75V = 0x01,
  VOLTAGE_RANGE_100V,
  VOLTAGE_RANGE_150V,
  VOLTAGE_RANGE_N_RANGES
};
#define PZ_VOLTAGE_RANGE_LABELS {"VOLTAGE_RANGE_INVALID_0",\
                                 "VOLTAGE_RANGE_75V",\
                                 "VOLTAGE_RANGE_100V",\
                                 "VOLTAGE_RANGE_150V"}

// page 184
enum class PZ_ANALOG_INPUT_SOURCE : uint8_t {
  ANALOG_INPUT_SOURCE_A = 0x01,
  ANALOG_INPUT_SOURCE_B,
  ANALOG_INPUT_SOURCE_EXTSIG_SMA,
  ANALOG_INPUT_SOURCE_N_SOURCES
};
#define PZ_ANALOG_INPUT_SOURCE_LABELS {"ANALOG_INPUT_SOURCE_INVALID_0",\
                                       "ANALOG_INPUT_SOURCE_A",\
                                       "ANALOG_INPUT_SOURCE_B",\
                                       "ANALOG_INPUT_SOURCE_EXTSIG_SMA"}

// page 160
enum class PZ_INPUT_VOLTAGE_SOURCE : uint8_t {
  INPUT_VOLTAGE_SOURCE_SW_ONLY = 0x00,
  INPUT_VOLTAGE_SOURCE_EXT_SIG = 0x01,
  INPUT_VOLTAGE_SOURCE_POT = 0x02,
  INPUT_VOLTAGE_SOURCE_N_SOURCES
};
#define PZ_INPUT_VOLTAGE_SOURCE_LABELS {"INPUT_VOLTS_SRC_SW_ONLY",\
                                        "INPUT_VOLTS_SRC_EXT_SIG",\
                                        "INPUT_VOLTS_SRC_POT"}

// page 156
enum class PZ_POSITION_CONTROL_MODE : uint8_t {
  POSITION_CONTROL_MODE_OPEN_LOOP = 0x01,
  POSITION_CONTROL_MODE_CLOSED_LOOP,
  POSITION_CONTROL_MODE_OPEN_LOOP_SMOOTH,
  POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH,
  POSITION_CONTROL_MODE_N_MODES
};
#define PZ_POSITION_CONTROL_MODE_LABELS {"POSITION_CONTROL_MODE_INVALID_0",\
                                         "POSITION_CONTROL_MODE_OPEN_LOOP",\
                                         "POSITION_CONTROL_MODE_CLOSED_LOOP",\
                                         "POSITION_CONTROL_MODE_OPEN_LOOP_SMOOTH",\
                                         "POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH"}

namespace aptserial {
  // ==================================================================================================================
  const std::string voltageRangeToString(const PZ_VOLTAGE_RANGE _vRange);
  const bool stringToVoltageRange(const std::string _vRangeString, PZ_VOLTAGE_RANGE& _vRange);

  const std::string analogInputSourceToString(const PZ_ANALOG_INPUT_SOURCE _analogInputSource);
  const bool stringToAnalogInputSource(const std::string _analogInputSourceString, PZ_ANALOG_INPUT_SOURCE& _analogInputSource);

  const std::string inputVoltageSourceToString(const PZ_INPUT_VOLTAGE_SOURCE _inputVSource);
  const bool stringToInputVoltageSource(const std::string _inputVSourceString, PZ_INPUT_VOLTAGE_SOURCE& _inputVSource);

  const std::string positionControlModeToString(const PZ_POSITION_CONTROL_MODE _positionCtrlMode);
  const bool stringToPositionControlMode(const std::string _positionCtrlModeString, PZ_POSITION_CONTROL_MODE& _positionCtrlMode);

  class KPZ101 : public APTDevice { // Piezo Controller
    struct stChannelValue { // page 174
      uint16_t channel = 0x0000;
      uint16_t value = 0x0000;
    } __attribute__((packed));
    struct stChannelSource { // page 176
      uint16_t channel = 0x0000;
      uint16_t source = 0x0000;
    } __attribute__((packed));
    struct stChannelIOSettings { // page 200
      uint16_t channel = 0x0000;
      uint16_t voltageRange = 0x0000;
      uint16_t analogInput = 0x0000;
      uint16_t future1 = 0x0000;
      uint16_t future2 = 0x0000;
    } __attribute__((packed));
  private:
    PZ_ANALOG_INPUT_SOURCE m_analogInputSource;
    PZ_VOLTAGE_RANGE m_vRange;
    PZ_INPUT_VOLTAGE_SOURCE m_inputVoltageSource;
    PZ_POSITION_CONTROL_MODE m_positionControlMode;
    uint16_t m_voltage_adu;
    PZ_STATE m_state;
  public:
    KPZ101(const std::string _deviceFileName, const uint8_t _idSrcDest);
    ~KPZ101() = default;
    KPZ101( const KPZ101& ) = delete; // disallows copy construction: KPZ101 a = b
    KPZ101& operator= ( const KPZ101& ) = delete; // disallows copy assignment: a = b;
    void identifyDevice() {
      return APTDevice::identifyDevice(APT_CHANNEL::CHANNEL_1);
    }

    void setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInputSource);
    void getIOSettings(PZ_VOLTAGE_RANGE& _vRange, PZ_ANALOG_INPUT_SOURCE& _analogInputSource);
    void setIOSettings(const PZ_VOLTAGE_RANGE _vRange) {
      return setIOSettings(_vRange, m_analogInputSource);
    }
    void setIOSettings(const PZ_ANALOG_INPUT_SOURCE _analogInputSource) {
      return setIOSettings(m_vRange, _analogInputSource);
    }
    void updateIOSettings();

    PZ_VOLTAGE_RANGE getVRange() {
      return m_vRange;
    }
    std::vector<PZ_VOLTAGE_RANGE> getVRangeList() {
      return std::vector<PZ_VOLTAGE_RANGE> {PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_75V, PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_100V, PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_150V};
    }

    PZ_ANALOG_INPUT_SOURCE getAnalogInputSource() {
      return m_analogInputSource;
    }
    std::vector<PZ_ANALOG_INPUT_SOURCE> getAnalogInputSourceList() {
      return std::vector<PZ_ANALOG_INPUT_SOURCE> {PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_A, PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_B, PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_EXTSIG_SMA};
    }

    void setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource);
    void getInputVoltageSource(PZ_INPUT_VOLTAGE_SOURCE& _inputVoltageSource);
    void updateInputVoltageSource();
    PZ_INPUT_VOLTAGE_SOURCE getInputVoltageSource() {
      return m_inputVoltageSource;
    }
    std::vector<PZ_INPUT_VOLTAGE_SOURCE> getInputVoltageSourceList() {
      return std::vector<PZ_INPUT_VOLTAGE_SOURCE> {PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_SW_ONLY, PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_EXT_SIG, PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_POT};
    }

    void setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode);
    void getPositionControlMode(PZ_POSITION_CONTROL_MODE& _positionControlMode);
    void updatePositionControlMode();
    PZ_POSITION_CONTROL_MODE getPositionControlMode() {
      return m_positionControlMode;
    }
    std::vector<PZ_POSITION_CONTROL_MODE> getPositionControlModeList() {
      return std::vector<PZ_POSITION_CONTROL_MODE> {PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_OPEN_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_CLOSED_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_OPEN_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH};
    }

    void setOutputVoltage(const uint16_t _voltage_adu);
    void getOutputVoltage(uint16_t& _voltage_adu);
    void updateOutputVoltage();
    uint16_t getOutputVoltage() {
      return m_voltage_adu;
    }

    void setChannelEnableState(const PZ_STATE _state);
    void updateChannelEnableState();
    PZ_STATE getChannelEnableState() {
      return m_state;
    }
  };
  // ==================================================================================================================
}

#endif /* __KPZ101_H__ */