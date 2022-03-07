#ifndef __KPZ101_H__
#define __KPZ101_H__

#pragma once

#include "libapt.h"

namespace aptserial {
  // ==================================================================================================================
  const std::string channelToString(const PZ_CHANNEL _channel);
  const bool stringToChannel(const std::string _channelString, PZ_CHANNEL& _channel);

  const std::string stateToString(const PZ_STATE _state);
  const bool stringToState(const std::string _stateString, PZ_STATE& _state);

  const std::string voltageRangeToString(const PZ_VOLTAGE_RANGE _vRange);
  const bool stringToVoltageRange(const std::string _vRangeString, PZ_VOLTAGE_RANGE& _vRange);

  const std::string analogInputSourceToString(const PZ_ANALOG_INPUT_SOURCE _analogInputSource);
  const bool stringToAnalogInputSource(const std::string _analogInputSourceString, PZ_ANALOG_INPUT_SOURCE& _analogInputSource);

  const std::string inputVoltageSourceToString(const PZ_INPUT_VOLTAGE_SOURCE _inputVSource);
  const bool stringToInputVoltageSource(const std::string _inputVSourceString, PZ_INPUT_VOLTAGE_SOURCE& _inputVSource);

  const std::string positionControlModeToString(const PZ_POSITION_CONTROL_MODE _positionCtrlMode);
  const bool stringToPositionControlMode(const std::string _positionCtrlModeString, PZ_POSITION_CONTROL_MODE& _positionCtrlMode);

  class KPZ101 : public APTDevice { // Piezo Controller
  private:
    PZ_CHANNEL m_channelId;
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
      return APTDevice::identifyDevice(m_channelId);
    }

    void setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInputSource);
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
    void updateInputVoltageSource();
    PZ_INPUT_VOLTAGE_SOURCE getInputVoltageSource() {
      return m_inputVoltageSource;
    }
    std::vector<PZ_INPUT_VOLTAGE_SOURCE> getInputVoltageSourceList() {
      return std::vector<PZ_INPUT_VOLTAGE_SOURCE> {PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_SW_ONLY, PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_EXT_SIG, PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_POT};
    }

    void setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode);
    void updatePositionControlMode();
    PZ_POSITION_CONTROL_MODE getPositionControlMode() {
      return m_positionControlMode;
    }
    std::vector<PZ_POSITION_CONTROL_MODE> getPositionControlModeList() {
      return std::vector<PZ_POSITION_CONTROL_MODE> {PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_OPEN_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_CLOSED_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_OPEN_LOOP, PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_CLOSED_LOOP_SMOOTH};
    }

    void setOutputVoltage(const uint16_t _voltage_adu);
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