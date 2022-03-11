#include "kpz101.h"

#include <string.h>

// ====================================================================================================================
const std::string aptserial::voltageRangeToString(const PZ_VOLTAGE_RANGE _vRange) {
  const char* vRangeStrings[(uint)PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_N_RANGES] = PZ_VOLTAGE_RANGE_LABELS;
  return std::string(vRangeStrings[(uint)_vRange]);
}


const bool aptserial::stringToVoltageRange(const std::string _vRangeString, PZ_VOLTAGE_RANGE& _vRange) {
  const char* vRangeStrings[(uint)PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_N_RANGES] = PZ_VOLTAGE_RANGE_LABELS;
  auto itVRange = std::find(std::begin(vRangeStrings), std::end(vRangeStrings), _vRangeString);
  if (itVRange != std::end(vRangeStrings)) {
    _vRange = (PZ_VOLTAGE_RANGE)(uint)std::distance(std::begin(vRangeStrings), itVRange);
    return true;
  }
  return false;
}


const std::string aptserial::analogInputSourceToString(const PZ_ANALOG_INPUT_SOURCE _analogInputSource) {
  const char* analogInputSourceStrings[(uint)PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_N_SOURCES] = PZ_ANALOG_INPUT_SOURCE_LABELS;
  return std::string(analogInputSourceStrings[(uint)_analogInputSource]);
}


const bool aptserial::stringToAnalogInputSource(const std::string _analogInputSourceString, PZ_ANALOG_INPUT_SOURCE& _analogInputSource) {
  const char* analogInputSourceStrings[(uint)PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_N_SOURCES] = PZ_ANALOG_INPUT_SOURCE_LABELS;
  auto itAnalogInputSource = std::find(std::begin(analogInputSourceStrings), std::end(analogInputSourceStrings), _analogInputSourceString);
  if (itAnalogInputSource != std::end(analogInputSourceStrings)) {
    _analogInputSource = (PZ_ANALOG_INPUT_SOURCE)(uint)std::distance(std::begin(analogInputSourceStrings), itAnalogInputSource);
    return true;
  }
  return false;
}


const std::string aptserial::inputVoltageSourceToString(const PZ_INPUT_VOLTAGE_SOURCE _inputVSource) {
  const char* inputVSourceStrings[(uint)PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_N_SOURCES] = PZ_INPUT_VOLTAGE_SOURCE_LABELS;
  return std::string(inputVSourceStrings[(uint)_inputVSource]);
}


const bool aptserial::stringToInputVoltageSource(const std::string _inputVSourceString, PZ_INPUT_VOLTAGE_SOURCE& _inputVSource) {
  const char* inputVSourceStrings[(uint)PZ_INPUT_VOLTAGE_SOURCE::INPUT_VOLTAGE_SOURCE_N_SOURCES] = PZ_INPUT_VOLTAGE_SOURCE_LABELS;
  auto itInputVSource = std::find(std::begin(inputVSourceStrings), std::end(inputVSourceStrings), _inputVSourceString);
  if (itInputVSource != std::end(inputVSourceStrings)) {
    _inputVSource = (PZ_INPUT_VOLTAGE_SOURCE)(uint)std::distance(std::begin(inputVSourceStrings), itInputVSource);
    return true;
  }
  return false;
}


const std::string aptserial::positionControlModeToString(const PZ_POSITION_CONTROL_MODE _positionCtrlMode) {
  const char* positionCtrlModeStringStrings[(uint)PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_N_MODES] = PZ_POSITION_CONTROL_MODE_LABELS;
  return std::string(positionCtrlModeStringStrings[(uint)_positionCtrlMode]);
}


const bool aptserial::stringToPositionControlMode(const std::string _positionCtrlModeString, PZ_POSITION_CONTROL_MODE& _positionCtrlMode) {
  const char* positionCtrlModeStringStrings[(uint)PZ_POSITION_CONTROL_MODE::POSITION_CONTROL_MODE_N_MODES] = PZ_POSITION_CONTROL_MODE_LABELS;
  auto itPositionControlModeSource = std::find(std::begin(positionCtrlModeStringStrings), std::end(positionCtrlModeStringStrings), _positionCtrlModeString);
  if (itPositionControlModeSource != std::end(positionCtrlModeStringStrings)) {
    _positionCtrlMode = (PZ_POSITION_CONTROL_MODE)(uint)std::distance(std::begin(positionCtrlModeStringStrings), itPositionControlModeSource);
    return true;
  }
  return false;
}


aptserial::KPZ101::KPZ101(const std::string _deviceFileName, const uint8_t _idSrcDest) : APTDevice(_deviceFileName, _idSrcDest), m_channelId(PZ_CHANNEL::CHANNEL_1) {
  updateHWInfo();
}


void aptserial::KPZ101::setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInput, const APT_CHANNEL _channelId) {
  stChannelIOSettings channelIOSettings;
  channelIOSettings.channel = (uint16_t)_channelId;
  channelIOSettings.voltageRange = (uint16_t)_vRange;
  channelIOSettings.analogInput = (uint16_t)_analogInput;
  Write(APT_MGMSG_PZ_SET_TPZ_IOSETTINGS, m_idSrcDest, (char*)&channelIOSettings, sizeof(stChannelIOSettings));
}


void aptserial::KPZ101::getIOSettings(PZ_VOLTAGE_RANGE& _vRange, PZ_ANALOG_INPUT_SOURCE& _analogInput, const APT_CHANNEL _channelId) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_TPZ_IOSETTINGS, APT_MGMSG_PZ_GET_TPZ_IOSETTINGS, m_idSrcDest, sizeof(uHeader)+sizeof(stChannelIOSettings));

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getIOSettings()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_TPZ_IOSETTINGS)
      throw IncorrectHeaderException("aptdevice.cpp", "getIOSettings()");

    stChannelIOSettings channelIOSettings;
    memcpy(&channelIOSettings, replyData.data()+sizeof(uHeader), sizeof(stChannelIOSettings));

    _vRange = (PZ_VOLTAGE_RANGE)channelIOSettings.voltageRange;
    _analogInput = (PZ_ANALOG_INPUT_SOURCE)channelIOSettings.analogInput;
  }
}


void aptserial::KPZ101::setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInputSource) {
  setIOSettings(_vRange, _analogInputSource, m_channelId);
  updateIOSettings();
}


void aptserial::KPZ101::updateIOSettings() {
  getIOSettings(m_vRange, m_analogInputSource, m_channelId);
}


void aptserial::KPZ101::setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource, const APT_CHANNEL _channelId) {
  stChannelSource channelSource;
  channelSource.channel = (uint16_t)_channelId;
  channelSource.source = (uint16_t)_inputVoltageSource;
  Write(APT_MGMSG_PZ_SET_INPUTVOLTSSRC, m_idSrcDest, (char*)&channelSource, sizeof(stChannelSource));
}


void aptserial::KPZ101::getInputVoltageSource(PZ_INPUT_VOLTAGE_SOURCE& _inputVoltageSource, const APT_CHANNEL _channelId) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_INPUTVOLTSSRC, APT_MGMSG_PZ_GET_INPUTVOLTSSRC, m_idSrcDest, sizeof(uHeader)+sizeof(stChannelSource));

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getInputVoltageSource()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_INPUTVOLTSSRC)
      throw IncorrectHeaderException("aptdevice.cpp", "getInputVoltageSource()");

    stChannelSource channelSource;
    memcpy(&channelSource, replyData.data()+sizeof(uHeader), sizeof(stChannelSource));
      
    _inputVoltageSource = (PZ_INPUT_VOLTAGE_SOURCE)channelSource.source;
  }
}


void aptserial::KPZ101::setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource) {
  KPZ101::setInputVoltageSource(_inputVoltageSource, m_channelId);
  updateInputVoltageSource();
}


void aptserial::KPZ101::updateInputVoltageSource() {
  KPZ101::getInputVoltageSource(m_inputVoltageSource, m_channelId);
}


void aptserial::KPZ101::setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode, const APT_CHANNEL _channelId) {
  Write(APT_MGMSG_PZ_SET_POSCONTROLMODE, m_idSrcDest, (uint8_t)_channelId, (uint8_t)_positionControlMode);
}


void aptserial::KPZ101::getPositionControlMode(PZ_POSITION_CONTROL_MODE& _positionControlMode, const APT_CHANNEL _channelId) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_POSCONTROLMODE, APT_MGMSG_PZ_GET_POSCONTROLMODE, m_idSrcDest);

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getPositionControlMode()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_POSCONTROLMODE)
      throw IncorrectHeaderException("aptdevice.cpp", "getPositionControlMode()");

    _positionControlMode = (PZ_POSITION_CONTROL_MODE)readHeader.command.param2;
  }
}


void aptserial::KPZ101::setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode) {
  aptserial::KPZ101::setPositionControlMode(_positionControlMode, m_channelId);
  updatePositionControlMode();
}


void aptserial::KPZ101::updatePositionControlMode() {
  aptserial::KPZ101::getPositionControlMode(m_positionControlMode, m_channelId);
}


void aptserial::KPZ101::setOutputVoltage(const uint16_t _voltage_adu) {
  aptserial::APTDevice::setOutputVoltage(_voltage_adu, m_channelId);
  updateOutputVoltage();
}


void aptserial::KPZ101::updateOutputVoltage() {
  aptserial::APTDevice::getOutputVoltage(m_voltage_adu, m_channelId);
}


void aptserial::KPZ101::setChannelEnableState(const PZ_STATE _state) {
  aptserial::APTDevice::setChannelEnableState(_state, m_channelId);
  updateChannelEnableState();
}


void aptserial::KPZ101::updateChannelEnableState() {
  aptserial::APTDevice::getChannelEnableState(m_state, m_channelId);
}
// ====================================================================================================================