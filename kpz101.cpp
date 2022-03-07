#include "kpz101.h"

// ====================================================================================================================
const std::string aptserial::channelToString(const PZ_CHANNEL _channel) {
  const char* channelStrings[(uint)PZ_CHANNEL::CHANNEL_N_CHANNELS] = PZ_CHANNEL_LABELS;
  return std::string(channelStrings[(uint)_channel]);
}


const bool aptserial::stringToChannel(const std::string _channelString, PZ_CHANNEL& _channel) {
  const char* channelStrings[(uint)PZ_CHANNEL::CHANNEL_N_CHANNELS] = PZ_CHANNEL_LABELS;
  auto itChannel = std::find(std::begin(channelStrings), std::end(channelStrings), _channelString);
  if (itChannel != std::end(channelStrings)) {
    _channel = (PZ_CHANNEL)(uint)std::distance(std::begin(channelStrings), itChannel);
    return true;
  }
  return false;
}


const std::string aptserial::stateToString(const PZ_STATE _state) {
  const char* stateStrings[(uint)PZ_STATE::STATE_N_STATES] = PZ_STATE_LABELS;
  return std::string(stateStrings[(uint)_state]);
}


const bool aptserial::stringToState(const std::string _stateString, PZ_STATE& _state) {
  const char* stateStrings[(uint)PZ_STATE::STATE_N_STATES] = PZ_STATE_LABELS;
  auto itState = std::find(std::begin(stateStrings), std::end(stateStrings), _stateString);
  if (itState != std::end(stateStrings)) {
    _state = (PZ_STATE)(uint)std::distance(std::begin(stateStrings), itState);
    return true;
  }
  return false;
}


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


void aptserial::KPZ101::setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInputSource) {
  aptserial::APTDevice::setIOSettings(_vRange, _analogInputSource, m_channelId);
  updateIOSettings();
}


void aptserial::KPZ101::updateIOSettings() {
  getIOSettings(m_vRange, m_analogInputSource, m_channelId);
}


void aptserial::KPZ101::setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource) {
  APTDevice::setInputVoltageSource(_inputVoltageSource, m_channelId);
  updateInputVoltageSource();
}


void aptserial::KPZ101::updateInputVoltageSource() {
  APTDevice::getInputVoltageSource(m_inputVoltageSource, m_channelId);
}


void aptserial::KPZ101::setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode) {
  APTDevice::setPositionControlMode(_positionControlMode, m_channelId);
  updatePositionControlMode();
}


void aptserial::KPZ101::updatePositionControlMode() {
  APTDevice::getPositionControlMode(m_positionControlMode, m_channelId);
}


void aptserial::KPZ101::setOutputVoltage(const uint16_t _voltage_adu) {
  APTDevice::setOutputVoltage(_voltage_adu, m_channelId);
  updateOutputVoltage();
}


void aptserial::KPZ101::updateOutputVoltage() {
  APTDevice::getOutputVoltage(m_voltage_adu, m_channelId);
}


void aptserial::KPZ101::setChannelEnableState(const PZ_STATE _state) {
  APTDevice::setChannelEnableState(_state, m_channelId);
  updateChannelEnableState();
}


void aptserial::KPZ101::updateChannelEnableState() {
  APTDevice::getChannelEnableState(m_state, m_channelId);
}
// ====================================================================================================================