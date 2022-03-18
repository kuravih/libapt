#include "kst101.h"

#include <string.h>
#include <unistd.h>

// ====================================================================================================================
const std::string aptserial::actuatorTypeToString(const ST_ACTUATOR_TYPE _actType) {
  const char* actTypeStrings[(uint)ST_ACTUATOR_TYPE::ACTUATOR_TYPE_N_TYPES] = ST_ACTUATOR_TYPE_LABELS;
  return std::string(actTypeStrings[(uint)_actType]);
}


const bool aptserial::stringToActuatorType(const std::string _actTypeString, ST_ACTUATOR_TYPE& _actType) { 
  const char* actTypeStrings[(uint)ST_ACTUATOR_TYPE::ACTUATOR_TYPE_N_TYPES] = ST_ACTUATOR_TYPE_LABELS;
  auto itActType = std::find(std::begin(actTypeStrings), std::end(actTypeStrings), _actTypeString);
  if (itActType != std::end(actTypeStrings)) {
    _actType = (ST_ACTUATOR_TYPE)(uint)std::distance(std::begin(actTypeStrings), itActType);
    return true;
  }
  return false;
}
// ====================================================================================================================





// ====================================================================================================================
aptserial::KST101::KST101(const std::string _deviceFileName) : APTDevice(_deviceFileName, APT_MGMSG_SRC_DEST_GENERIC_USB) {
  updateHWInfo();
  stopMoving(); // stop any movement
  setVelocityParameters(0, 136533*100, 136533*200); // set to default parameters
  updateVelocityParameters();
  updatePositionCounter();
  setMovementParameters(0);
  updateMovementParameters();
}





void aptserial::KST101::setActuatorType(const ST_ACTUATOR_TYPE _actType) {
  Write(APT_MGMSG_MOT_SET_TSTACTUATORTYPE, m_idSrcDest, (uint8_t)_actType);
}





void aptserial::KST101::setVelocityParameters(const uint32_t _minVelocity, const uint32_t _acceleration, const uint32_t _maxVelocity) {
  stVelocityParameters velocityParameters;
  velocityParameters.minVelocity = _minVelocity;
  velocityParameters.acceleration = _acceleration;
  velocityParameters.maxVelocity = _maxVelocity;
  Write(APT_MGMSG_MOT_SET_VELPARAMS, m_idSrcDest, (char*)&velocityParameters, sizeof(velocityParameters));
  updateVelocityParameters();
}

void aptserial::KST101::getVelocityParameters(uint32_t& _minVelocity, uint32_t& _acceleration, uint32_t& _maxVelocity) {
  std::vector<char> replyData = Try_Write_Read(APT_MGMSG_MOT_REQ_VELPARAMS, APT_MGMSG_MOT_GET_VELPARAMS, m_idSrcDest, sizeof(uHeader)+sizeof(stVelocityParameters));

  if (replyData.size() == 0)
    throw SerialPortException("kst101.cpp", "getVelocityParameters()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_MOT_GET_VELPARAMS)
      throw IncorrectHeaderException("kst101.cpp", "getVelocityParameters()");

    stVelocityParameters velocityParameters;
    memcpy(&velocityParameters, replyData.data()+sizeof(uHeader), sizeof(stVelocityParameters));

    _minVelocity = velocityParameters.minVelocity;
    _acceleration = velocityParameters.acceleration;
    _maxVelocity = velocityParameters.maxVelocity;
  }
}

void aptserial::KST101::updateVelocityParameters() {
  getVelocityParameters(m_minVelocity, m_acceleration, m_maxVelocity);
}





void aptserial::KST101::setMovementParameters(const uint32_t _backlash) {
  stChannelValue channelValue;
  channelValue.channel = (uint16_t)APT_CHANNEL::CHANNEL_1;
  channelValue.value = _backlash;
  Write(APT_MGMSG_MOT_SET_GENMOVEPARAMS, m_idSrcDest, (char*)&channelValue, sizeof(channelValue));
  updateMovementParameters();
}

void aptserial::KST101::getMovementParameters(uint32_t& _backlash) {
  std::vector<char> replyData = Try_Write_Read(APT_MGMSG_MOT_REQ_GENMOVEPARAMS, APT_MGMSG_MOT_GET_GENMOVEPARAMS, m_idSrcDest, sizeof(uHeader)+sizeof(stChannelValue));

  if (replyData.size() == 0)
    throw SerialPortException("kst101.cpp", "getMovementParameters()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_MOT_GET_GENMOVEPARAMS)
      throw IncorrectHeaderException("kst101.cpp", "getMovementParameters()");

    stChannelValue channelValue;
    memcpy(&channelValue, replyData.data()+sizeof(uHeader), sizeof(stChannelValue));

    _backlash = channelValue.value;
  }
}

void aptserial::KST101::updateMovementParameters() {
  getMovementParameters(m_backlash);
}





void aptserial::KST101::setPositionCounter(const int32_t _position_tick) {
  stChannelValue channelValue;
  channelValue.channel = (uint16_t)APT_CHANNEL::CHANNEL_1;
  channelValue.value = _position_tick;
  Write(APT_MGMSG_MOT_SET_POSCOUNTER, m_idSrcDest, (char*)&channelValue, sizeof(channelValue));
  updatePositionCounter();
}

void aptserial::KST101::getPositionCounter(int32_t& _position_tick) {
  std::vector<char> replyData = Try_Write_Read(APT_MGMSG_MOT_REQ_POSCOUNTER, APT_MGMSG_MOT_GET_POSCOUNTER, m_idSrcDest, sizeof(uHeader)+sizeof(stChannelValue), (uint8_t)APT_CHANNEL::CHANNEL_1);

  if (replyData.size() == 0)
    throw SerialPortException("kst101.cpp", "getPositionCounter()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_MOT_GET_POSCOUNTER)
      throw IncorrectHeaderException("kst101.cpp", "getPositionCounter()");

    stChannelValue channelValue;
    memcpy(&channelValue, replyData.data()+sizeof(uHeader), sizeof(stChannelValue));

    _position_tick = channelValue.value;
  }
}

void aptserial::KST101::updatePositionCounter() {
  getPositionCounter(m_position_tick);
}





void aptserial::KST101::setPositionRelative() {
  Write(APT_MGMSG_MOT_MOVE_RELATIVE, m_idSrcDest, sizeof(uHeader)+sizeof(stMotorStatus), (uint8_t)APT_CHANNEL::CHANNEL_1); // assume not failing
  usleep(APT_SLEEP_US);

  uint tries = 0;
  std::vector<char> replyData, data;
  uint8_t expectReplyLength = sizeof(uHeader)+sizeof(stMotorStatus);
  while(true) {
    try {
      data = m_serialPort.Read();
      usleep(APT_SLEEP_US);
      replyData.insert(replyData.end(), data.begin(), data.end());
      LogMessage("libapt.cpp","WriteRead()","replyData = "+std::to_string(replyData.size())+", expectReplyLength = "+std::to_string(expectReplyLength));
      if (replyData.size()==expectReplyLength)
        break;
    } catch (const aptserial::IncorrectHeaderException& _exception) {
      if (++tries == APT_MAX_TIRES)
        throw _exception;
    }
  }
}

void aptserial::KST101::setPositionRelative(const int32_t _position_tick) {
  stChannelValue channelValue;
  channelValue.channel = (uint16_t)APT_CHANNEL::CHANNEL_1;
  channelValue.value = _position_tick;
  Write(APT_MGMSG_MOT_MOVE_RELATIVE, m_idSrcDest, (char*)&channelValue, sizeof(channelValue)); // assume not failing
  usleep(APT_SLEEP_US);

  uint tries = 0;
  std::vector<char> replyData, data;
  uint8_t expectReplyLength = sizeof(uHeader)+sizeof(stMotorStatus);
  while(true) {
    try {
      data = m_serialPort.Read();
      usleep(APT_SLEEP_US);
      replyData.insert(replyData.end(), data.begin(), data.end());
      LogMessage("libapt.cpp","WriteRead()","replyData = "+std::to_string(replyData.size())+", expectReplyLength = "+std::to_string(expectReplyLength));
      if (replyData.size()==expectReplyLength)
        break;
    } catch (const aptserial::IncorrectHeaderException& _exception) {
      if (++tries == APT_MAX_TIRES)
        throw _exception;
    }
  }
  stMotorStatus status;
  memcpy(&status, replyData.data()+sizeof(uHeader), sizeof(stMotorStatus));
  m_position_tick = status.position;
}





void aptserial::KST101::setPositionAbsolute(const int32_t _position_tick) {
  stChannelValue channelValue;
  channelValue.channel = (uint16_t)APT_CHANNEL::CHANNEL_1;
  channelValue.value = _position_tick;
  Write(APT_MGMSG_MOT_MOVE_ABSOLUTE, m_idSrcDest, (char*)&channelValue, sizeof(channelValue)); // assume not failing
  usleep(APT_SLEEP_US);

  uint tries = 0;
  std::vector<char> replyData, data;
  uint8_t expectReplyLength = sizeof(uHeader)+sizeof(stMotorStatus);
  while(true) {
    try {
      data = m_serialPort.Read();
      usleep(APT_SLEEP_US);
      replyData.insert(replyData.end(), data.begin(), data.end());
      LogMessage("libapt.cpp","WriteRead()","replyData = "+std::to_string(replyData.size())+", expectReplyLength = "+std::to_string(expectReplyLength));
      if (replyData.size()==expectReplyLength)
        break;
    } catch (const aptserial::IncorrectHeaderException& _exception) {
      if (++tries == APT_MAX_TIRES)
        throw _exception;
    }
  }
  stMotorStatus status;
  memcpy(&status, replyData.data()+sizeof(uHeader), sizeof(stMotorStatus));
  m_position_tick = status.position;
}





void aptserial::KST101::moveHome() {
  std::vector<char> replyData = Try_Write_Read(APT_MGMSG_MOT_MOVE_HOME, APT_MGMSG_MOT_MOVE_HOMED, m_idSrcDest, sizeof(uHeader), (uint8_t)APT_CHANNEL::CHANNEL_1);

  if (replyData.size() == 0)
    throw SerialPortException("kst101.cpp", "moveHome()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_MOT_MOVE_HOMED)
      throw IncorrectHeaderException("kst101.cpp", "moveHome()");

    stMotorStatus status;
    memcpy(&status, replyData.data()+sizeof(uHeader), sizeof(stMotorStatus));

    m_position_tick = status.position;
  }
}





void aptserial::KST101::stopMoving() {
  Write(APT_MGMSG_MOT_MOVE_STOP, m_idSrcDest, (uint8_t)APT_CHANNEL::CHANNEL_1);
}





void aptserial::KST101::setChannelEnableState(const ST_STATE _state) {
  aptserial::APTDevice::setChannelEnableState(_state, APT_CHANNEL::CHANNEL_1);
  updateChannelEnableState();
}





void aptserial::KST101::updateChannelEnableState() {
  aptserial::APTDevice::getChannelEnableState(m_state, APT_CHANNEL::CHANNEL_1);
}
// ====================================================================================================================