#include <unistd.h>
#include <assert.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <deque>

#include "libapt.h"

// ====================================================================================================================
const std::string aptserial::timeStamp() {
  std::time_t currentTime = std::time(nullptr);
  char tempFileName[32];
  std::strftime(tempFileName, sizeof(tempFileName), "%Y%m%d.%H%M%S", std::localtime(&currentTime));
  return std::string(tempFileName);
}


void aptserial::LogMessage(const char* _source, const char* _function, const char* _message) {
#if APT_DEBUG 
  std::cout << "[" << aptserial::timeStamp() << "] : " << std::setw(18) << _source << " : " << std::setw(30) << _function << " : " << _message << std::endl;
#endif
}


void aptserial::LogMessage(const char* _source, const std::string& _function, const std::string& _message) {
  return aptserial::LogMessage(_source, _function.c_str(), _message.c_str());
}


void aptserial::printData(const std::string _message, const char* _ptrData, const size_t _length) {
#if APT_DEBUG
  std::cout << "-------------------------------------------------------------------" << std::endl << _message << "(" << std::setw(3) << std::setfill('0') << _length << ") : ";
  for (size_t i = 0; i < _length; i++) {
    if (i == APT_MGMSG_SIZE)
      std::cout << std::endl << std::string(_message.size(),' ') << "(" << std::setw(3) << std::setfill('0') << _length-i << ") : ";
    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(0xFF & _ptrData[i]) << " ";
  }
  std::cout << std::dec << std::endl << "-------------------------------------------------------------------" << std::endl << std::setfill(' ');
#endif /* DEBUG */
}


void aptserial::printRawData(const std::string _message, const char* _ptrData, const size_t _length) {
#if APT_DEBUG
  std::cout << "-------------------------------------------------------------------" << std::endl << _message << "(" << std::setw(3) << std::setfill('0') << _length << ") : ";
  for (size_t i = 0; i < _length; i++) {
    if (((i%0x10)==0) && (i!=0))
      std::cout << std::endl << std::string(_message.size(),' ') << "(" << std::setw(3) << std::setfill('0') << _length-i << ") : ";
    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)(0xFF & _ptrData[i]) << " ";
  }
  std::cout << std::dec << std::endl << "-------------------------------------------------------------------" << std::endl << std::setfill(' ');
#endif /* DEBUG */
}


aptserial::SerialPort::SerialPort(const std::string& _device, BaudRate _baudRate) : m_device(_device), m_baudRate(_baudRate) {
  m_echo = false;
  m_timeout_ms = APT_SERAIL_DEFAULT_TIMEOUT_MS;
  m_readBuffer.resize(APT_SERIAL_DEFAULT_BUFFER_SIZE,'\0');
  Open();
  return;
}


aptserial::SerialPort::~SerialPort() {
  try {
    Close();
  } catch(...) {
    // We can't do anything about this! But we don't want to throw within destructor, so swallow
  }
}


struct termios aptserial::SerialPort::GetTermios() {
  struct termios tty;
  if (tcgetattr(m_fileDescriptor, &tty) != 0)
    throw SerialPortException("aptdevice.cpp", "GetTermios()", "tcgetattr() failed.");
  return tty;
}


void aptserial::SerialPort::SetTermios(struct termios& _tty) {
  tcflush(m_fileDescriptor, TCIFLUSH);
  if (tcsetattr(m_fileDescriptor, TCSANOW, &_tty) != 0)
    throw SerialPortException("aptdevice.cpp", "SetTermios()", "tcsetattr() failed.");
}


void aptserial::SerialPort::ConfigureTermios() {
  // ---- configure ---------------------------------------------------------------------------------------------------
  termios tty = GetTermios();
  // ---- tty.c_cflag -------------------------------------------------------------------------------------------------
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag |= CRTSCTS; // Enable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // ---- BAUD RATE ---------------------------------------------------------------------------------------------------
  // We used to use cfsetispeed() and cfsetospeed() with the B... macros, but this didn't allow us to set custom baud rates.
  // only implement standard baudrates for simplicity. Different from https://github.com/gbmhunter/CppLinuxSerial
  cfsetispeed(&tty, (speed_t)m_baudRate);
  cfsetospeed(&tty, (speed_t)m_baudRate);

  // ----  (.c_oflag) -------------------------------------------------------------------------------------------------
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  
  // ---- CONTROL CHARACTERS (.c_cc[]) --------------------------------------------------------------------------------
  // c_cc[VTIME] sets the inter-character timer, in units of 0.1s.
  // Only meaningful when port is set to non-canonical mode
  // VMIN = 0, VTIME = 0: No blocking, return immediately with what is available
  // VMIN > 0, VTIME = 0: read() waits for VMIN bytes, could block indefinitely
  // VMIN = 0, VTIME > 0: Block until any amount of data is available, OR timeout occurs
  // VMIN > 0, VTIME > 0: Block until either VMIN characters have been received, or VTIME after first character has elapsed
  // c_cc[WMIN] sets the number of characters to block (wait) for when read() is called.
  // Set to 0 if you don't want read to block. Only meaningful when port set to non-canonical mode
  if(m_timeout_ms == -1) { // Always wait for at least one byte, this could block indefinitely
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 1;
  } else if(m_timeout_ms == 0) { // Setting both to 0 will give a non-blocking read
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;
  } else if(m_timeout_ms > 0) {
    tty.c_cc[VTIME] = (cc_t)(m_timeout_ms/100); // 0.5 seconds read timeout
    tty.c_cc[VMIN] = 0;
  }

  // ---- .c_iflag) ---------------------------------------------------------------------------------------------------
  tty.c_iflag &= ~(IXON|IXOFF|IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
  
  // ---- LOCAL MODES (c_lflag) ---------------------------------------------------------------------------------------
  // Canonical input is when read waits for EOL or EOF characters before returning.
  // In non-canonical mode, the rate at which read() returns is instead controlled by c_cc[VMIN] and c_cc[VTIME]
  tty.c_lflag &= ~ICANON; // Turn off canonical input, which is suitable for pass-through
  if(m_echo) // Configure echo depending on m_echo boolean
    tty.c_lflag |= ECHO; // Enable echo
  else
    tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Turn off echo erase (echo erase only relevant if canonical input is active)
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disables recognition of INTR (interrupt), QUIT and SUSP (suspend) characters

  SetTermios(tty);
}


void aptserial::SerialPort::Open() {
  if(m_device.empty())
    throw SerialPortException("aptdevice.cpp", "Open()", "m_device not specified [m_device.empty()]");

  m_fileDescriptor = open(m_device.c_str(), O_RDWR);
  if(m_fileDescriptor <= 0)
    throw SerialPortException("aptdevice.cpp", "Open()", "Opening m_device failed [m_fileDescriptor <= 0]");

  ConfigureTermios();

  m_state = State::OPEN;
}


size_t aptserial::SerialPort::Write(const std::vector<char>& _data) {
  if(m_state != State::OPEN)
    throw SerialPortException("aptdevice.cpp", "Write()", "Port not opened [m_state != State::OPEN]");

  if(m_fileDescriptor <= 0)
    throw SerialPortException("aptdevice.cpp", "Write()", "Opening m_device failed [m_fileDescriptor <= 0]");

  tcflush(m_fileDescriptor, TCIOFLUSH);
  size_t writeResult = write(m_fileDescriptor, _data.data(), _data.size());
  tcdrain(m_fileDescriptor);

  if (writeResult < 0)
    throw SerialPortException("aptdevice.cpp", "Write()", "Write failed.");

  return writeResult;
}


const std::vector<char> aptserial::SerialPort::Read() {
  if(m_fileDescriptor == 0)
    throw SerialPortException("aptdevice.cpp", "Read()", "Port not opened [m_fileDescriptor == 0]");

  // Read from file
  // We provide the underlying raw array from the readBuffer_ vector to this C api.
  // This will work because we do not delete/resize the vector while this method is called
  m_readBuffer.resize(APT_SERIAL_DEFAULT_BUFFER_SIZE);
  ssize_t num = read(m_fileDescriptor, &m_readBuffer[0], m_readBuffer.size());
  // Error Handling
  if(num < 0) // Read was unsuccessful
    throw SerialPortException("aptdevice.cpp", "Read()", "No bytes read.");
  else 
    m_readBuffer.resize(num);

  return m_readBuffer;
}


void aptserial::SerialPort::Close() {
  if(m_fileDescriptor != -1) {
    auto retVal = close(m_fileDescriptor);
    if(retVal != 0)
      throw SerialPortException("aptdevice.cpp", "Close()", "Tired to close but failed.");
    m_fileDescriptor = -1;
  }
  m_state = State::CLOSED;
}
// ====================================================================================================================




















// ====================================================================================================================
aptserial::APTDevice::APTDevice(const std::string _deviceFileName, const uint8_t _idSrcDest) : m_serialPort(_deviceFileName, aptserial::BaudRate::B_115200), m_idSrcDest(_idSrcDest) {
  stopUpdateMessages();
  identifyDevice(PZ_CHANNEL::CHANNEL_1);
}


aptserial::APTDevice::~APTDevice() {
  m_serialPort.Close();
  // disconnectDevice();
}


size_t aptserial::APTDevice::Write(const uint16_t _messageId, const uint8_t _destination, const uint8_t _param1, const uint8_t _param2) {
  uHeader writeUHeader;
  writeUHeader.command = {.messageId = _messageId, 
                          .param1 = _param1, 
                          .param2 = _param2,
                          .destination = _destination,
                          .source = APT_MGMSG_SRC_DEST_HOST};
  std::vector<char> sendData(writeUHeader.raw, writeUHeader.raw+sizeof(uHeader));
  printRawData("Write()     ", sendData.data(), sendData.size());
  return m_serialPort.Write(sendData);
}


size_t aptserial::APTDevice::Write(const uint16_t _messageId, const uint8_t _destination, const char* _ptrData, const uint16_t _dataLength) {
  uHeader writeUHeader;
  writeUHeader.dataCommand = {.messageId = _messageId, 
                              .dataLength = _dataLength,
                              .destination = (uint8_t)(_destination|APT_MGMSG_LONG_DATA),
                              .source = APT_MGMSG_SRC_DEST_HOST};
  std::vector<char> sendData(writeUHeader.raw, writeUHeader.raw+sizeof(uHeader));
  for (size_t i = 0; i < _dataLength; i++)
    sendData.push_back(*(_ptrData++));
  printRawData("Write()     ", sendData.data(), sendData.size());
  return m_serialPort.Write(sendData);
}


const std::vector<char> aptserial::APTDevice::Read(const uint16_t _expect, const uint8_t _expectLength) {
  std::vector<char> replyData = m_serialPort.Read();
  std::deque<char> processReplyData(replyData.begin(), replyData.end());
  try {
    while ( ((((processReplyData[1]&0xFF)<<8)+(processReplyData[0]&0xFF)) != (_expect&0xFFF)) && (processReplyData.size() > 0) ) // compare first two bytes to the expected
      processReplyData.pop_front();
  } catch(const std::exception& e) {
    throw IncorrectHeaderException("aptdevice.cpp", "Read()");
  }

  if (_expectLength != 0)
    while (_expectLength < processReplyData.size())
      processReplyData.pop_back();
  std::vector<char> returnData{processReplyData.begin(), processReplyData.end()};
  printRawData("Read()      ", returnData.data(), returnData.size());
  return returnData;
}


const std::vector<char> aptserial::APTDevice::WriteRead(const uint16_t _messageId, const uint16_t _expect, const uint8_t _destination, const uint8_t _expectLength, const uint8_t _param1, const uint8_t _param2) {
  uint tries = 0;
  std::vector<char> replyData;
  while(true) {
    try {
      size_t writeLength = Write(_messageId,_destination,_param1,_param2);
      usleep(APT_SLEEP_US);
      size_t expectReplyLength = (_expectLength==0)?writeLength:_expectLength;
      replyData = Read(_expect, expectReplyLength);
      usleep(APT_SLEEP_US);
  
      LogMessage("aptdevice.cpp","WriteRead()","writeLength = "+std::to_string(writeLength)+", replyData = "+std::to_string(replyData.size())+", expectReplyLength = "+std::to_string(expectReplyLength));

      if (replyData.size()==expectReplyLength)
        break;
      else
        continue;

    } catch (const aptserial::IncorrectHeaderException& _exception) {
      if (++tries == APT_MAX_TIRES)
        throw _exception;
    }
    LogMessage("aptdevice.cpp","WriteRead()","Attempt "+std::to_string(tries)+" of "+std::to_string(APT_MAX_TIRES));
  }
  return replyData;
}


const std::vector<char> aptserial::APTDevice::WriteRead(const uint16_t _messageId, const uint16_t _expect, const uint8_t _destination, const char* _ptrData, const uint16_t _dataLength, const uint8_t _expectLength) {
  uint tries = 0;
  std::vector<char> replyData;
  while(true) {
    try {
      size_t writeLength = Write(_messageId,_destination,_ptrData,_dataLength);
      usleep(APT_SLEEP_US);
      size_t expectReplyLength = (_expectLength==0)?writeLength:_expectLength;
      replyData = Read(_expect,expectReplyLength);
      usleep(APT_SLEEP_US);

      LogMessage("aptdevice.cpp","WriteRead()","writeLength = "+std::to_string(writeLength)+", replyData = "+std::to_string(replyData.size())+", expectReplyLength = "+std::to_string(expectReplyLength));

      if (replyData.size()==expectReplyLength)
        break;
      else
        continue;

      break;
    } catch (const aptserial::IncorrectHeaderException& _exception) {
      if (++tries == APT_MAX_TIRES)
        throw _exception;
    }
    LogMessage("aptdevice.cpp","WriteRead()","Attempt "+std::to_string(tries)+" of "+std::to_string(APT_MAX_TIRES));
  }
  return replyData;
}


void aptserial::APTDevice::updateHWInfo() {
  std::vector<char> replyData = WriteRead(APT_MGMSG_HW_REQ_INFO, APT_MGMSG_HW_GET_INFO, m_idSrcDest, sizeof(uHeader)+sizeof(stHWInfo));

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "updateHWInfo()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_HW_GET_INFO)
      throw IncorrectHeaderException("aptdevice.cpp", "updateHWInfo()");

    stHWInfo hwInfo;
    memcpy(&hwInfo, replyData.data()+sizeof(uHeader), replyData.size()-sizeof(uHeader));

    m_hwInfo.identifier = m_serialPort.GetDevice();

    char serial[20];
    snprintf(serial, 20, "%d", hwInfo.serial);
    m_hwInfo.serial = serial;

    m_hwInfo.model = hwInfo.model;

    char type[20];
    snprintf(type, 20, "%d", hwInfo.type);
    m_hwInfo.type = type;

    char firmware[20];
    snprintf(firmware, 20, "%d.%d.%d", hwInfo.fw.major, hwInfo.fw.interim, hwInfo.fw.minor);
    m_hwInfo.firmware = firmware;

    char hw[20];
    snprintf(hw, 20, "%d", hwInfo.hw);
    m_hwInfo.hardware = hw;

    char mod[20];
    snprintf(mod, 20, "%d", hwInfo.mod);
    m_hwInfo.mod = type;

    m_hwInfo.nChannels = (uint16_t)hwInfo.nChannels;
  }
}


void aptserial::APTDevice::identifyDevice(PZ_CHANNEL _channelId) {
  Write(APT_MGMSG_MOD_IDENTIFY, m_idSrcDest, (uint8_t)_channelId);
}


void aptserial::APTDevice::disconnectDevice() {
  Write(APT_MGMSG_HW_DISCONNECT, m_idSrcDest);
}


void aptserial::APTDevice::stopUpdateMessages() {
  Write(APT_MGMSG_HW_STOP_UPDATEMSGS, m_idSrcDest);
}


void aptserial::APTDevice::setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInput, const PZ_CHANNEL _channelId) {
  stChannelIOSettings channelIOSettings;
  channelIOSettings.channel = (uint16_t)_channelId;
  channelIOSettings.voltageRange = (uint16_t)_vRange;
  channelIOSettings.analogInput = (uint16_t)_analogInput;
  Write(APT_MGMSG_PZ_SET_TPZ_IOSETTINGS, m_idSrcDest, (char*)&channelIOSettings, sizeof(stChannelIOSettings));
}


void aptserial::APTDevice::getIOSettings(PZ_VOLTAGE_RANGE& _vRange, PZ_ANALOG_INPUT_SOURCE& _analogInput, const PZ_CHANNEL _channelId) {
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


void aptserial::APTDevice::setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource, const PZ_CHANNEL _channelId) {
  stChannelSource channelSource;
  channelSource.channel = (uint16_t)_channelId;
  channelSource.source = (uint16_t)_inputVoltageSource;
  Write(APT_MGMSG_PZ_SET_INPUTVOLTSSRC, m_idSrcDest, (char*)&channelSource, sizeof(stChannelSource));
}


void aptserial::APTDevice::getInputVoltageSource(PZ_INPUT_VOLTAGE_SOURCE& _inputVoltageSource, const PZ_CHANNEL _channelId) {
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


void aptserial::APTDevice::setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode, const PZ_CHANNEL _channelId) {
  Write(APT_MGMSG_PZ_SET_POSCONTROLMODE, m_idSrcDest, (uint8_t)_channelId, (uint8_t)_positionControlMode);
}


void aptserial::APTDevice::getPositionControlMode(PZ_POSITION_CONTROL_MODE& _positionControlMode, const PZ_CHANNEL _channelId) {
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


void aptserial::APTDevice::setDisplaySettings(const uint16_t _brightness_adu) {
  uint16_t brightness_adu = _brightness_adu & 0xFF;
  Write(APT_MGMSG_PZ_SET_TPZ_DISPSETTINGS, m_idSrcDest, (char*)&brightness_adu, sizeof(uint16_t));
}


void aptserial::APTDevice::getDisplaySettings(uint16_t& _brightness_adu) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_TPZ_DISPSETTINGS, APT_MGMSG_PZ_GET_TPZ_DISPSETTINGS, m_idSrcDest, sizeof(uHeader)+2);

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getDisplaySettings()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_TPZ_DISPSETTINGS)
      throw IncorrectHeaderException("aptdevice.cpp", "getDisplaySettings()");
      
    memcpy(&_brightness_adu, replyData.data()+sizeof(uHeader), sizeof(uint16_t));
  }
}


void aptserial::APTDevice::setOutputVoltage(const uint16_t _voltage_adu, const PZ_CHANNEL _channelId) {
  stChannelValue channelValue;
  channelValue.channel = (uint16_t)_channelId;
  channelValue.value = _voltage_adu;
  Write(APT_MGMSG_PZ_SET_OUTPUTVOLTS, m_idSrcDest, (char*) &channelValue, sizeof(channelValue));
}


void aptserial::APTDevice::getOutputVoltage(uint16_t& _voltage_adu, const PZ_CHANNEL _channelId) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_OUTPUTVOLTS, APT_MGMSG_PZ_GET_OUTPUTVOLTS, m_idSrcDest, sizeof(uHeader)+sizeof(stChannelValue));

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getOutputVoltage()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_OUTPUTVOLTS)
      throw IncorrectHeaderException("aptdevice.cpp", "getOutputVoltage()");

    stChannelValue channelValue;
    memcpy(&channelValue, replyData.data()+sizeof(uHeader), sizeof(stChannelValue));
      
    _voltage_adu = channelValue.value;
  }
}


void aptserial::APTDevice::setChannelEnableState(const PZ_STATE _state, const PZ_CHANNEL _channelId) {
  Write(APT_MGMSG_MOD_SET_CHANENABLESTATE, m_idSrcDest, (uint8_t)_channelId, (uint8_t)_state);
}


void aptserial::APTDevice::getChannelEnableState(PZ_STATE& _state, const PZ_CHANNEL _channelId) {
  std::vector<char> replyData = WriteRead(APT_MGMSG_PZ_REQ_PZSTATUSUPDATE, APT_MGMSG_PZ_GET_PZSTATUSUPDATE, m_idSrcDest, sizeof(uHeader)+sizeof(stStatus));

  if (replyData.size() == 0)
    throw SerialPortException("aptdevice.cpp", "getChannelEnableState()", "Reply not received");
  else {
    uHeader readHeader;
    memcpy(readHeader.raw, replyData.data(), sizeof(uHeader));

    if (readHeader.command.messageId != APT_MGMSG_PZ_GET_PZSTATUSUPDATE)
      throw IncorrectHeaderException("aptdevice.cpp", "getChannelEnableState()");

    stStatus status;
    memcpy(&status, replyData.data()+sizeof(uHeader), sizeof(stStatus));

    _state = ((bool)(status.status[3]&0b10000000))?(PZ_STATE::STATE_ENABLE):(PZ_STATE::STATE_DISABLE);
  }
}
// ====================================================================================================================