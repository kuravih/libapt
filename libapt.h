#ifndef __APTDEVICE_H__
#define __APTDEVICE_H__

#pragma once

#include <termios.h> // Contains POSIX terminal control definitions
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>
#include <iomanip>

#include "apt_def.h"

#define APT_DEBUG 0
#define APT_MGMSG_SIZE 6
#define APT_SERAIL_DEFAULT_TIMEOUT_MS 1000
#define APT_SERIAL_DEFAULT_BUFFER_SIZE 255
#define APT_SERIAL_DEFAULT_BAUDRATE BaudRate::B_115200
#define APT_SERIAL_DEFAULT_DEVICE "/dev/ttyUSB0"
#define APT_MAX_TIRES 5
#define APT_SLEEP_US 1000

// custom baudrates are not implemented for a complete implementation see 
// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
// https://github.com/gbmhunter/CppLinuxSerial/blob/master/src/SerialPort.cpp
namespace aptserial {

  enum class BaudRate : speed_t {
    B_57600 = B57600,
    B_115200 = B115200,
    B_230400 = B230400,
    B_460800 = B460800,
    B_500000 = B500000,
    B_576000 = B576000,
    B_921600 = B921600,
    B_1000000 = B1000000,
    B_1152000 = B1152000,
    B_1500000 = B1500000,
    B_2000000 = B2000000,
    B_2500000 = B2500000,
    B_3000000 = B3000000,
    B_3500000 = B3500000,
    B_4000000 = B4000000,
  };

  enum class State {
    CLOSED,
    OPEN,
  };

  const std::string timeStamp();
  void printData(const std::string _message, const char* _ptrData, const size_t _length);
  void printRawData(const std::string _message, const char* _ptrData, const size_t _length);
  void LogMessage(const char* _source, const char* _function, const char* _message);
  void LogMessage(const char* _source, const std::string& _function, const std::string& _message="");

  class SerialPortException : public std::runtime_error {
  private:
    std::string m_msg;
  public:
    explicit SerialPortException(const char* _source, const char* _function, const char* _message) : std::runtime_error(_message) {
      std::stringstream out;
      out << "[" << timeStamp() << "] : " << std::setw(18) << std::string(_source) << " : " << std::setw(30) << std::string(_function) << " : " << std::string(_message);
      m_msg = out.str();
    }
    ~SerialPortException() throw() {}
    const char *what() const throw() override {
      return m_msg.c_str();
    }
  };

  // ---- SerialPort --------------------------------------------------------------------------------------------------
  class SerialPort {
  private:
    int m_fileDescriptor;
    std::string m_device;
    BaudRate m_baudRate = BaudRate::B_115200;
    speed_t m_baudRateCustom; // future
    int32_t m_timeout_ms = APT_SERAIL_DEFAULT_TIMEOUT_MS;
    bool m_echo;
    State m_state;
    std::vector<char> m_readBuffer;
    struct termios GetTermios();
    void SetTermios(struct termios& _tty);
    void ConfigureTermios();
  public:
    SerialPort(const std::string& _device=APT_SERIAL_DEFAULT_DEVICE, BaudRate _baudRate=APT_SERIAL_DEFAULT_BAUDRATE);
    ~SerialPort();
    void SetDevice(const std::string& _device) {
      m_device = _device;  m_device = _device;
    }
    const std::string GetDevice() {
      return m_device;
    }
    void SetBaudRate(BaudRate _baudRate) {
      m_baudRate = _baudRate;
    }
    void SetTimeout(int32_t _timeout_ms) {
      m_timeout_ms = _timeout_ms;
    }
    void SetEcho(bool _echo) {
      m_echo = _echo;
    }
    void Open();
    void Close();
    size_t Write(const std::vector<char>& _data);
    const std::vector<char> Read();
  };
  // ==================================================================================================================




















  // ==================================================================================================================
  // page 31
  struct stCommand { 
    uint16_t messageId;
    uint8_t param1;
    uint8_t param2;
    uint8_t destination;
    uint8_t source;
  } __attribute__((packed));

  // page 31
  struct stDataCommand { 
    uint16_t messageId;
    uint16_t dataLength;
    uint8_t destination;
    uint8_t source;
  } __attribute__((packed));

  // #define APT_MGMSG_MAX_DATA_SIZE 0xFFFF // max uint16_t
  #define APT_MGMSG_MAX_DATA_SIZE 0xFF
  #define APT_MGMSG_SIZE 6
  union uHeader { 
    struct stCommand command;
    struct stDataCommand dataCommand;
    char raw[APT_MGMSG_SIZE] = {0}; // guarantee zeros
  };

  // page 47
  struct stHWInfo {
    uint32_t serial = 0x00000000;
    char model[8] = {0};
    uint16_t type;
    struct stFW {
      uint8_t minor = 0x00;
      uint8_t interim = 0x00;
      uint8_t major = 0x00;
      uint8_t unused = 0x00;
    } __attribute__((packed)) fw;
    char internal[60] = {0};
    uint16_t hw = 0x0000;
    uint16_t mod = 0x0000;
    uint16_t nChannels = 0x0000;
    // char __more[84] = {0};
  } __attribute__((packed));

  struct stStatus {
    uint16_t channel = 0x0000;
    uint16_t voltage = 0x0000;
    uint16_t position = 0x0000;
    uint8_t status[4] = {0};
  };

  // page 184
  struct stChannelIOSettings {
    uint16_t channel = 0x0000;
    uint16_t voltageRange = 0x0000;
    uint16_t analogInput = 0x0000;
    uint16_t future1 = 0x0000;
    uint16_t future2 = 0x0000;
  };

  struct stChannelSource {
    uint16_t channel = 0x0000;
    uint16_t source = 0x0000;
  };

  // page 158, 159
  struct stChannelValue {
    uint16_t channel = 0x0000;
    uint16_t value = 0x0000;
  };

  struct HWInfo {
    std::string identifier;
    std::string serial;
    std::string model;
    std::string type;
    std::string firmware;
    std::string hardware;
    std::string mod;
    int nChannels;
    std::string toString() const {
      return "identifier: "+identifier+", model: "+model+", serial: "+serial+", type:"+type;
    }
  };
  inline bool operator==(const HWInfo& lhs, const HWInfo& rhs) {
    return ((lhs.serial == rhs.serial) && (lhs.model == rhs.model));
  }

  class IncorrectHeaderException : public SerialPortException {
  private:
    std::string m_msg;
  public:
    explicit IncorrectHeaderException(const char* _source, const char* _function) : SerialPortException(_source, _function, "Incorrect Header") {}
    ~IncorrectHeaderException() throw() {}
    const char *what() const throw() override {
      return SerialPortException::what();
    }
  };

  // ---- APTDevice ---------------------------------------------------------------------------------------------------
  class APTDevice {
  protected:
    SerialPort m_serialPort;
    HWInfo m_hwInfo;
    const uint8_t m_idSrcDest;
    void updateHWInfo();
    size_t Write(const uint16_t _messageId, const uint8_t _destination, const uint8_t _param1=0, const uint8_t _param2=0);
    size_t Write(const uint16_t _messageId, const uint8_t _destination, const char* _ptrData, const uint16_t _dataLength);
    const std::vector<char> Read(const uint16_t _expect, const uint8_t _expectLength=0);
    const std::vector<char> WriteRead(const uint16_t _messageId, const uint16_t _expect, const uint8_t _destination, const uint8_t _expectLength=0, const uint8_t _param1=0, const uint8_t _param2=0);
    const std::vector<char> WriteRead(const uint16_t _messageId, const uint16_t _expect, const uint8_t _destination, const char* _ptrData, const uint16_t _dataLength, const uint8_t _expectLength=0);
  public:
    APTDevice(const std::string _deviceFileName, const uint8_t _idSrcDest);
    ~APTDevice();
    HWInfo getHWInfo() {
      return m_hwInfo;
    }
    void stopUpdateMessages(); 
    void identifyDevice(const PZ_CHANNEL _channelId);
    void setIOSettings(const PZ_VOLTAGE_RANGE _vRange, const PZ_ANALOG_INPUT_SOURCE _analogInputSource, const PZ_CHANNEL _channelId);
    void getIOSettings(PZ_VOLTAGE_RANGE& _vRange, PZ_ANALOG_INPUT_SOURCE& _analogInputSource, const PZ_CHANNEL _channelId);

    void setInputVoltageSource(const PZ_INPUT_VOLTAGE_SOURCE _inputVoltageSource, const PZ_CHANNEL _channelId);
    void getInputVoltageSource(PZ_INPUT_VOLTAGE_SOURCE& _inputVoltageSource, const PZ_CHANNEL _channelId);

    void setPositionControlMode(const PZ_POSITION_CONTROL_MODE _positionControlMode, const PZ_CHANNEL _channelId);
    void getPositionControlMode(PZ_POSITION_CONTROL_MODE& _positionControlMode, const PZ_CHANNEL _channelId);

    void setDisplaySettings(const uint16_t _brightness_adu);
    void getDisplaySettings(uint16_t& _brightness_adu);

    void setOutputVoltage(const uint16_t _voltage_adu, const PZ_CHANNEL _channelId);
    void getOutputVoltage(uint16_t& _voltage_adu, const PZ_CHANNEL _channelId);

    void setChannelEnableState(const PZ_STATE _state, const PZ_CHANNEL _channelId);
    void getChannelEnableState(PZ_STATE& _state, const PZ_CHANNEL _channelId);

    void disconnectDevice();
  };
 // ===================================================================================================================









  










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

  class KPZ101 : public APTDevice {
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
    ~KPZ101();
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


#endif /* __APTDEVICE_H__ */