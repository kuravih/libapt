#include <iostream>
#include <unistd.h>

#include "kpz101.h"

using namespace std;

int main() {

  aptserial::KPZ101 piezo("/dev/ttyUSB0");

  sleep(1);

  cout << "Serial number     : [" << piezo.getHWInfo().serial << "]" << "\n";
  cout << "Model             : [" << piezo.getHWInfo().model << "]" << "\n";
  cout << "Type              : [" << piezo.getHWInfo().type << "]" << "\n";
  cout << "Firmware version  : [" << piezo.getHWInfo().firmware << "]" << "\n";
  cout << "Hardware          : [" << piezo.getHWInfo().hardware << "]" << "\n";
  cout << "Mod               : [" << piezo.getHWInfo().mod << "]" << "\n";
  cout << "Channels          : [" << piezo.getHWInfo().nChannels << "]" << "\n";

  cout << "HWInfo            : [" << piezo.getHWInfo().toString() << "]" << "\n";

  piezo.setIOSettings(PZ_VOLTAGE_RANGE::VOLTAGE_RANGE_75V, PZ_ANALOG_INPUT_SOURCE::ANALOG_INPUT_SOURCE_A);

  sleep(1);

  return 0;
}