#include <iostream>
#include <unistd.h>

#include "kpz101.h"
#include "kst101.h"

using namespace std;

int main() {

  aptserial::KST101 stepper1("/dev/ttyUSB1");
  sleep(1);
  cout << "-------------------------------------------------------" << "\n";
  cout << "HWInfo            : [" << stepper1.getHWInfo().toString() << "]" << "\n";
  stepper1.setActuatorType(ST_ACTUATOR_TYPE::ACTUATOR_TYPE_ZFS_NEW_25MM);
  // stepper1.moveHome();
  cout << "absolute position : [" << (float)stepper1.getPosition_mm() << "]" << "\n";
  stepper1.setPositionAbsolute_mm(7);
  cout << "absolute position : [" << (float)stepper1.getPosition_mm() << "]" << "\n";
  stepper1.setPositionRelative_mm(-1.5);
  cout << "absolute position : [" << (float)stepper1.getPosition_mm() << "]" << "\n";


  aptserial::KST101 stepper2("/dev/ttyUSB2");
  sleep(1);
  cout << "-------------------------------------------------------" << "\n";
  cout << "HWInfo            : [" << stepper2.getHWInfo().toString() << "]" << "\n";
  stepper2.setActuatorType(ST_ACTUATOR_TYPE::ACTUATOR_TYPE_ZFS_NEW_25MM);
  // stepper2.moveHome();
  cout << "absolute position : [" << (float)stepper2.getPosition_mm() << "]" << "\n";
  stepper2.setPositionAbsolute_mm(7);
  cout << "absolute position : [" << (float)stepper2.getPosition_mm() << "]" << "\n";
  stepper2.setPositionRelative_mm(-1.5);
  cout << "absolute position : [" << (float)stepper2.getPosition_mm() << "]" << "\n";

  return 0;
}