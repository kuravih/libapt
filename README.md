# libapt

C++ Library for the Thorlabs APT Protocol

This Library is for communicating with the Thorlabs Kinesis devices. This implements the full [APT protocol - Issue 29 - 20 October 2021](https://www.thorlabs.com/Software/Motion%20Control/APT_Communications_Protocol.pdf), but have been tested with the [KPZ101 piezo controller](https://www.thorlabs.com/thorproduct.cfm?partnumber=KPZ101) and [KST101 stepper motor controller](https://www.thorlabs.com/thorproduct.cfm?partnumber=KST101) (with [ZFS25B](https://www.thorlabs.com/thorproduct.cfm?partnumber=ZFS25B)) on Ubuntu 20.04.3.

## Installation

No installation required. Build the static library and link to your code. See example. The `APTDevice` class may be extended for other devices similar to the implementation of the `KPZ101` and `KST101` devices. 

### Dependencies

No special dependencies other than `build-essential`.