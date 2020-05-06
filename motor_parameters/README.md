# Motor parameters

This folder containst Matlab scripts that estimate:

* *Ct* - Motor thrust constant
* *Cm* - Motor moment constant

## Usage

Using the static propeller performance information found in [APC propellers](https://www.apcprop.com/technical-information/performance-data/) compile
mat files containing RPM, thrust and torque vectors as seen in files:

* *22x8.mat*
* *22x11e.mat*

By running *get_thrust_and_torque_k.m* script with the appropriate input .mat file respective motor constants can be obtained.