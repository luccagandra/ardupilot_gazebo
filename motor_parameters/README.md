# Motor parameters

This folder containst Matlab scripts that estimate:

* *Ct* - Motor thrust constant
* *Cm* - Motor moment constant

Additional information to be found in:

```bibtex
@Inbook{Furrer2016,
author="Furrer, Fadri
and Burri, Michael
and Achtelik, Markus
and Siegwart, Roland",
editor="Koubaa, Anis",
chapter="RotorS---A Modular Gazebo MAV Simulator Framework",
title="Robot Operating System (ROS): The Complete Reference (Volume 1)",
year="2016",
publisher="Springer International Publishing",
address="Cham",
pages="595--625",
isbn="978-3-319-26054-9",
doi="10.1007/978-3-319-26054-9_23",
url="http://dx.doi.org/10.1007/978-3-319-26054-9_23"
}
```

## Usage

Using the static propeller performance information found in [APC propellers](https://www.apcprop.com/technical-information/performance-data/) compile
mat files containing RPM, thrust and torque vectors as presented in:

* *22x8.mat*
* *22x11e.mat*

By running *get_thrust_and_torque_k.m* script with the appropriate input .mat file respective motor constants can be obtained.