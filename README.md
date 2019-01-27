# CM730

This repository contains a suite of ROS 2 packages to work with the
[CM730 sub
controller](http://support.robotis.com/en/product/darwin-op/references/reference/hardware_specifications/electronics/sub_controller_(cm-730).htm). More
specifically, it is set up to work with a robot that has a combination
of a CM-730 and Dynamixel motors (specifically MX-28s), such as the
[DARwIn-OP](http://support.robotis.com/en/product/darwin-op.htm) or
derivatives. It should also work with (or be easy to adapt for)
similar components/robots, that use the same Dynamixel protocol, such
as the [ROBOTIS OP2 with a
CM-740](http://www.robotis.us/robotis-op2-us/), but this has not been
tested.

The following diagram shows the different components of the suite and
their interactions:

![CM-730 ROS 2 diagram](https://robocuplab.herts.ac.uk/ros2/cm730driver/raw/master/cm730ros2diagram.svg)

## `cm730driver`

Performs the actual communication with the CM-730 over USB, exposes
ROS 2 services that map to the Dynamixel serial protocol.

## `cm730controller`

Runs a 125Hz loop that reads and writes to the cm730driver. It
transforms the raw byte messages from the driver to structured
`CM730Info` and `MX28Info` messages and publishes these. It also
subscribes to commands to be transformed into driver write requests
(TODO).

## `mx_joint_state_publisher`

Transforms motor information messages into standard ROS 2
`sensor_msgs/JointState` messages, to be used for instance by ROS 2's
`robot_state_publisher` node.
