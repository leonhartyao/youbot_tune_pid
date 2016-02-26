# youBot_pid
==============

The default controller parameters from manufactory are not optimal, it leads to delay and overshoot in the motion. This repository contains source code to tune youbot arm pid parameters. The code has been developed under the following setup:

* Ubuntu 14.04 with ROS-indigo


Before You Tune The PID Parameters!
------------------------------

In order to tune pid parameters for each joint, you will have to have a overview of cascade control loops for the KUKA youBot arm. You can find a description of the motor controller in the following link:

http://www.robocupatwork.org/download/RoboCup-At-Work_Camp_2012/RAW_Camp2012-Jan-Paulus_youBotAPI.pdf

Installation
------------

### Dependencies

yoBot driver:

    sudo apt-get install ros-indigo-youbot-driver

The brics_actuator packages:

    sudo apt-get install ros-hydro-brics-actuator
    
Additionally, you need an adapted version of the ros wrapper of the driver which you have to clone into the src folder of your catkin workspace

    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/youbot_driver_ros_interface.git

this is an adapted version, which provides torque command interface.

youBot_Arm_configurator is uses to configure parameters.

rqt_plot package is used to visualize responses.

    sudo apt-get install ros-indigo-rqt-plot

### Main Installation

You can download the source code by running
    
    cd catkin_ws/src
    git clone https://github.com/chaolmu/youbot_tune_pid.git
    
Then, you can simply compile it

    cd catkin_ws
    catkin_make
    
Basic Usage
-----------

Since it is a cascade controller, you have to start tuning with inner loop. It means current loop -> velocity loop -> position loop. 

### current loop

The current controller should be tested without moving the joint. PID parameters could be tuned with respect to step response. We normally use a simple I-controller for current loop. 
Disable arm calibration in youbot driver if you want joints stay.

[WARNING]: You have to move the joint, which will be tested, to its stop and give the input in the direction that joint will not leave its stop.  Otherwise hardware damage may occur! 
[WARNING]: Joint 3 has opposite direction with compare to others.

Here are step response before and after tuning:

  ![alt tag](http://i65.tinypic.com/209rq4n.png)
  ![alt tag](http://i68.tinypic.com/25t86k8.png)

### velocity loop

To tuning parameters you have to disable ramp generator. The velocity controller could be tested with step response, the control loop can't be faster when it reaches max. acceleration.
The following ability can be tested with trapezoid input.

A example before and after tuning:
  
  ![alt tag](http://i66.tinypic.com/qxqnpw.png)
  ![alt tag](http://i66.tinypic.com/mw5u7a.png)

### position loop

  ![alt tag](http://i67.tinypic.com/110gg2c.png)

a example after tuning:
  ![alt tag](http://i63.tinypic.com/2h5jg9s.png)
