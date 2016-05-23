##This is a ROS node for [sf30 rangefinder](http://www.lightware.co.za/shop/en/4-drone-altimeters)##

The node assume the following setup for the laser (I used the [Lightware](http://www.lightware.co.za/shop/en/content/8-software) terminal from the manufacture):

  
```
#!term

   1: Active data port           USB distance in m
   2: Resolution                 0.03 m
   3: Serial port update rate    1000 / sec  (actual = 1665 / sec)
   4: Serial port baud rate      115200
   5: Analog port update rate    1 / sec  (actual = 1 / sec)
   6: Analog maximum range       256 m
   7: Alarm activation distance  17.50 m
   8: Alarm latch                Off
   9: USB port update rate       50 / sec  (actual = 50 / sec)

```

Before you leave the terminal, make sure the sensor is sending messages of the form (hit space to make it happen):


```
#!term
0.57 m
0.57 m
0.59 m
0.57 m
0.59 m
0.59 m
0.55 m
0.59 m
0.57 m
0.57 m
0.57 m

```


###To install the package: ###

Create a catkin workspace. For instructions on how to create the workspace go [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). Download and compile the package:


```
#!bash

cd catkin_ws/src
git clone git@bitbucket.org:castacks/sf30_node.git
cd ..
catkin_make
```


###To run the package: ###

In a terminal run:

```
#!bash
source devel/setup.bash
roslaunch sf30_node sf30.launch


```

The message of type `sensor_msgs::LaserScan` will be published in topic `/sf30/range` at 50Hz. The intensities field on this message means data confidence. It is `1` if we can trust the given range.

### Who do I talk to? ###

* Guilherme Pereira - gpereira@ufmg.br