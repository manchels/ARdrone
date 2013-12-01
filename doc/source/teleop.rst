.. _teleop:

Teleop
======

This node is designed to solve the command to give to the ardrone_driver node,
using the joystick's and :ref:`red_ball_tracker <red_ball_tracker>`'s commands.

To run it:

* :command:`rosrun teleop teleop`

Modes
-----

There is two running modes that you can use:

.. tabularcolumns:: |C|L|

=============== ===========================================================================
Mode            Description
=============== ===========================================================================
Joy Control     In this mode, the movements of the ARDrone are controlled by joystick.
--------------- ---------------------------------------------------------------------------
Tracking        In this mode, the ARDrone follow the nearest red ball that it see.
=============== ===========================================================================

Joystick
--------

Here is the desciption of the joystick fonctionalities.

.. tabularcolumns:: |C|L|

=============== ===========================================================================
Joystick        Description
=============== ===========================================================================
button 0        Toggle move activation.
--------------- ---------------------------------------------------------------------------
button 1        Switch between mode.
--------------- ---------------------------------------------------------------------------
button 2        Toggle land/take off in **joy control mode**.
--------------- ---------------------------------------------------------------------------
button 3        Toggle ARDrone emergency mode.
--------------- ---------------------------------------------------------------------------
axe 0           Lateral movement.
--------------- ---------------------------------------------------------------------------
axe 1           Move forward and backward.
--------------- ---------------------------------------------------------------------------
axe 2           Change yaw.
--------------- ---------------------------------------------------------------------------
axe 3           Change altitude.
=============== ===========================================================================

.. warning::

  When you start the node you have to active movement (button 0) before 
  trying to use it.  

..

Parameters
----------

You can change the following parameters using :command:`rosparam`:

.. tabularcolumns:: |p{2.3cm}|p{1.2cm}|L|

============== ======= ===================================================================
Name           Type    Description                            
============== ======= ===================================================================
user_distance  double  Distance between the user and the ball.
-------------- ------- -------------------------------------------------------------------
dist_x_p       double  **P** parameter of the **PID** controller
                       for distance between the user and the ball.
-------------- ------- -------------------------------------------------------------------
dist_x_i       double  **I** parameter of the **PID** controller for distance
                       between the user and the ball.
-------------- ------- -------------------------------------------------------------------
dist_x_d       double  **D** parameter of the **PID** controller for
                       distance between the user and the ball.
-------------- ------- -------------------------------------------------------------------
theta_z_p      double  **P** parameter of the **P** controller for angle between
                       the axe ball-drone and the drone orientation.
============== ======= ===================================================================

Messages
--------

Here is the list of the messages used by the node.

Publishers
++++++++++

+--------------------+---------------------+-----------------------------------------------+
| Name               | Type                | Decritption                                   |
+====================+=====================+===============================================+
| ardrone/reset      | std_msgs/Empty      | Reset message for the ARDrone driver.         |
+--------------------+---------------------+-----------------------------------------------+
| ardrone/land       | std_msgs/Empty      | Order to the ARDrone to land.                 |
+--------------------+---------------------+-----------------------------------------------+
| ardrone/takeoff    | std_msgs/Empty      | Order to the ARDrone to take off.             |
+--------------------+---------------------+-----------------------------------------------+
| cmd_vel            | geometry_msgs/Twist | Command velocity to the ARDrone.              |
+--------------------+---------------------+-----------------------------------------------+

Subscriber
++++++++++

.. tabularcolumns:: |p{3.4cm}|p{2.9cm}|L|

+--------------------+---------------------+-----------------------------------------------+
| Name               | Type                | Decritption                                   |
+====================+=====================+===============================================+
| joy                | sensor_msgs/Joy     | Reset message for the ARDrone driver.         |
+--------------------+---------------------+-----------------------------------------------+
| ardrone/image_raw/ | red_ball_tracker/   | Order to the ARDrone to land.                 |
| red_ball_tracking  | TrackerMsg          |                                               |
+--------------------+---------------------+-----------------------------------------------+
