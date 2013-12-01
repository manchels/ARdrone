Launch Facilities
=================

Once you have build the stack, you can use the following launch files
to run the stuff:

.. |RED_BALL_TRACKER| replace:: :ref:`red_ball_tracker <red_ball_tracker>`
.. |TELEOP|           replace:: :ref:`teleop <teleop>`

Red Ball Control
++++++++++++++++

This is the main launcher that you need to use.

This launch:

* The **ardrone_driver**

* The |RED_BALL_TRACKER|

* The |TELEOP|

To use it:

* :command:`roslaunch ardrone_launch` :file:`ardrone_red_ball_control.launch`

Red Ball Control With Gui
+++++++++++++++++++++++++

This is the same than the previous, but this one will display:

* Filtered images

* Ball detetection

* Debug console

To use it:

* :command:`roslaunch ardrone_launch` :file:`ardrone_red_ball_control_with_gui.launch`

Rosbag Red Ball Tracker
+++++++++++++++++++++++

This is a test bench for the |RED_BALL_TRACKER| based on a rosbag file.

To use it:

* :command:`roslaunch ardrone_launch` :file:`test_rosbag_red_ball_tracker.launch`

ARDrone Red Ball Tracker
++++++++++++++++++++++++

This is a test bench for the |RED_BALL_TRACKER| based on the ARDrone camera.

To use it:

* :command:`roslaunch ardrone_launch` :file:`test_ardrone_red_ball_tracker.launch`
