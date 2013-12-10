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

* :ref:`Filtered images <tracking>`

* :ref:`Ball detetection <tracking>`

* Debug console:

.. _console:

  .. figure:: _static/launch_console.png
    :width: 80%
    :align: center
    :alt: alternate text

  ..

  This console is a very convenient tools to debug, because you can can filter
  the messages to display. Here, it has been configured the to display also **DEBUG**
  messages which is not the case by default in a simple terminal.

To use it:

* :command:`roslaunch ardrone_launch` :file:`ardrone_red_ball_control_with_gui.launch`

Rosbag Red Ball Tracker
+++++++++++++++++++++++

| This is a test bench for the |RED_BALL_TRACKER| based on a rosbag file.
| It display also the :ref:`console <console>` tool.

To use it:

* :command:`roslaunch ardrone_launch` :file:`test_rosbag_red_ball_tracker.launch`

ARDrone Red Ball Tracker
++++++++++++++++++++++++

| This is a test bench for the |RED_BALL_TRACKER| based on the ARDrone camera.
| It display also the :ref:`console <console>` tool.

To use it:

* :command:`roslaunch ardrone_launch` :file:`test_ardrone_red_ball_tracker.launch`
