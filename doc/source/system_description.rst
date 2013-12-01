System Description
==================

In the stack you can find the following ros packages:

.. |RED_BALL_TRACKER| replace:: :ref:`red_ball_tracker <red_ball_tracker>`
.. |TELEOP|           replace:: :ref:`teleop <teleop>`

.. tabularcolumns:: |p{3cm}|L|

==================== ============================================================================
Package              Description
==================== ============================================================================
ardrone_autonomy     It contain the ardrone driver. If you don't want to
                     download it. Don't use the :command:`git submodule`
                     then you clone the repository.
-------------------- ----------------------------------------------------------------------------
ardrone_bags         It contains some rosbag used to make tests.
-------------------- ----------------------------------------------------------------------------
ardrone_launch       It contains the launch files of the project.
-------------------- ----------------------------------------------------------------------------
doc                  This documentation. (sphinx)
-------------------- ----------------------------------------------------------------------------
|RED_BALL_TRACKER|   Where the image processing is done. Detect the nearest red ball.
-------------------- ----------------------------------------------------------------------------
|TELEOP|             Where the command given to the **ardrone_driver** are processed.
==================== ============================================================================
