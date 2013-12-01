How To
======

Get The Sources
---------------

You can get a clone of the git repository:

* :command:`git clone` git@github.com:manchels/ARdrone.git

And to get the **ardrone_autonomy package** in the stack:

* :command:`git submodule init`

* :command:`git submodule update`

Build The Sources
-----------------

You have to install:

* `ros fuerte <http://wiki.ros.org/fuerte/Installation>`_

* `joy <http://wiki.ros.org/joy>`_ ros package.

* `opencv2 <http://wiki.ros.org/opencv2>`_ ros package.

Then just run :command:`rosmake` in the stack directory.

Other version of ros are not tested with this stack.

