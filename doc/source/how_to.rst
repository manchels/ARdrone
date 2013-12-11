How To
======

Get The Sources
---------------

You can get a clone of the git repository:

* :command:`git clone`
  `git@github.com:manchels/ARdrone.git <https://github.com/manchels/ARdrone>`_

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

Documentation
+++++++++++++

If you want to build the documentation, you have to install these applications
too:

* `python3 <http://www.python.org/download/>`_

* `python-sphinx3 <http://sphinx-doc.org/install.html>`_

* `latex <http://latex-project.org/ftp.html>`_ if you want to build as pdf.

then go to the :ref:`doc <doc>` directory and type:

* :command:`make <target>` if you use linux.

* :command:`start make.bat <target>` if you use windows.

where **<target>** is one of these:

* html

* latex

* latexpdf

* man

* text

* qthelp

* json

* epub

* devhelp

* info