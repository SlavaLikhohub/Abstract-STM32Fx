################
Abstract-STM32Fx
################

Description
***********

This library is an abstraction over `libopencm3 <https://github.com/libopencm3/libopencm3/tree/24bef9c49eda109e92e926e065b246a71d454f2d>`_ library including
 
 - Working with GPIO and GPIO groups
 
 - Delay

 - Working with LCD display

 - Systick

Requirment
**********

- `OpenOCD <http://openocd.org>`_.

- `arm-none-eabi Toolchain <https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm>`_

- `Doxygen <https://doxygen.nl>`_ and `GraphViz <https://graphviz.org/>`_ for building libopencm3 documentation

Add user to plugdev group:
~~~~~~~~~~~~~~~~~~~~~~~~~
This step is required to allow working with debuggers OpenOCD supports as a user, without a need
for having root privileges.

OpenOCD package on Arch comes with udev rules file (``/usr/lib/udev/rules.d/60-openocd.rules``).
It gives access rights to users in plugdev group, which exists on Debian, but is not present
on Arch Linux. So we need to create the group and add our user to it:

.. code-block:: shell-session
   
   sudo groupadd -r -g 46 plugdev
   sudo useradd -G plugdev $USER

And log out (or reboot)

Installing packages in Manjaro:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
.. code-block:: shell-session
   
   yay -S openocd-git
   sudo pacman -S arm-none-eabi-{gcc,binutils,gdb,newlib} doxygen graphviz

.. note::
   You need to either run ``sudo udevadm control --reload-rules`` and ``sudo udevadm trigger``
   or to reboot after installing OpenOCD for udev rules to start working

Build
*****

.. code-block:: shell-session
   
   make PROFILE=release V=1 clean all

Documentation
*************

Build documentation

To build a documentation install 

#. **Sphinx** with version not more than 2.4.3 (Versions 3.x.x do not support crossreferencing in ``Hawkmoth``)

.. code-block:: shell-session

   pip install sphinx==2.4.3
   pip install sphinx_rtd_theme

#. **Hawkmoth** (tested with 0.5 version)

.. code-block:: shell-session

   pip install hawkmoth

.. code-block:: shell-session

   make documentation

``abstractSTM32.html`` link will apear in ``doc`` directory.

Example
*******

See `Abstact-STM32Fx-Sample <https://github.com/SlavaLikhohub/Abstract-STM32Fx-Sample>`_.
