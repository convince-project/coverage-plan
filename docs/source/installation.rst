Installation Instructions
=========================

.. role:: bash(code)
   :language: bash

COVERAGE-PLAN has been tested on Ubuntu 22.04 with CMake 3.22.

Dependencies
------------

COVERAGE-PLAN requires the following dependencies:

* `Boost`_ (Tested with 1.82)
* `Catch`_ (Tested with 2.13.8-1, installed via :bash:`apt install` on Ubuntu 22.04)
* `Eigen`_ (Tested with 3.3)
* `Despot`_ (Tested with :bash:`API_redesign` branch)

Eigen may not work after installing via apt. You may need to create a symlink in :bash:`/usr/include`:

.. code-block:: bash

	cd /usr/include
	sudo ln -sf eigen3/Eigen Eigen
	sudo ln -sf eigen3/unsupported unsupported


Building COVERAGE-PLAN
----------------------

After installing the dependencies, run the following in the root directory:

.. code-block:: bash

	cmake -S . -B build
	cmake --build build

Running Unit Tests
------------------

To run the unit tests, run the following in the root directory:

.. code-block:: bash

	cd build/tests
	./unitTests

Examples
--------

Upon building, example files can be found in the :bash:`build/apps` directory.


Build the Documentation
-----------------------

If you want to build it locally, do the following:

1. Install the required packages:

.. code-block:: bash
    
    sudo apt install doxygen
    pip install -r docs/requirements.txt

2. Build the documentation:

.. code-block:: bash
    
    cd docs
    make html

3. Look at the documentation:

.. code-block:: bash
    
    cd docs
    firefox build/html/index.html

Clean Documentation Build Artifacts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to clean the documentation, you can run:

.. code-block:: bash
	
	cd docs
	make clean

.. _Boost: https://linux.how2shout.com/how-to-install-boost-c-on-ubuntu-20-04-or-22-04/
.. _Catch: https://github.com/catchorg/Catch2
.. _Eigen: https://eigen.tuxfamily.org/index.php?title=Main_Page
.. _Despot: https://github.com/AdaCompNUS/despot

