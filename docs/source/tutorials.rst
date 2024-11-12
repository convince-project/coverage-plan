Tutorials
=========

.. role:: bash(code)
   :language: bash
   
.. role:: cpp(code)
   :language: cpp

How to run the POMDP coverage planner in a known environment
------------------------------------------------------------

In this tutorial, we will show how to run COVERAGE-PLAN in a known (and static) environment.

The source code for this tutorial can be found `here`_.

**Consider a robot tasked with covering a static 3x3 grid map. The robot starts at cell (0,0) and must cover as much of the
environment by timestep 13 as possible. The robot can observe the cells in its four neighbourhood.**

Our first step is to include all necessary code, as below:

.. code-block:: cpp

    #include "coverage_plan/mod/grid_cell.h"
    #include "coverage_plan/mod/imac.h"
    #include "coverage_plan/mod/imac_executor.h"
    #include "coverage_plan/planning/pomdp_coverage_robot.h"
    #include <Eigen/Dense>
    #include <filesystem>
    #include <fstream>
    #include <memory>
    #include <random>
    #include <vector>


Next, we create our `iMac`_ model which describes the dynamics of our environment.
An ``IMac`` object has three attributes: an ``entry`` matrix, which describes the probability that each
cell transitions from free to occupied within a timestep; an ``exit`` matrix, which describes the probability
that each cell transitions from occupied to free within a timestep; and an ``init`` matrix, which describes the 
probability of each cell being initially occupied. We achieve this through the below function:

.. code-block:: cpp

    /**
     * Create a random IMac instance for this example.
     *
     * @return imac A shared pointer to an IMac instance
     */
    std::shared_ptr<IMac> createIMac() {
      Eigen::MatrixXd entry{Eigen::MatrixXd::Zero(3, 3)};
      Eigen::MatrixXd exit{Eigen::MatrixXd::Ones(3, 3)};
      Eigen::MatrixXd init{Eigen::MatrixXd::Zero(3, 3)};

      return std::make_shared<IMac>(entry, exit, init);
    }

We can now look at our ``main`` function. First, we create an instance of our iMac model:

.. code-block:: cpp

    std::shared_ptr<IMac> imac{createIMac()};

We now create an ``IMacExecutor`` which acts as our simulator of the real environment.
COVERAGE-PLAN plans online, and so we require a 'world' our planning agent can act on.
In practice, this would be a ROS simulation or a real robot. Here, it is an ``IMacExecutor``, which
allows us to sample through our known iMac model:

.. code-block:: cpp

    // Create the IMac Executor
    std::shared_ptr<IMacExecutor> exec{std::make_shared<IMacExecutor>(imac)};


Next we define more parameters for our planning agent. The robot's field of view is defined as 
a vector of ``GridCell`` objects defined relative to the robot's current position. We also define the robot's 
initial position (0,0), and the time bound for planning (13):

.. code-block:: cpp

    // The robot's field of view
    std::vector<GridCell> fov{GridCell{-1, 0}, GridCell{1, 0}, GridCell{0, -1},
                            GridCell{0, 1}};

    GridCell initPos{0, 0};
    int timeBound{13};

With this, we can now create our ``POMDPCoverageRobot`` planning agent. This object takes the robot's position, the planning
time bound, the x and y dimensions of the grid map, the robot's field of view, the ``IMacExecutor`` which acts as our
'world', and our true iMac model. If the true iMac model is unknown, this parameter is not provided and the robot
will learn it over multiple coverage episodes.

.. code-block:: cpp

    // Assume true IMac model known
    std::unique_ptr<POMDPCoverageRobot> robot{
        std::make_unique<POMDPCoverageRobot>(initPos, timeBound, 3, 3, fov, exec,
                                             imac)};

After creating the ``POMDPCoverageRobot``, we can now run ``runCoverageEpisode`` to run an episode of online
coverage planning. This function takes a path to a CSV file as input. After the coverage episode has finished,
a list of the robot's position at each timestep is written here.

.. code-block:: cpp

    // Start example
    robot->runCoverageEpisode("pomdpCoverageRobotExampleVisited.csv");

This concludes the tutorial. Though this example is very simplistic, hopefully it gives a rough idea about how to use 
COVERAGE-PLAN.
This tutorial code can be adapted to learn an environment with unknown dynamics by not passing in the ground truth iMac model to the :bash:`POMDPCoverageRobot class`.
Then, as :cpp:`runCoverageEpisode` is run repeatedly over multiple episodes, the robot will learn the environment dynamics.

.. _here: https://github.com/convince-project/coverage-plan/blob/main/apps/pomdp_coverage_example.cpp
.. _iMac: https://ieeexplore.ieee.org/abstract/document/6385629
