/**
 * Header file defining the IMac class.
 * IMac is a map of dynamics which uses an independent Markov chain at each grid
 * cell.
 * @author Charlie Street
 */
#ifndef IMAC_H
#define IMAC_H

#include <Eigen/Dense>
#include <memory>

/**
 * Interactive Markov chain map of dynamics (IMac).
 * This class contains 2 matrices representing the transitions in 2 state Markov
 * chains at every cell on a grid.
 *
 * Members:
 * _entryMatrix: A matrix where _entryMatrix(x,y) is the (free->occupied)
 * probability for (x,y)
 * _exitMatrix: A matrix where _exitMatrix(x,y) is the (occupied->free)
 * probability for (x,y)
 * _initialBelief: A 2D matrix representing the initial
 * belief over each cell being occupied.
 */
class IMac {
private:
  const Eigen::MatrixXd _entryMatrix{};
  const Eigen::MatrixXd _exitMatrix{};
  Eigen::MatrixXd _initialBelief{};

public:
  /**
   * Constructor initialises the member variables.
   *
   * @param entryMatrix The entry matrix
   * @param exitMatrix The exit matrix
   */
  IMac(const Eigen::MatrixXd &entryMatrix, const Eigen::MatrixXd &exitMatrix)
      : _entryMatrix{entryMatrix}, _exitMatrix{exitMatrix}, _initialBelief{} {}

  /**
   * Computes the initial belief over the map of dynamics for timestep 0.
   *
   * Here the probability in each cell represents the probability of occupation.
   *
   * @return initialBelief The initial belief over the map
   */
  Eigen::MatrixXd computeInitialBelief();

  /**
   * Runs a given belief or state through iMac to get the distribution for the
   * next timestep.
   *
   * Passing a deterministic state into this function is fine, and it'll still
   * work. However, note the arg type is double, not int, so casting is probably
   * required first.
   *
   * Here the probability in each cell represents the probability of occupation.
   *
   * @param currentBelief a 2D matrix of the current map belief or state
   * @return nextBelief a 2D matrix of the subsequent map belief
   */
  Eigen::MatrixXd forwardStep(const Eigen::MatrixXd &currentBelief);

  /**
   * Getter for _entryMatrix. Need to retrieve for experimental purposes.
   *
   * @return entryMatrix A copy of_entryMatrix
   */
  Eigen::MatrixXd getEntryMatrix() { return this->_entryMatrix; }

  /**
   * Getter for _exitMatrix. Need to retrieve for experimental purposes.
   *
   * @return exitMatrix A copy of_exitMatrix
   */
  Eigen::MatrixXd getExitMatrix() { return this->_exitMatrix; }
};

#endif