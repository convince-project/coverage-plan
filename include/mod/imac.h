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
  std::unique_ptr<Eigen::MatrixXd> _entryMatrix{};
  std::unique_ptr<Eigen::MatrixXd> _exitMatrix{};
  std::unique_ptr<Eigen::MatrixXd> _initialBelief{};

public:
  /**
   * Constructor initialises the member variables.
   *
   * @param entryMatrix A ptr to the entry matrix
   * @param exitMatrix A ptr to the exit matrix
   */
  IMac(std::unique_ptr<Eigen::MatrixXd> entryMatrix,
       std::unique_ptr<Eigen::MatrixXd> exitMatrix)
      : _entryMatrix{std::move(entryMatrix)},
        _exitMatrix{std::move(exitMatrix)}, _initialBelief{} {}

  /**
   * Computes the initial belief over the map of dynamics for timestep 0.
   *
   * Here the probability in each cell represents the probability of occupation.
   *
   * @return initialBelief The initial belief over the map
   */
  std::unique_ptr<Eigen::MatrixXd> computeInitialBelief();

  /**
   * Runs a given state through iMac to get the distribution for the next
   * timestep.
   *
   * Here the probability in each cell represents the probability of occupation.
   *
   * @param currentState a 2D matrix of the current map state
   * @return nextState a 2D matrix of the subsequent map state
   */
  std::unique_ptr<Eigen::MatrixXd>
  forwardStep(std::unique_ptr<Eigen::MatrixXd> currentState);
};

#endif