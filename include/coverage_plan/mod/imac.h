/**
 * @file imac.h
 *
 * @brief Header file defining the IMac class.
 *
 * IMac is a map of dynamics which uses an independent Markov chain at each grid
 * cell.
 *
 * Based on the paper: Saarinen, J., Andreasson, H. and Lilienthal, A.J., 2012,
 * October. Independent markov chain occupancy grid maps for representation of
 * dynamic environment. In 2012 IEEE/RSJ International Conference on Intelligent
 * Robots and Systems (pp. 3489-3495). IEEE.
 *
 * @author Charlie Street
 */
#ifndef IMAC_H
#define IMAC_H

#include <Eigen/Dense>
#include <filesystem>
#include <memory>

/**
 * Interactive Markov chain map of dynamics (IMac).
 * This class contains 2 matrices representing the transitions in 2 state Markov
 * chains at every cell on a grid.
 *
 * Members:
 * * _entryMatrix: A matrix where _entryMatrix(x,y) is the (free->occupied)
 * probability for (x,y)
 * * _exitMatrix: A matrix where _exitMatrix(x,y) is the (occupied->free)
 * probability for (x,y)
 * * _initialBelief: A 2D matrix representing the initial
 * belief over each cell being occupied
 * * _staticOccupancy: A 2D matrix estimating the static occupancy of a cell
 */
class IMac {
private:
  const Eigen::MatrixXd _entryMatrix{};
  const Eigen::MatrixXd _exitMatrix{};
  const Eigen::MatrixXd _initialBelief{};
  Eigen::MatrixXd _staticOccupancy{};

  /**
   * Reads IMac matrix in from file.
   *
   * @param inFile The file to read the matrix in from
   *
   * @returns The IMac matrix
   */
  Eigen::MatrixXd _readIMacMatrix(const std::filesystem::path &inFile);

  /**
   * Write a single IMac matrix to file.
   *
   * @param matrix The matrix to write out
   * @param  outFile The file to write the IMac matrix to
   */
  void _writeIMacMatrix(const Eigen::MatrixXd &matrix,
                        const std::filesystem::path &outFile);

public:
  /**
   * Constructor initialises the member variables.
   *
   * @param entryMatrix The entry matrix
   * @param exitMatrix The exit matrix
   * @param initialBelief The initial belief matrix
   */
  IMac(const Eigen::MatrixXd &entryMatrix, const Eigen::MatrixXd &exitMatrix,
       const Eigen::MatrixXd &initialBelief)
      : _entryMatrix{entryMatrix}, _exitMatrix{exitMatrix},
        _initialBelief{initialBelief}, _staticOccupancy{} {}

  /**
   * This constructor reads an IMac config in from file.
   *
   * @param inDir The directory where the IMac files are stored
   */
  IMac(const std::filesystem::path &inDir)
      : _entryMatrix{this->_readIMacMatrix(inDir / "entry.csv")},
        _exitMatrix{this->_readIMacMatrix(inDir / "exit.csv")},
        _initialBelief{this->_readIMacMatrix(inDir / "initial_belief.csv")},
        _staticOccupancy{} {}

  /**
   *  Estimates the static occupancy of the map.
   *
   * This is the time-abstract probability of the cells being occupied.
   * A probability of 1 represents the cell being occupied.
   *
   * @returns The static occupancy matrix
   */
  Eigen::MatrixXd estimateStaticOccupancy();

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
   * @returns a 2D matrix of the subsequent map belief
   */
  Eigen::MatrixXd forwardStep(const Eigen::MatrixXd &currentBelief) const;

  /**
   * Getter for _entryMatrix. Need to retrieve for experimental purposes.
   *
   * @returns A copy of_entryMatrix
   */
  Eigen::MatrixXd getEntryMatrix() const { return this->_entryMatrix; }

  /**
   * Getter for _exitMatrix. Need to retrieve for experimental purposes.
   *
   * @returns A copy of_exitMatrix
   */
  Eigen::MatrixXd getExitMatrix() const { return this->_exitMatrix; }

  /**
   * Return the initial belief over the map of dynamics.
   *
   * Here the probability in each cell represents the probability of occupation.
   *
   * @returns The initial belief over the map
   */
  Eigen::MatrixXd getInitialBelief() const { return this->_initialBelief; }

  /**
   * Write IMac matrices out to file.
   *
   * @param outDir The directory to write the BIMac matrices to
   */
  void writeIMac(const std::filesystem::path &outDir);
};

#endif