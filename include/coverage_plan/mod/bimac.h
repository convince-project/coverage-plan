/**
 * Header file for the BIMac class.
 *
 * BiMac is an extension of IMac which explicitly maintains the distribution
 * over the IMac parameters.
 *
 * @author Charlie Street
 */
#ifndef BIMAC_H
#define BIMAC_H

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <filesystem>
#include <memory>

/**
 * Struct for storing BiMac observations.
 *
 * Members:
 * x: X position on grid
 * y: Y position on grid
 * freeToOccupied: Number of observations at (x,y) going from free to occupied
 * freeToFree: Number of observations at (x,y) going from free to free
 * occupiedToFree: Number of observations at (x,y) going from occupied to free
 * occupiedToOccupied: Number of observations at (x,y) going from occupied to
 * occupied
 */
struct BIMacObservation {
  int x{};
  int y{};
  int freeToOccupied{};
  int freeToFree{};
  int occupiedToFree{};
  int occupiedToOccupied{};
};

/**
 * A class which maintains our uncertainty over the true IMac model.
 * @see mod/imac.h
 * This class maintains a Beta distribution for each transition at each grid
 * cell.
 *
 * Members:
 *  _alphaEntry: The matrix of alpha parameters for the lambda_entry
 * parameter (free->occupied) _betaEntry: The matrix of beta parameters for
 * the lambda_entry parameter (free->occupied) _alphaExit: The matrix of
 * alpha parameters for the lambda_exit parameter (occupied->free)
 * _betaExit: The matrix of beta parameters for the lambda_exit parameter
 * (occupied->free)
 */
class BIMac {
private:
  Eigen::MatrixXi _alphaEntry{};
  Eigen::MatrixXi _betaEntry{};
  Eigen::MatrixXi _alphaExit{};
  Eigen::MatrixXi _betaExit{};

  /**
   * Reads BIMac matrix in from file.
   *
   * @param inFile The Matrix file to read in
   * @return mat The matrix read from file
   */
  Eigen::MatrixXi _readBIMacMatrix(std::filesystem::path inFile);

  /**
   * Write a single BIMac matrix to file.
   *
   * @param matrix The matrix to write out
   * @param outFile The file to send the BIMac matrix
   */
  void _writeBIMacMatrix(Eigen::MatrixXi matrix, std::filesystem::path outFile);

public:
  /**
   * This constructor initialises all matrices to be full of ones.
   *
   * @param x The length of the x dimension of the grid map
   * @param y The length of the y dimension of the grid map
   */
  BIMac(int x, int y)
      : _alphaEntry{Eigen::MatrixXi::Ones(x, y)},
        _betaEntry{Eigen::MatrixXi::Ones(x, y)},
        _alphaExit{Eigen::MatrixXi::Ones(x, y)}, _betaExit{
                                                     Eigen::MatrixXi::Ones(
                                                         x, y)} {}

  /**
   * This constructor reads a BIMac config in from file.
   *
   * @param inDir The directory where the IMac files are stored
   */
  BIMac(std::filesystem::path inDir)
      : _alphaEntry{inDir / "alpha_entry.txt"}, _betaEntry{inDir /
                                                           "beta_entry.txt"},
        _alphaExit{inDir / "alpha_exit.txt"}, _betaExit{inDir /
                                                        "beta_exit.txt"} {}

  /**
   * Sample from BIMac to get a single IMac instance.
   *
   * @return imac A shared ptr to an imac instance
   */
  std::shared_ptr<IMac> sample();

  /**
   * Compute the MLE given the observed data.
   *
   * Given how BiMac is initialised, this corresponds to computing the mode
   * of each beta distribution.
   *
   * @return imac A shared ptr to an imac instance
   */
  std::shared_ptr<IMac> mle();

  /**
   * Update the BIMac posterior given a new set of observations.
   *
   * @param observations A list of BIMacObservations
   */
  void updatePosterior(std::vector<BIMacObservation> observations);

  /**
   * Write BiMac matrices out to file.
   *
   * @param outDir The directory to write the BIMac matrices to
   */
  void writeBIMac(std::filesystem::path outDir);
};
#endif