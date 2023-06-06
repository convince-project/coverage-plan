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
#include <random>

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

  /**
   * Sample single IMac parameter.
   *
   * Samples a uniform random number in [0,1] and runs it through the Beta
   * quantile function
   *
   * Note neither gen or sampler are passed by const reference as they maintain
   * an internal state which we want to change to ensure different random
   * numbers are generated
   *
   * @param alpha The alpha value for the corresponding Beta distribtion
   * @param beta The beta value for the corresponding Beta distribtion
   * @param gen A random number generator
   * @param sampler A sampler from a uniform distribution which uses gen
   *
   * @return paramSample The sampled IMac parameter value
   */
  double _sampleForCell(int alpha, int beta, std::mt19937 &gen,
                        std::uniform_real_distribution<double> &sampler);

  /**
   * Compute the MLE value for a single parameter.
   * Used in a nullary expression in mle
   *
   * @param alpha The alpha value for the corresponding Beta distribtion
   * @param beta The beta value for the corresponding Beta distribution
   *
   * @return mle The MLE for the parameter
   */
  double _computeMleForCell(int alpha, int beta);

  /**
   * Compute the posterior mean for a single parameter.
   * Used in a nullary expression in posteriorMean
   *
   * @param alpha The alpha value for the corresponding Beta distribtion
   * @param beta The beta value for the corresponding Beta distribution
   *
   * @return mean The posterior mean for that parameter
   */
  double _computePosteriorMeanForCell(int alpha, int beta);

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
      : _alphaEntry{_readBIMacMatrix(inDir / "alpha_entry.csv")},
        _betaEntry{_readBIMacMatrix(inDir / "beta_entry.csv")},
        _alphaExit{_readBIMacMatrix(inDir / "alpha_exit.csv")},
        _betaExit{_readBIMacMatrix(inDir / "beta_exit.csv")} {}

  /**
   * Take a posterior sample from BIMac to get a single IMac instance.
   *
   * @return imac A shared ptr to an IMac instance
   */
  std::shared_ptr<IMac> posteriorSample();

  /**
   * Compute the MLE given the observed data.
   *
   * Given how BiMac is initialised, this corresponds to computing the mode
   * of each beta distribution.
   *
   * @return imac A shared ptr to an IMac instance
   */
  std::shared_ptr<IMac> mle();

  /**
   * Compute the posterior mean given the observed data.
   *
   * As the + 1 part is already in the beta distribution, its just:
   * alpha/(alpha + beta)
   *
   * @return imac A shared ptr to an IMac instance
   */
  std::shared_ptr<IMac> posteriorMean();

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