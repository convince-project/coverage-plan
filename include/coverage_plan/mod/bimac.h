/**
 * Header file for the BIMac class.
 *
 * BiMac is an extension of IMac which explicitly maintains the distribution
 * over the IMac parameters (lambda_entry, lambda_exit, initial state
 * distribution).
 *
 * Initial state distribution learning was not included in the original IMac
 * paper.
 *
 * @author Charlie Street
 */
#ifndef BIMAC_H
#define BIMAC_H

#include "coverage_plan/mod/grid_cell.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <filesystem>
#include <functional>
#include <memory>
#include <random>

/**
 * Struct for storing BiMac observations.
 *
 * Note an observation at (x,y) corresponds to element (y,x) in the matrices.
 * This is so we map to the Cartesian coordinates the robot operates over.
 *
 * Members:
 * cell: The cell on the grid
 * freeToOccupied: Number of observations at (x,y) going from free to occupied
 * freeToFree: Number of observations at (x,y) going from free to free
 * occupiedToFree: Number of observations at (x,y) going from occupied to free
 * occupiedToOccupied: Number of observations at (x,y) going from occupied to
 * occupied
 * initFree: Number of observations of (x,y) being free at time 0
 * initOccupied: Number of observationsof (x,y) being occupied at time 0
 */
struct BIMacObservation {
  GridCell cell{};
  int freeToOccupied{};
  int freeToFree{};
  int occupiedToFree{};
  int occupiedToOccupied{};
  int initFree{};
  int initOccupied{};
};

/**
 * A class which maintains our uncertainty over the true IMac model.
 * @see mod/imac.h
 * This class maintains a Beta distribution for each transition at each grid
 * cell.
 *
 * Members:
 *  _alphaEntry: The matrix of alpha parameters for the lambda_entry
 * parameter (free->occupied)
 * _betaEntry: The matrix of beta parameters for the lambda_entry parameter
 * (free->occupied)
 * _alphaExit: The matrix of alpha parameters for the lambda_exit parameter
 * (occupied->free)
 * _betaExit: The matrix of beta parameters for the lambda_exit
 * parameter (occupied->free)
 * _alphaInit: The matrix of alpha parameters for the Pr(occupied at time 0)
 * parameter, i.e. the initial state distribution
 * _betaInit: The matrix of beta parameters for the Pr(occupied at time 0)
 * parameter, i.e. the initial state distribution
 */
class BIMac {
private:
  Eigen::MatrixXi _alphaEntry{};
  Eigen::MatrixXi _betaEntry{};
  Eigen::MatrixXi _alphaExit{};
  Eigen::MatrixXi _betaExit{};
  Eigen::MatrixXi _alphaInit{};
  Eigen::MatrixXi _betaInit{};

  /**
   * Reads BIMac matrix in from file.
   *
   * @param inFile The Matrix file to read in
   * @return mat The matrix read from file
   */
  Eigen::MatrixXi _readBIMacMatrix(const std::filesystem::path &inFile);

  /**
   * Write a single BIMac matrix to file.
   *
   * @param matrix The matrix to write out
   * @param outFile The file to send the BIMac matrix
   */
  void _writeBIMacMatrix(const Eigen::MatrixXi &matrix,
                         const std::filesystem::path &outFile);

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
  double _sampleForCell(int alpha, int beta, std::mt19937_64 &gen,
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

  /**
   * Generates a single IMac matrix given a function to generate each value.
   *
   * @param alphaMat The matrix of alpha values (from Beta distributions)
   * @param betaMat The matrix of beta values (from Beta distributions)
   * @param getSingleVal a function which takes an alpha and beta and returns
   * the parameter value
   *
   * @return iMacMatrix The matrix of parameters for IMac
   */
  Eigen::MatrixXd
  _createIMacMatrix(const Eigen::MatrixXi &alphaMat,
                    const Eigen::MatrixXi &betaMat,
                    const std::function<double(int, int)> &getSingleVal);

public:
  /**
   * This constructor initialises all matrices to be full of ones.
   *
   * @param x The length of the x dimension of the grid map (num cols)
   * @param y The length of the y dimension of the grid map (num rows)
   */
  BIMac(int x, int y)
      : _alphaEntry{Eigen::MatrixXi::Ones(y, x)},
        _betaEntry{Eigen::MatrixXi::Ones(y, x)},
        _alphaExit{Eigen::MatrixXi::Ones(y, x)},
        _betaExit{Eigen::MatrixXi::Ones(y, x)},
        _alphaInit{Eigen::MatrixXi::Ones(y, x)}, _betaInit{
                                                     Eigen::MatrixXi::Ones(
                                                         y, x)} {}

  /**
   * This constructor reads a BIMac config in from file.
   *
   * @param inDir The directory where the IMac files are stored
   */
  BIMac(const std::filesystem::path &inDir)
      : _alphaEntry{_readBIMacMatrix(inDir / "alpha_entry.csv")},
        _betaEntry{_readBIMacMatrix(inDir / "beta_entry.csv")},
        _alphaExit{_readBIMacMatrix(inDir / "alpha_exit.csv")},
        _betaExit{_readBIMacMatrix(inDir / "beta_exit.csv")},
        _alphaInit{_readBIMacMatrix(inDir / "alpha_init.csv")},
        _betaInit{_readBIMacMatrix(inDir / "beta_init.csv")} {}

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
  void updatePosterior(const std::vector<BIMacObservation> &observations);

  /**
   * Write BiMac matrices out to file.
   *
   * @param outDir The directory to write the BIMac matrices to
   */
  void writeBIMac(const std::filesystem::path &outDir);
};
#endif