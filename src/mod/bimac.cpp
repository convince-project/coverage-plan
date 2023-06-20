/**
 * Implementation of the BIMac class in bimac.h.
 * @see bimac.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <boost/math/special_functions/beta.hpp>
#include <filesystem>
#include <fstream>
#include <memory>

/**
 * Read BIMac matrix in from file.
 * Credit to Aleksandar Haber's tutorial for this function
 * https://www.youtube.com/watch?v=K9QB1LbemnY&ab_channel=AleksandarHaber
 */
Eigen::MatrixXi BIMac::_readBIMacMatrix(const std::filesystem::path &inFile) {

  // Vector to temporarily store matrix entries
  std::vector<int> matrixElems;

  std::ifstream f(inFile);

  // storage for each row of the matrix and each element
  std::string rowString;
  std::string entryString;

  int numRows{0};

  while (getline(f, rowString)) {
    // iterate through each row using string streams
    std::stringstream rowStream(rowString);
    while (getline(rowStream, entryString, ',')) {

      // Now we get each single element of the matrix
      matrixElems.push_back(std::stoi(entryString));
    }
    ++numRows;
  }

  // Convert the vector of elements back into an Eigen matrix
  // I want to keep this in column major to ensure consistency and avoid bugs
  // Function won't be called frequently, so doesn't need to be efficient
  int numCols{(int)matrixElems.size() / numRows};
  Eigen::MatrixXi matrix{Eigen::MatrixXi::Ones(numRows, numCols)};

  for (int i{0}; i < numRows; ++i) {
    for (int j{0}; j < numCols; ++j) {

      // numCols is the size of each row
      matrix(i, j) = matrixElems.at(i * numCols + j);
    }
  }

  return matrix;
}

/**
 * Write a single BIMac matrix to file.
 * Credit to Aleksandar Haber's tutorial for this function
 * https://www.youtube.com/watch?v=K9QB1LbemnY&ab_channel=AleksandarHaber
 */
void BIMac::_writeBIMacMatrix(const Eigen::MatrixXi &matrix,
                              const std::filesystem::path &outFile) {

  // output format for matrix
  const static Eigen::IOFormat csvFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", "\n");
  std::ofstream f(outFile);
  if (f.is_open()) {
    f << matrix.format(csvFormat);
    f.close();
  }
}

double BIMac::_sampleForCell(int alpha, int beta, std::mt19937 &gen,
                             std::uniform_real_distribution<double> &sampler) {
  return boost::math::ibeta_inv(alpha, beta, sampler(gen));
}

/**
 * Compute the MLE value for a single parameter.
 */
double BIMac::_computeMleForCell(int alpha, int beta) {
  if (alpha == 1 && beta == 1) {
    // design choice to just return 0.5 for a uniform distribution
    return 0.5;
  } else {
    return (alpha - 1.0) / (alpha + beta - 2.0);
  }
}

/**
 * Compute the posterior mean for a single parameter.
 */
double BIMac::_computePosteriorMeanForCell(int alpha, int beta) {
  return ((double)alpha) / (alpha + beta);
}

/**
 * Generates a single IMac matrix given a function to generate each value.
 */
Eigen::MatrixXd
BIMac::_createIMacMatrix(const Eigen::MatrixXi &alphaMat,
                         const Eigen::MatrixXi &betaMat,
                         const std::function<double(int, int)> &getSingleVal) {
  return Eigen::MatrixXd::NullaryExpr(
      alphaMat.rows(), alphaMat.cols(),
      [&](Eigen::Index i) { return getSingleVal(alphaMat(i), betaMat(i)); });
}

/**
 * Take a posterior sample from BIMac to get a single IMac instance
 */
std::shared_ptr<IMac> BIMac::posteriorSample() {
  // Generate a uniform rng between 0 and 1 to sample IMac parameters
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> sampler{0.0, 1.0};

  auto psLambda{[&](int alpha, int beta) {
    return this->_sampleForCell(alpha, beta, gen, sampler);
  }};
  return std::make_shared<IMac>(
      this->_createIMacMatrix(this->_alphaEntry, this->_betaEntry, psLambda),
      this->_createIMacMatrix(this->_alphaExit, this->_betaExit, psLambda),
      this->_createIMacMatrix(this->_alphaInit, this->_betaInit, psLambda));
}

/**
 * Compute the MLE given the observed data.
 *
 * This amounts to computing the mode of each Beta distribution.
 * mode = (alpha - 1) / (alpha + beta - 2)
 */
std::shared_ptr<IMac> BIMac::mle() {
  auto mleLambda{[&](int alpha, int beta) {
    return this->_computeMleForCell(alpha, beta);
  }};
  return std::make_shared<IMac>(
      this->_createIMacMatrix(this->_alphaEntry, this->_betaEntry, mleLambda),
      this->_createIMacMatrix(this->_alphaExit, this->_betaExit, mleLambda),
      this->_createIMacMatrix(this->_alphaInit, this->_betaInit, mleLambda));
}

/**
 * Compute the posterior mean given the observed data.
 *
 * The mean of a beta distribution is alpha/(alpha + beta)
 */
std::shared_ptr<IMac> BIMac::posteriorMean() {
  auto pmLambda{[&](int alpha, int beta) {
    return this->_computePosteriorMeanForCell(alpha, beta);
  }};
  return std::make_shared<IMac>(
      this->_createIMacMatrix(this->_alphaEntry, this->_betaEntry, pmLambda),
      this->_createIMacMatrix(this->_alphaExit, this->_betaExit, pmLambda),
      this->_createIMacMatrix(this->_alphaInit, this->_betaInit, pmLambda));
}

/**
 * Updates the BIMac posterior given a new set of observations.
 */
void BIMac::updatePosterior(const std::vector<BIMacObservation> &observations) {

  for (BIMacObservation obs : observations) {
    // Update lambda_entry parameters
    // Recall that rows correspond to the y direction, columns to x
    this->_alphaEntry(obs.cell.y, obs.cell.x) += obs.freeToOccupied;
    this->_betaEntry(obs.cell.y, obs.cell.x) += obs.freeToFree;

    // Update lambda exit parameters
    this->_alphaExit(obs.cell.y, obs.cell.x) += obs.occupiedToFree;
    this->_betaExit(obs.cell.y, obs.cell.x) += obs.occupiedToOccupied;

    // Update initial state distribution parameters
    // Distribution is over the initial occupation probability, so alpha
    // gets the initOccupied observations
    this->_alphaInit(obs.cell.y, obs.cell.x) += obs.initOccupied;
    this->_betaInit(obs.cell.y, obs.cell.x) += obs.initFree;
  }
}

/**
 * Writes each of the BIMac matrices out to file
 */
void BIMac::writeBIMac(const std::filesystem::path &outDir) {
  // Each matrix is stored in a different file
  _writeBIMacMatrix(this->_alphaEntry, outDir / "alpha_entry.csv");
  _writeBIMacMatrix(this->_betaEntry, outDir / "beta_entry.csv");
  _writeBIMacMatrix(this->_alphaExit, outDir / "alpha_exit.csv");
  _writeBIMacMatrix(this->_betaExit, outDir / "beta_exit.csv");
  _writeBIMacMatrix(this->_alphaInit, outDir / "alpha_init.csv");
  _writeBIMacMatrix(this->_betaInit, outDir / "beta_init.csv");
}