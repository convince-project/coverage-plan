/**
 * Implementation of the BIMac class in bimac.h.
 * @see bimac.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

/**
 * Read BIMac matrix in from file.
 * Credit to Aleksandar Haber's tutorial for this function
 * https://www.youtube.com/watch?v=K9QB1LbemnY&ab_channel=AleksandarHaber
 */
Eigen::MatrixXi _readBIMacMatrix(std::filesystem::path inFile) {

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
  int numCols{matrixElems.size() / numRows};
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
void BIMac::_writeBIMacMatrix(Eigen::MatrixXi matrix,
                              std::filesystem::path outFile) {

  // output format for matrix
  const static Eigen::IOFormat csvFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", "\n");
  std::ofstream f(outFile);
  if (f.is_open()) {
    f << matrix.format(csvFormat);
    f.close();
  }
}

// double
// BIMac::_sampleForCell(int alpha, int beta, const std::mt19937 &gen,
//                       const std::uniform_real_distribution<double> &sampler)
//                       {
//   return sampler(gen);
// }

/**
 * Compute the MLE value for a single parameter.
 */
double BIMac::_computeMleForCell(int alpha, int beta) {
  if (alpha == 1 && beta == 1) {
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

// TODO: sample
std::shared_ptr<IMac> BIMac::sample() {
  // Generate a uniform rng between 0 and 1 to sample IMac parameters
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::uniform_real_distribution<double> sampler{0.0, 1.0};

  // Compute sampled entry matrix
  Eigen::MatrixXd entryMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaEntry.rows(), this->_alphaEntry.cols(), [&](Eigen::Index i) {
        return this->_sampleForCell(this->_alphaEntry(i), this->_betaEntry(i),
                                    gen, sampler);
      })};

  // Compute sampled exit matrix
  Eigen::MatrixXd exitMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaExit.rows(), this->_alphaExit.cols(), [&](Eigen::Index i) {
        return this->_sampleForCell(this->_alphaExit(i), this->_betaExit(i),
                                    gen, sampler);
      })};

  return std::make_shared<IMac>(entryMatrix, exitMatrix);
}

/**
 * Compute the MLE given the observed data.
 *
 * This amounts to computing the mode of each Beta distribution.
 * mode = (alpha - 1) / (alpha + beta - 2)
 */
std::shared_ptr<IMac> BIMac::mle() {
  // Compute MLE for entry matrix
  Eigen::MatrixXd entryMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaEntry.rows(), this->_alphaEntry.cols(), [&](Eigen::Index i) {
        return this->_computeMleForCell(this->_alphaEntry(i),
                                        this->_betaEntry(i));
      })};

  // Compute MLE for exit matrix
  Eigen::MatrixXd exitMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaExit.rows(), this->_alphaExit.cols(), [&](Eigen::Index i) {
        return this->_computeMleForCell(this->_alphaExit(i),
                                        this->_betaExit(i));
      })};

  return std::make_shared<IMac>(entryMatrix, exitMatrix);
}

/**
 * Compute the posterior mean given the observed data.
 *
 * The mean of a beta distribution is alpha/(alpha + beta)
 */
std::shared_ptr<IMac> BIMac::posteriorMean() {
  // Compute posterior mean for entry matrix
  Eigen::MatrixXd entryMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaEntry.rows(), this->_alphaEntry.cols(), [&](Eigen::Index i) {
        return this->_computePosteriorMeanForCell(this->_alphaEntry(i),
                                                  this->_betaEntry(i));
      })};

  // Compute posterior mean for exit matrix
  Eigen::MatrixXd exitMatrix{Eigen::MatrixXd::NullaryExpr(
      this->_alphaExit.rows(), this->_alphaExit.cols(), [&](Eigen::Index i) {
        return this->_computePosteriorMeanForCell(this->_alphaExit(i),
                                                  this->_betaExit(i));
      })};

  return std::make_shared<IMac>(entryMatrix, exitMatrix);
}

/**
 * Updates the BIMac posterior given a new set of observations.
 */
void BIMac::updatePosterior(std::vector<BIMacObservation> observations) {

  for (BIMacObservation obs : observations) {
    // Update lambda_entry parameters
    this->_alphaEntry(obs.x, obs.y) += obs.freeToOccupied;
    this->_betaEntry(obs.x, obs.y) += obs.freeToFree;

    // Update lambda exit parameters
    this->_alphaExit(obs.x, obs.y) += obs.occupiedToFree;
    this->_betaExit(obs.x, obs.y) += obs.occupiedToOccupied;
  }
}

/**
 * Writes each of the BIMac matrices out to file
 */
void BIMac::writeBIMac(std::filesystem::path outDir) {
  // Each matrix is stored in a different file
  _writeBIMacMatrix(this->_alphaEntry, outDir / "alpha_entry.csv");
  _writeBIMacMatrix(this->_betaEntry, outDir / "beta_entry.csv");
  _writeBIMacMatrix(this->_alphaExit, outDir / "alpha_exit.csv");
  _writeBIMacMatrix(this->_betaExit, outDir / "beta_exit.csv");
}