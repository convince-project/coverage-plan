/**
 * Implementation of the IMac class in imac.h.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <memory>

/**
 * Reads IMac matrix in from file.
 */
Eigen::MatrixXd _readIMacMatrix(const std::filesystem::path &inFile) {
  std::vector<double> matrixElems;

  std::ifstream f(inFile);

  // storage for each row of the matrix and each element
  std::string rowString;
  std::string entryString;

  int numRows{0};

  while (getline(f, rowString)) {
    // iterate through each row using string streams
    std::stringstream rowStream(rowString);
    while (getline(rowStream, entryString, ',')) {

      // Get each element of the matrix
      matrixElems.push_back(std::stod(entryString));
    }
    ++numRows;
  }

  // Convert the vector of elements back into an Eigen matrix
  // Kept in column major to ensure consistency and avoid bugs
  int numCols{(int)matrixElems.size() / numRows};
  Eigen::MatrixXd matrix{Eigen::MatrixXd::Zero(numRows, numCols)};

  for (int i{0}; i < numRows; ++i) {
    for (int j{0}; j < numCols; ++j) {

      // numCols is the size of each row
      matrix(i, j) = matrixElems.at(i * numCols + j);
    }
  }

  return matrix;
}

/**
 * Write a single IMac matrix to file.
 */
void _writeIMacMatrix(const Eigen::MatrixXd &matrix,
                      const std::filesystem::path &outFile) {
  // output matrix format
  const static Eigen::IOFormat csvFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", "\n");
  std::ofstream f(outFile);
  if (f.is_open()) {
    f << matrix.format(csvFormat);
    f.close();
  }
}

/**
 * Returns estimate of static occupancy from IMac map of dynamics.
 */
Eigen::MatrixXd IMac::estimateStaticOccupancy() {
  // Check if already cached - if it isn't, the matrix will be empty
  if (this->_staticOccupancy.size() == 0) {
    Eigen::MatrixXd ones{Eigen::MatrixXd::Ones(this->_entryMatrix.rows(),
                                               this->_entryMatrix.cols())};
    this->_staticOccupancy =
        (0.5 * this->_entryMatrix) + (0.5 * (ones - this->_exitMatrix));
  }
  return this->_staticOccupancy;
}

/**
 * Runs a given belief or state through IMac to get distribution for the next
 * timestep
 */
Eigen::MatrixXd IMac::forwardStep(const Eigen::MatrixXd &currentBelief) const {
  Eigen::MatrixXd ones{Eigen::MatrixXd::Ones(this->_entryMatrix.rows(),
                                             this->_entryMatrix.cols())};
  return (ones - currentBelief).cwiseProduct(this->_entryMatrix) +
         currentBelief.cwiseProduct((ones - this->_exitMatrix));
}

/**
 * Write IMac matrices out to file.
 */
void IMac::writeIMac(const std::filesystem::path &outDir) {
  // Each matrix is going to a separate file
  this->_writeIMacMatrix(this->_entryMatrix, outDir / "entry.csv");
  this->_writeIMacMatrix(this->_exitMatrix, outDir / "exit.csv");
  this->_writeIMacMatrix(this->_initialBelief, outDir / "initial_belief.csv");
}