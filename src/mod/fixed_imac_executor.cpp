/**
 * Implementation of the FixedIMacExecutor class in fixed_imac_executor.h.
 * @see fixed_imac_executor.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/fixed_imac_executor.h"
#include "coverage_plan/mod/imac_executor.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

/**
 * Reads in data from file for new episode to get IMac trace.
 */
void FixedIMacExecutor::_setCurrentEpisode() {
  this->_currentEpisode.clear();
  std::ifstream f{this->_files.at(this->_episode)};
  if (f.is_open()) {
    // Placeholder variables for file reading
    std::string row{};
    std::string entry{};
    std::vector<int> matAtTs{};

    while (getline(f, row)) {
      std::stringstream rowStream{row};
      while (getline(rowStream, entry, ',')) {
        // Get each number in the file
        matAtTs.push_back(std::stoi(entry));
      }

      // Now reconstruct the matrix, assuming format output in logMapDynamics
      Eigen::MatrixXi currentMat{this->_yDim, this->_xDim};
      for (int i{1}; i < matAtTs.size(); i += 3) {
        currentMat(matAtTs.at(i + 1), matAtTs.at(i)) = matAtTs.at(i + 2);
      }
      this->_currentEpisode.push_back(currentMat);
      matAtTs.clear();
    }
  }
}

/**
 * Restart the simulation and start running the next episode.
 */
Eigen::MatrixXi
FixedIMacExecutor::restart(const std::vector<IMacObservation> &observations) {
  this->_mapDynamics.clear(); // Clear as new run
  ++this->_episode;
  this->_ts = 0;

  if (this->_episode >= this->_files.size()) {
    std::cerr << "WARNING: Looping around FixedIMacExecutor.\n";
    this->_episode = 0;
  }

  // Get new matrices
  this->_setCurrentEpisode();

  this->_currentState = this->_currentEpisode.at(this->_ts);

  this->_addMapForTs();
  return this->_currentState;
}

/**
 * Move onto the next IMac state in the episode
 */
Eigen::MatrixXi FixedIMacExecutor::updateState(
    const std::vector<IMacObservation> &observations) {

  // Update timestep and check if we've reached the end
  ++this->_ts;

  if (this->_ts >= this->_currentEpisode.size()) {
    std::cerr
        << "ERROR: Trying to update FixedIMacExecutor after end of episode.\n";
    throw "ERROR: Trying to update FixedIMacExecutor after end of episode.\n";
  }

  // Get next state from episode vector
  this->_currentState = this->_currentEpisode.at(this->_ts);

  this->_addMapForTs();
  return this->_currentState;
}