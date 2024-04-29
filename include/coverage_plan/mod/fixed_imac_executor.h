/**
 * @file fixed_imac_executor.h
 *
 * @brief Header file for the FixedIMacExecutor class.
 *
 * FixedIMacExecutor allows us to run pre-defined MoD traces again and again.
 * This is useful for keeping things consistent in experiments.
 *
 * @author Charlie Street
 */
#ifndef FIXED_IMAC_EXECUTOR_H
#define FIXED_IMAC_EXECUTOR_H

#include "coverage_plan/mod/imac_executor.h"
#include <Eigen/Dense>
#include <filesystem>
#include <vector>

/**
 * Class allows to execute IMac traces read in from a file.
 * The idea is that we read in a number of episodes, and once we run restart,
 * we switch to the next episode.
 *
 * Members:
 * As in superclass, plus:
 * * _files: A vector of files
 * * _episode: The episode number
 * * _ts: The current timestep
 * * _currentEpisode: The vector of matrices for the current episode
 * * _xDim: Size of X dimension of the map
 * * _yDim: Size of Y dimension of the map
 */
class FixedIMacExecutor : public IMacExecutor {

private:
  const std::vector<std::filesystem::path> _files{};
  int _episode{};
  int _ts{};
  std::vector<Eigen::MatrixXi> _currentEpisode{};
  const int _xDim{};
  const int _yDim{};

  /**
   * Reads in data from file for new episode to get IMac trace.
   */
  void _setCurrentEpisode();

public:
  /**
   * Constructor initialises members.
   *
   * @param files A vector of files with the IMac traces
   * @param xDim: Size of X dimension of the map
   * @param yDim: Size of Y dimension of the map
   *
   */
  FixedIMacExecutor(const std::vector<std::filesystem::path> &files,
                    const int &xDim, const int &yDim)
      : IMacExecutor(nullptr), _files{files}, _episode{-1}, _ts{0},
        _currentEpisode{std::vector<Eigen::MatrixXi>{}}, _xDim{xDim},
        _yDim{yDim} {}

  /**
   * Restart the simulation and start running the next episode
   *
   * @param observations Not used in this class.
   *
   * @returns the initial state of the new episode
   */
  Eigen::MatrixXi restart(const std::vector<IMacObservation> &observations =
                              std::vector<IMacObservation>{});

  /**
   * Move onto the next IMac state in the episode
   *
   * @param observations Notused in this class
   *
   * @returns The successor state in the episode
   *
   * @throw outOfEpisode Thrown if update occurs beyond end of episode
   */
  Eigen::MatrixXi updateState(const std::vector<IMacObservation> &observations);
};

#endif