/**
 * Implementation of the CoverageBelief class in  coverage_belief.h.
 * @see coverage_belief.h
 *
 * @author Charlie Street
 */

#include "coverage_plan/planning/coverage_belief.h"
#include "coverage_plan/mod/imac_belief_sampler.h"
#include "coverage_plan/mod/imac_executor.h"
#include "coverage_plan/planning/action.h"
#include "coverage_plan/planning/coverage_observation.h"
#include "coverage_plan/planning/coverage_state.h"
#include <despot/interface/belief.h>
#include <despot/interface/pomdp.h>
#include <iomanip>
#include <sstream>
#include <string>
#include <tuple>

/**
 * Sample a number of states from the IMac model.
 */
std::vector<despot::State *> CoverageBelief::Sample(int num) const {
  // Note: For now I'm assuming unweighted particles. Lets see how this goes
  double weight{1.0 / (double)num};

  std::vector<despot::State *> particles{};

  for (int i{0}; i < num; ++i) {

    // Create a sampled state
    CoverageState *particle{
        static_cast<CoverageState *>(this->model_->Allocate(-1, weight))};

    // Set the fully observable components
    particle->robotPosition = this->_robotPosition;
    particle->time = this->_time;
    particle->covered = this->_covered;

    // Sample a map state from the current belief
    particle->map = this->_beliefSampler->sampleFromBelief(this->_mapBelief);

    particles.push_back(particle);
  }

  return particles;
}

/**
 * Update the belief.
 */
void CoverageBelief::Update(despot::ACT_TYPE action, despot::OBS_TYPE obs) {

  // I'm not using the history, but store for completeness
  history_.Add(action, obs);

  std::pair<std::vector<IMacObservation>, bool> obsInfo{
      Observation::fromObsType(obs, this->_fov)};

  // Update robot location if action successful (action success is observable)
  if (std::get<1>(obsInfo)) {
    this->_robotPosition = ActionHelpers::applySuccessfulAction(
        this->_robotPosition, ActionHelpers::fromInt(action));
  }

  // Update time (time is observable)
  ++this->_time;

  // Update covered (location is observable)
  this->_covered.push_back(this->_robotPosition);

  // Update map belief (forward step of IMac model and setting known locations)
  this->_mapBelief = this->_imac->forwardStep(this->_mapBelief);
  // Set robot position as being free of obstacles
  this->_mapBelief(this->_robotPosition.y, this->_robotPosition.x) = 0;
  for (const IMacObservation &imacObs : std::get<0>(obsInfo)) {
    GridCell obsLoc{this->_robotPosition.x + imacObs.cell.x,
                    this->_robotPosition.y + imacObs.cell.y};
    if (!obsLoc.outOfBounds(0, this->_mapBelief.cols(), 0,
                            this->_mapBelief.rows())) {
      this->_mapBelief(obsLoc.y, obsLoc.x) = imacObs.occupied;
    }
  }
}

/**
 * Convert belief into a string.
 */
std::string CoverageBelief::text() const {
  std::ostringstream stream{};

  int pctCovered{int(round(100 * (double)this->_covered.size() /
                           (double)this->_mapBelief.size()))};

  // Write out pos, time, percentage covered
  stream << std::setprecision(2) << "Robot Position: ("
         << this->_robotPosition.x << ", " << this->_robotPosition.y
         << "); Time: " << this->_time << "; Covered: " << pctCovered
         << "%; Map Belief: \n";

  // Write out the map belief to a string
  for (int x{0}; x < this->_mapBelief.cols(); ++x) {
    for (int y{0}; y < this->_mapBelief.rows(); ++y) {
      stream << this->_mapBelief(y, x) << " ";
    }
    stream << "\n";
  }

  return stream.str();
}

/**
 * Make a copy of the belief.
 */
despot::Belief *CoverageBelief::MakeCopy() const {
  // Note: Allocated with new, so make sure this is deallocated...
  return new CoverageBelief(this->model_, this->_robotPosition, this->_time,
                            this->_covered, this->_mapBelief, this->_imac,
                            this->_fov);
}