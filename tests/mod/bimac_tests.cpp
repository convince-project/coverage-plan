/**
 * Tests for the BIMac class.
 *
 * @author Charlie Street
 */

#include "coverage_plan/mod/bimac.h"
#include "coverage_plan/mod/imac.h"
#include <Eigen/Dense>
#include <catch2/catch.hpp>
#include <filesystem>
#include <memory>

TEST_CASE("Tests for initialisation BIMac constructor", "[BIMac]") {

  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};

  std::shared_ptr<IMac> imac{bimac->posteriorMean()};

  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  Eigen::MatrixXd initBelief{imac->estimateStaticOccupancy()};
  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  // Add some observations and try this again
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 0, 2, 2, 0, 0, 2});
  obsVec.push_back(BIMacObservation{2, 0, 0, 2, 2, 0, 2, 0});
  bimac->updatePosterior(obsVec);

  imac = bimac->posteriorMean();
  staticOcc = imac->estimateStaticOccupancy();
  initBelief = imac->getInitialBelief();

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.75, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));
}

TEST_CASE("Tests for reading and writing BIMac objects", "[readWrite]") {
  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 0, 2, 2, 0, 0, 2});
  obsVec.push_back(BIMacObservation{1, 0, 1, 3, 1, 4, 2, 1});
  obsVec.push_back(BIMacObservation{2, 0, 10, 25, 17, 18, 6, 0});
  obsVec.push_back(BIMacObservation{0, 1, 20, 4, 7, 9, 2, 6});
  obsVec.push_back(BIMacObservation{1, 1, 1, 1, 1, 1, 5, 13});
  obsVec.push_back(BIMacObservation{2, 1, 5, 6, 7, 8, 4, 4});
  bimac->updatePosterior(obsVec);

  std::filesystem::path matDir{"/tmp"};

  std::shared_ptr<IMac> imac{bimac->posteriorMean()};
  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  Eigen::MatrixXd initBelief{imac->getInitialBelief()};

  bimac->writeBIMac(matDir);

  std::unique_ptr<BIMac> twoBiTwoMac{std::make_unique<BIMac>(matDir)};
  std::shared_ptr<IMac> imacTwo{twoBiTwoMac->posteriorMean()};
  Eigen::MatrixXd staticOccTwo{imacTwo->estimateStaticOccupancy()};
  Eigen::MatrixXd initBeliefTwo{imacTwo->getInitialBelief()};

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE(staticOccTwo.rows() == 2);
  REQUIRE(staticOccTwo.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0),
               Catch::Matchers::WithinRel(staticOccTwo(0, 0), 0.001));
  REQUIRE_THAT(staticOcc(0, 1),
               Catch::Matchers::WithinRel(staticOccTwo(0, 1), 0.001));
  REQUIRE_THAT(staticOcc(0, 2),
               Catch::Matchers::WithinRel(staticOccTwo(0, 2), 0.001));
  REQUIRE_THAT(staticOcc(1, 0),
               Catch::Matchers::WithinRel(staticOccTwo(1, 0), 0.001));
  REQUIRE_THAT(staticOcc(1, 1),
               Catch::Matchers::WithinRel(staticOccTwo(1, 1), 0.001));
  REQUIRE_THAT(staticOcc(1, 2),
               Catch::Matchers::WithinRel(staticOccTwo(1, 2), 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE(initBeliefTwo.rows() == 2);
  REQUIRE(initBeliefTwo.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0),
               Catch::Matchers::WithinRel(initBeliefTwo(0, 0), 0.001));
  REQUIRE_THAT(initBelief(0, 1),
               Catch::Matchers::WithinRel(initBeliefTwo(0, 1), 0.001));
  REQUIRE_THAT(initBelief(0, 2),
               Catch::Matchers::WithinRel(initBeliefTwo(0, 2), 0.001));
  REQUIRE_THAT(initBelief(1, 0),
               Catch::Matchers::WithinRel(initBeliefTwo(1, 0), 0.001));
  REQUIRE_THAT(initBelief(1, 1),
               Catch::Matchers::WithinRel(initBeliefTwo(1, 1), 0.001));
  REQUIRE_THAT(initBelief(1, 2),
               Catch::Matchers::WithinRel(initBeliefTwo(1, 2), 0.001));
}

TEST_CASE("Tests for posterior update", "[updatePosterior]") {

  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};

  std::shared_ptr<IMac> imac{bimac->posteriorMean()};

  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  Eigen::MatrixXd initBelief{imac->getInitialBelief()};

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  // Add some observations
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 0, 2, 2, 0, 0, 2});
  obsVec.push_back(BIMacObservation{1, 0, 0, 3, 0, 3, 2, 1});
  obsVec.push_back(BIMacObservation{2, 0, 0, 2, 2, 0, 6, 0});
  obsVec.push_back(BIMacObservation{0, 1, 1, 2, 2, 1, 2, 6});
  obsVec.push_back(BIMacObservation{1, 1, 3, 3, 4, 2, 5, 13});
  obsVec.push_back(BIMacObservation{2, 1, 5, 3, 1, 7, 4, 4});
  bimac->updatePosterior(obsVec);

  imac = bimac->posteriorMean();
  staticOcc = imac->estimateStaticOccupancy();
  initBelief = imac->getInitialBelief();

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.4375, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.7, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.75, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.125, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));
}

TEST_CASE("Tests for posterior mean", "[posteriorMean]") {
  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};

  // Add some observations
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 0, 2, 2, 0, 0, 2});
  obsVec.push_back(BIMacObservation{1, 0, 0, 3, 0, 3, 2, 1});
  obsVec.push_back(BIMacObservation{2, 0, 4, 14, 3, 5, 6, 0});
  obsVec.push_back(BIMacObservation{0, 1, 1, 2, 2, 1, 2, 6});
  obsVec.push_back(BIMacObservation{1, 1, 3, 3, 4, 2, 5, 13});
  obsVec.push_back(BIMacObservation{2, 1, 5, 3, 1, 7, 4, 4});
  bimac->updatePosterior(obsVec);

  // Can only evaluate by means of computing the initial belief
  std::shared_ptr<IMac> imac{bimac->posteriorMean()};
  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  Eigen::MatrixXd initBelief{imac->getInitialBelief()};

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.425, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.4375, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.7, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.75, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.125, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));
}

TEST_CASE("Tests for mle", "[mle]") {

  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};

  std::shared_ptr<IMac> imac{bimac->mle()};

  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  Eigen::MatrixXd initBelief{imac->getInitialBelief()};

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));

  // Add some observations and try this again
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 1, 3, 3, 1, 1, 3});
  obsVec.push_back(BIMacObservation{1, 0, 1, 4, 1, 4, 3, 2});
  obsVec.push_back(BIMacObservation{2, 0, 5, 15, 4, 6, 7, 1});
  obsVec.push_back(BIMacObservation{0, 1, 2, 3, 3, 2, 3, 7});
  obsVec.push_back(BIMacObservation{1, 1, 4, 4, 5, 3, 6, 14});
  obsVec.push_back(BIMacObservation{2, 1, 6, 4, 2, 8, 5, 5});
  bimac->updatePosterior(obsVec);

  // Can only evaluate by means of computing the initial belief
  imac = bimac->mle();
  staticOcc = imac->estimateStaticOccupancy();
  initBelief = imac->getInitialBelief();

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE_THAT(staticOcc(0, 0), Catch::Matchers::WithinRel(0.25, 0.001));
  REQUIRE_THAT(staticOcc(0, 1), Catch::Matchers::WithinRel(0.5, 0.001));
  REQUIRE_THAT(staticOcc(0, 2), Catch::Matchers::WithinRel(0.425, 0.001));
  REQUIRE_THAT(staticOcc(1, 0), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(staticOcc(1, 1), Catch::Matchers::WithinRel(0.4375, 0.001));
  REQUIRE_THAT(staticOcc(1, 2), Catch::Matchers::WithinRel(0.7, 0.001));

  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);
  REQUIRE_THAT(initBelief(0, 0), Catch::Matchers::WithinRel(0.75, 0.001));
  REQUIRE_THAT(initBelief(0, 1), Catch::Matchers::WithinRel(0.4, 0.001));
  REQUIRE_THAT(initBelief(0, 2), Catch::Matchers::WithinRel(0.125, 0.001));
  REQUIRE_THAT(initBelief(1, 0), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 1), Catch::Matchers::WithinRel(0.7, 0.001));
  REQUIRE_THAT(initBelief(1, 2), Catch::Matchers::WithinRel(0.5, 0.001));
}

TEST_CASE("Tests for posterior sample", "[posteriorSample]") {
  std::unique_ptr<BIMac> bimac{std::make_unique<BIMac>(3, 2)};

  std::shared_ptr<IMac> imac{bimac->posteriorSample()};

  Eigen::MatrixXd staticOcc{imac->estimateStaticOccupancy()};
  Eigen::MatrixXd initBelief{imac->getInitialBelief()};

  REQUIRE(staticOcc.rows() == 2);
  REQUIRE(staticOcc.cols() == 3);
  REQUIRE(initBelief.rows() == 2);
  REQUIRE(initBelief.cols() == 3);

  // Check that we've got different sample values at each cell
  for (int i{0}; i < 2; ++i) {
    for (int j{0}; j < 3; ++j) {
      for (int k{0}; k < 2; ++k) {
        for (int l{0}; l < 3; ++l) {
          if (!((i == k) && (j == l))) {
            REQUIRE_THAT(initBelief(i, j),
                         !Catch::Matchers::WithinRel(initBelief(k, l), 0.0001));
            REQUIRE_THAT(staticOcc(i, j),
                         !Catch::Matchers::WithinRel(staticOcc(k, l), 0.0001));
          }
        }
      }
    }
  }

  // Make it nearly deterministic and check initial belief matches that
  std::vector<BIMacObservation> obsVec{};
  obsVec.push_back(BIMacObservation{0, 0, 10000, 0, 0, 10000, 0, 10000});
  obsVec.push_back(BIMacObservation{1, 0, 10000, 0, 0, 10000, 0, 10000});
  obsVec.push_back(BIMacObservation{2, 0, 10000, 0, 0, 10000, 0, 10000});
  obsVec.push_back(BIMacObservation{0, 1, 10000, 0, 0, 10000, 0, 10000});
  obsVec.push_back(BIMacObservation{1, 1, 10000, 0, 0, 10000, 0, 10000});
  obsVec.push_back(BIMacObservation{2, 1, 10000, 0, 0, 10000, 0, 10000});
  bimac->updatePosterior(obsVec);

  imac = bimac->posteriorSample();
  staticOcc = imac->estimateStaticOccupancy();
  initBelief = imac->getInitialBelief();

  for (int i{0}; i < 2; ++i) {
    for (int j{0}; j < 3; ++j) {
      REQUIRE(staticOcc(i, j) > 0.99);
      REQUIRE(initBelief(i, j) > 0.99);
    }
  }
}