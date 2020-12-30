/**
 *  @file  RotationKinematicFactor.cpp
 *  @author Tim Setterfield
 **/

#include "mit-slam/RotationKinematicFactor.h"

#include <iostream>

namespace gtsam {

using namespace std;

/* ************************************************************************* */
RotationKinematicFactor::RotationKinematicFactor(Key PWtoBi, Key PGitoBi,
                                                 Key PWtoBj, Key PGjtoBj,
                                                 Key tGtoBt_G,
                                                 const SharedNoiseModel& model)
    : Base(model, PWtoBi, PGitoBi, PWtoBj, PGjtoBj, tGtoBt_G) {}

/* ************************************************************************* */
Vector RotationKinematicFactor::evaluateError(
    const Pose3& PWtoBi, const Pose3& PGitoBi, const Pose3& PWtoBj,
    const Pose3& PGjtoBj, const Point3& tGtoBt_G, boost::optional<Matrix&> H1,
    boost::optional<Matrix&> H2, boost::optional<Matrix&> H3,
    boost::optional<Matrix&> H4, boost::optional<Matrix&> H5) const {
  /** Rotations */
  Rot3 RBitoW = PWtoBi.rotation();
  Rot3 RGitoBi = PGitoBi.rotation().inverse();
  Rot3 RBjtoW = PWtoBj.rotation();
  Rot3 RGjtoBj = PGjtoBj.rotation().inverse();
  /** Translations */
  Point3 tWtoBi_W = PWtoBi.translation();
  Point3 tGitoBi_Gi = PGitoBi.translation();
  Point3 tWtoBj_W = PWtoBj.translation();
  Point3 tGjtoBj_Gj = PGjtoBj.translation();

  /** Jacobians */
  Point3 xi = RBitoW * RGitoBi * (tGitoBi_Gi - tGtoBt_G);
  Point3 xj = RBjtoW * RGjtoBj * (tGtoBt_G - tGjtoBj_Gj);
  if (H1) {  // dKappa/dRBitoW, and dKappa/tWtoBi_W
    *H1 = (Matrix(3, 6) << -skewSymmetric(xi.x(), xi.y(), xi.z()) *
                               RBitoW.matrix(),
           -RBitoW.matrix())
              .finished();
  }
  if (H2) {  // dKappa/dRBitoGi, and dKappa/tGitoBi_Gi
    *H2 = (Matrix(3, 6) << skewSymmetric(xi.x(), xi.y(), xi.z()) *
                               RBitoW.matrix(),
           RBitoW.matrix())
              .finished();
  }
  if (H3) {  // dKappa/dRBjtoW, and dKappa/tWtoBj_W
    *H3 = (Matrix(3, 6) << -skewSymmetric(xj.x(), xj.y(), xj.z()) *
                               RBjtoW.matrix(),
           RBjtoW.matrix())
              .finished();
  }
  if (H4) {  // dKappa/dRBjtoGj, and dKappa/tGjtoBj_Gj
    *H4 = (Matrix(3, 6) << skewSymmetric(xj.x(), xj.y(), xj.z()) *
                               RBjtoW.matrix(),
           -RBjtoW.matrix())
              .finished();
  }
  if (H5) {  // dKappa/tGtoBt_G
    *H5 = (Matrix(3, 3) << RBjtoW.matrix() * RGjtoBj.matrix() -
                               RBitoW.matrix() * RGitoBi.matrix())
              .finished();
  }

  /**
   * The vector addition below will completes a loop and will equal the zero
   * vector when the kinematic constraints are satisfied.
   */
  return RBitoW * RGitoBi * (tGitoBi_Gi - tGtoBt_G) +
         RBjtoW * RGjtoBj * (tGtoBt_G - tGjtoBj_Gj) + (tWtoBj_W - tWtoBi_W);
}

/* ************************************************************************* */
void RotationKinematicFactor::print(const std::string& s,
                                    const KeyFormatter& keyFormatter) const {
  Base::print(s, keyFormatter);
  if (noiseModel_) noiseModel_->print("  noise model: ");
}

/* ************************************************************************* */
bool RotationKinematicFactor::equals(const RotationKinematicFactor& factor,
                                     double tol) const {
  return Base::equals(factor);
}

}  // end namespace gtsam
