 /**
 * @file    RotationKinematicFactor.h
 * @brief   Kinematic rotation constraint when observing a rotating rigid body.
 * @author  Tim Setterfield
 *
 * This factor applies when an observer with body frame B has pose estimates
 * relative to a stationary world frame W and a geometric frame G which is
 * attached to a rotating rigid body. The rotating body has an unknown body
 * frame Bt, whose origin is at its center of mass. The origin of the world
 * frame W moves with the origin of the body frame Bt. Here, a kinematic
 * constraint is formed using the translation from the geometric frame G to
 * the body frame Bt (tGtoBt_G).
 * Given variables:
 *   Pose of observer in W:    PWtoB = {RBtoW, tWtoB_W} at time steps i and j
 *   Pose of observer in G:    PGtoB = {RBtoG, tGtoB_G} at time steps i and j
 *   Translation from G to Bt: tGtoBt_G                 at all time steps
 * Then the following constraint can be formed:
 *   [0 0 0]' =    RBitoW*RGitoBi * (tGitoBi_Gi - tGtoBt_G)
 *               + RBjtoW*RGjtoBj * (tGtoBt_G   - tGjtoBj_Gj)
 *               +                  (tWtoBj_W   - tWtoBi_W);
 * Reference:
 *   [1] T. P. Setterfield, “On-Orbit Inspection of a Rotating Object Using a
 *       Moving Observer,” PhD, Massachusetts Institute of Technology, 2017.
 */

#ifndef ROTATIONKINEMATICFACTOR_H_
#define ROTATIONKINEMATICFACTOR_H_

#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

/**
 * A factor for enforcing a kinematic constraint on a rotating rigid body.
 */
class RotationKinematicFactor
    : public NoiseModelFactor5<Pose3, Pose3, Pose3, Pose3, Point3> {
 private:
  typedef NoiseModelFactor5<Pose3, Pose3, Pose3, Pose3, Point3> Base;

 public:
  /** Default constructor - only use for serialization */
  RotationKinematicFactor() {}

  /**
   * Constructor      Note that this factor does not take a measurement
   * @param PWtoBi    Observer body pose relative to stationary W frame at t_i
   * @param PGitoBi   Observer body pose relative to moving G frame at t_i
   * @param PWtoBj    Observer body pose relative to stationary W frame at t_j
   * @param PGitoBj   Observer body pose relative to moving G frame at t_j
   * @param tGtoBt_G  Translation from frame G to frame Bt in G coords
   */
  RotationKinematicFactor(Key PWtoBi, Key PGitoBi, Key PWtoBj, Key PGjtoBj,
                          Key tGtoBt_G, const SharedNoiseModel& model);

  /** Destructor */
  virtual ~RotationKinematicFactor() {}

  /**
   * Error, where the nominal measurement is the zero vector [0 0 0]'
   *  - satisfies requirements of NonlinearFactor
   * */
  Vector evaluateError(const Pose3& PWtoBi, const Pose3& PGitoBi,
                       const Pose3& PWtoBj, const Pose3& PGjtoBj,
                       const Point3& tGtoBt_G,
                       boost::optional<Matrix&> H1 = boost::none,
                       boost::optional<Matrix&> H2 = boost::none,
                       boost::optional<Matrix&> H3 = boost::none,
                       boost::optional<Matrix&> H4 = boost::none,
                       boost::optional<Matrix&> H5 = boost::none) const;

  /**
   * Print with optional string
   * - satisfies requirements of Testable
   * */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /**
   * Check if equal within a tolerance
   * - satisfies requirements of Testable
   * */
  using NonlinearFactor::equals;  // To let compiler know we are not hiding.
  bool equals(const RotationKinematicFactor& factor, double tol = 1e-9) const;

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& boost::serialization::make_nvp(
        "NoiseModelFactor5", boost::serialization::base_object<Base>(*this));
  }
};  // end class RotationKinematicFactor

// Implement the Testable traits
template <>
struct traits<RotationKinematicFactor>
    : public Testable<RotationKinematicFactor> {};

}  // end namespace gtsam

#endif // ROTATIONKINEMATICFACTOR_H_
