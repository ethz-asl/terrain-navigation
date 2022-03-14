/*!
 * \file DubinsPath.hpp
 *
 * \brief Describes a path between two states.
 *
 *  Created on: Nov 4, 2016
 *      Author: Daniel Schneider, ASL
 *              Florian Achermann, ASL
 */

#ifndef FW_PLANNING_PLANNING__SPACES__DUBINS_PATH_HPP_
#define FW_PLANNING_PLANNING__SPACES__DUBINS_PATH_HPP_

#include <array>
#include <limits>

namespace fw_planning {

namespace spaces {

/** \brief DubinsPath
 * Complete description of a (non-optimal) Dubins airplane path
 */
class DubinsPath {
public:
  /** \brief The Dubins car/airplane path segment type.  */
  enum DubinsPathSegmentType {
    DUBINS_LEFT = 0,
    DUBINS_STRAIGHT = 1,
    DUBINS_RIGHT = 2
  };

  /** \brief Classification
   * Classification of path according to "Classification of the Dubins Set", Shkel, Lumelsky, 2001
   */
  enum Classification {
    CLASS_A11 = 0,
    CLASS_A12 = 1,
    CLASS_A13 = 2,
    CLASS_A14 = 3,
    CLASS_A21 = 4,
    CLASS_A22 = 5,
    CLASS_A23 = 6,
    CLASS_A24 = 7,
    CLASS_A31 = 8,
    CLASS_A32 = 9,
    CLASS_A33 = 10,
    CLASS_A34 = 11,
    CLASS_A41 = 12,
    CLASS_A42 = 13,
    CLASS_A43 = 14,
    CLASS_A44 = 15,
    NOT_ASSIGNED = 16
  };

  /** \brief Index
   * Type of the dubins path.
   */
  enum Index {
    TYPE_LSL = 0,
    TYPE_RSR = 1,
    TYPE_RSL = 2,
    TYPE_LSR = 3,
    TYPE_RLR = 4,
    TYPE_LRL = 5
  };

  /** \brief AltitudeCase
   * Altitude case of the path.
   */
  enum AltitudeCase {
    ALT_CASE_LOW = 0,
    ALT_CASE_MEDIUM = 1,
    ALT_CASE_HIGH = 2
  };

  /** \brief Dubins car path types */
  static const DubinsPathSegmentType dubinsPathType[6][3];

  /** \brief Constructor */
  DubinsPath(Index type = TYPE_LSL,
             double t = 0.0 /* length of first path segment of a 2D Dubins car path */,
             double p = std::numeric_limits<double>::quiet_NaN() /* length of second path segment of a 2D Dubins car path */,
             double q = 0.0 /* length of third path segment of a 2D Dubins car path */,
             double gam = 0.0,
             unsigned int ks = 0,
             unsigned int ke = 0,
             double r = 1.0);

  /** \brief A function returning the length (normalized by minimum radius rho_) of the projection of the
   * 3D (non-optimal) Dubins airplane path on the x-y plane of the world frame. */
  double length_2D() const;

  /** \brief A function returning the length (normalized by minimum radius) of the 3D (non-optimal) Dubins airplane path. */
  double length_3D() const;

  /** \brief Return foundOptimalPath_ */
  bool getFoundOptimalPath() const;

  /** \brief Set foundOptimalPath_ */
  void setFoundOptimalPath(bool found_optimal_path);

  /** \brief Return additionalManeuver_ */
  bool getAdditionalManeuver() const;

  /** \brief Set additionalManeuver_ */
  void setAdditionalManeuver(bool additional_maneuver);

  /** \brief Return lmh_ */
  AltitudeCase getAltitudeCase() const;

  /** \brief Set lmh_ */
  void setAltitudeCase(AltitudeCase altitude_case);

  /** \brief Return idx_ */
  Index getIdx() const;

  /** \brief Set classification_ */
  void setClassification(Classification classification);

  /** \brief Return classification_ */
  Classification getClassification() const;

  /** \brief Set a start helix with num_helix full circles and a radius ratio of radius_ratio. */
  void setStartHelix(unsigned int num_helix, double radius_ratio);

  /** \brief Set a end helix with num_helix full circles and a radius ratio of radius_ratio. */
  void setEndHelix(unsigned int num_helix, double radius_ratio);

  /** \brief Get gamma_ */
  void setGamma(double gamma);

  /** \brief Return gamma_ */
  double getGamma() const;

  /** \brief Return radiusRatio_ of the corresponding index, the index must be between 0 and 5. */
  double getRadiusRatio(unsigned int idx) const;

  /** \brief Return radiusRatioInverse_ of the corresponding index, the index must be between 0 and 5. */
  double getInverseRadiusRatio(unsigned int idx) const;

  /** \brief Return length_ of the corresponding index, the index must be between 0 and 5. */
  double getSegmentLength(unsigned int idx) const;

  /** \brief Set length_ of the corresponding index, the index must be between 0 and 5. */
  void setSegmentLength(double length, unsigned int idx);

  /** \brief Return type_ */
  const DubinsPathSegmentType* getType() const;

private:
  /** \brief type_
   * Path segment types
   */
  const DubinsPathSegmentType* type_;

  /** \brief length_
   * On x-y plane projected path segment lengths, normalized by minimum radius rho_ ( (.)*rho_ gives length of projection of path segments in meters)
   * length_[1,3,4]: length of 2D Dubins car path segments
   * length_[0,5]: length of start/ end helix for optimality in high altitude case
   * length_[2]: length of intermediate maneuver (at start) for optimality in intermediate altitude case
   */
  std::array<double, 6> length_;

  /** \brief length_
   * The 2D length of the curve based on the values in the length_ array.
   */
  double length_2D_;

  /** \brief radiusRatio_
   * Radius ratio (R_opt/rho_) for each segment.
   * For high altitude case, the radius may be bigger than rho_ in order to guarantee optimal paths
   */
  std::array<double, 6> radiusRatio_;

  /** \brief radiusRatioInverse_
   * The inverse value of the radius ratio.
   */
  std::array<double, 6> radiusRatioInverse_;

  /** \brief one_div_cos_abs_gamma_
   * Current path angle (positive for flying upwards)
   */
  double gamma_;

  /** \brief one_div_cos_abs_gamma_
   * 1.0 / cos(fabs(gamma))
   * Computed to speed up the length3D computation.
   */
  double one_div_cos_abs_gamma_;

  /** \brief k_start_
   * Number of circles to fly in end-helix
   */
  unsigned int k_end_;

  /** \brief k_start_
   * Number of circles to fly in start-helix
   */
  unsigned int k_start_;

  /** \brief classification_
   * Classification of path according to "Classification of the Dubins Set", Shkel, Lumelsky, 2001
   * If a is equal to 0, the class of the path corresponds to a_11 of the mentioned paper:
   *  0: a_11
   *  1: a_12
   *  2: a_13
   *  3: a_14,
   *  4: a_21,   ...
   *  ...
   *  12: a_41,  ...
   *  ...
   *  15: a_44
   *  16, the class is not assigned
   */
  Classification classification_;

  /** \brief idx
   * Same information as in type_:
   *   0: LSL
   *   1: RSR
   *   2: RSL
   *   3: LSR
   *   4: RLR
   *   5: LRL
   */
  Index idx_;


  /** \brief lmh
   * Altitude case of the path:
   *     low (0)
   *     (medium (1), never appears for this state space)
   *     high (2)
   */
  AltitudeCase lmh_;

  /** \brief path_case_
   * Indicates if the path consists of three parts (CCC and CSC) with a value of 0 or of four parts
   * CCSC with a value of 1
   */
  bool additionalManeuver_;

  /** \brief foundOptimalPath_
   * True if an optimal path was found, false if no or a suboptimal path was found.
   */
  bool foundOptimalPath_;
};

} // namespace spaces

} // namespace fw_planning

#endif /* FW_PLANNING_PLANNING__SPACES__DUBINS_PATH_HPP_ */
