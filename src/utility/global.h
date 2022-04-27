/**
 * @brief Types for global use.
 * @file global.h
 * @version @buildversion
 * @author Jinyu Li (itsuhane@gmail.com)
 * @date @builddate
 **/
#ifndef ITSLAM_GLOBAL_H
#define ITSLAM_GLOBAL_H

#include <Eigen/Eigen>

//#include "version.h"

//#include "patch/patch.h"

#define GRAVITY_NOMINAL 9.80665

/**
 * @brief The main namespace for all itslam components.
 **/
namespace itslam {

/**
 * @brief Dense matrix type used in itslam.
 * @tparam
 *  Rows number of matrix rows.
 * @tparam
 *  Cols number of matrix columns. Default to match `Rows`.
 * @tparam
 *  UseRowMajor specify storage order,
 * `false` for column-major (default), `true` for row-major.
 * @tparam
 *  T underlying scalar type, itslam always uses `double`.
 * @see @ref vector, @ref quaternion, @ref map, @ref const_map.
 **/
template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<Rows != 1 && Cols != 1,
                                         Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
                                         Eigen::Matrix<T, Rows, Cols>>::type;

/**
 * @brief Row vector type used in itslam.
 * @tparam Rows number of vector rows.
 * @tparam T underlying scalar type, itslam always uses `double`.
 * @see @ref matrix, @ref quaternion, @ref map, @ref const_map.
 **/
template <int Rows = Eigen::Dynamic, typename T = double>
using vector = matrix<Rows, 1, false, T>;

/**
 * @brief Maps continuous memory to a compatible matrix/vector/quaternion.
 * @tparam T target type to map.
 * @see @ref matrix, @ref vector, @ref quaternion, @ref const_map.
 **/
template <typename T>
using map = Eigen::Map<T>;

/**
 * @brief Maps continuous memory to a compatible matrix/vector/quaternion, while
 *holding the content constant.
 * @tparam T target type to map.
 * @see @ref matrix, @ref vector, @ref quaternion, @ref map.
 **/
template <typename T>
using const_map = Eigen::Map<const T>;

/**
 * @brief
 *  Quaternion type used in itslam.
 * @details
 *  itslam uses quaternion for representing orientations/rotations.
 *  Quaternions follow Hamilton-convention and should always be normalized to
 *unit-length. Check `Eigen::Quaternion` for storage order of its four
 *components.
 * @see @ref matrix, @ref vector, @ref map, @ref const_map.
 **/
using quaternion = Eigen::Quaternion<double>;

inline constexpr size_t nil() { return size_t(-1); }

// struct IMUData {
//    double t;    // timestamp
//    vector<3> w; // gyro measurement
//    vector<3> a; // accelerometer measurement
//};
//
// struct Image {
//    double t;
//    vector<3> g; // don't touch this!
//    virtual ~Image() = default;
//    virtual void detect_keypoints(std::vector<vector<2>> &keypoints, size_t
//    max_points = 0) const = 0; virtual void track_keypoints(const Image
//    *next_image, const std::vector<vector<2>> &curr_keypoints,
//    std::vector<vector<2>> &next_keypoints, std::vector<char> &result_status)
//    const = 0; virtual void show_direction(const std::string &name, const
//    vector<3> &direction, const matrix<3> &K) const {} virtual void
//    detect_segments(std::vector<std::pair<vector<2>, vector<2>>> &segments)
//    const {
//        segments.clear();
//    }
//    virtual void show_segments(ITSLAM_UNUSED_ATTR const
//    std::vector<std::pair<vector<2>, vector<2>>> &segments) const {
//        ITSLAM_UNUSED_EXPR(segments);
//    }
//};

}  // namespace itslam

//#include "system/diagnosis.h"
//#include "utility/debug.h"

#endif  // ITSLAM_GLOBAL_H
