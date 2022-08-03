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

/**
 * @brief The main namespace for all itslam components.
 **/
namespace xrsfm {

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

}  // namespace xrsfm

#endif  // ITSLAM_GLOBAL_H
