#ifndef COMMON_TOOL_INCLUDE_TOOL_CONCATENATE_VECTOR_H_
#define COMMON_TOOL_INCLUDE_TOOL_CONCATENATE_VECTOR_H_

#include <Eigen/Dense>
#include <vector>

namespace math {

/**
 * @brief Concatenates a vector of Eigen vectors into a single Eigen vector.
 *
 * This function takes a vector of Eigen vectors and concatenates them into a single Eigen vector.
 * The resulting vector will have a size equal to the sum of the sizes of the input vectors.
 *
 * @tparam Derived The derived type of the input Eigen vectors.
 * @param data The vector of Eigen vectors to be concatenated.
 * @return The concatenated Eigen vector.
 */
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> ConcatenateVectors(const std::vector<Derived>& data) {
  using Scalar = typename Derived::Scalar;
  using VectorType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  std::size_t total_size = 0;
  for (const auto& vec : data) {
    total_size += vec.size();
  }

  VectorType res(total_size);

  std::size_t index = 0;
  for (const auto& vec : data) {
    res.segment(index, vec.size()) = Eigen::Map<const VectorType>(vec.data(), vec.size());
    index += vec.size();
  }

  return res;
}
}  // namespace common

#endif  // COMMON_TOOL_INCLUDE_TOOL_CONCATENATE_VECTOR_H_
