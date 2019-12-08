#pragma once

#include "vsnakes_export.h"

#include <Eigen/Core>
#include <vector>

namespace vsnakes
{
/// Optimize anchors using first degree constraints for the active contour
/// @param alpha weight for the internal energy term
/// @param gamma weight for the external energy term
/// @param anchorSpacing targetSpacing between consecutive anchors
/// @param maxIter maximal number of iterations
/// @param anchors inital position of the curve anchor points
/// @param energyMap map of external energy at every possible location
/// @returns the optimized anchor points
VSNAKES_EXPORT std::vector<Eigen::Vector2i> optimizeAnchorsFirstOrder(float alpha, float gamma, int anchorSpacing,
                                                                      int maxIter, std::vector<Eigen::Vector2i> anchors,
                                                                      const Eigen::MatrixXf& energyMap);

}  // namespace vsnakes
