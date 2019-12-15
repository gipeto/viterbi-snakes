#pragma once

#include <Eigen/Core>
#include <array>
#include <vector>

namespace vsnakes
{
namespace detail
{
template <int StateDim>
using PositionsBuffer = Eigen::Matrix<uint8_t, Eigen::Dynamic, StateDim, Eigen::RowMajor>;

/// Optimize the anchor points using the viterbi algorithm
/// @tparam StateDim the number of possible states for each anchor point
/// @tparam EnergyEvaluator a functor with signature float(const Eigen::Vector2i& nextAnchor,const Eigen::Vector2i&
/// currentAnchor),
///         returning the energy evaluated at currentAnchor
/// @param anchors initial positions of the anchors to be optmized
/// @param neighborsOffsets offset to apply to anchor points to retrieve its neighbors. Each neighbor correspond to a
/// possible state for the anchor.
/// @param positionBuffer table of size StateDim * anchors.size()-1 storing for each anchor the state index
/// corresponsing to the minimum energy path to its preceeding anchor.
///                       Note that this buffer is injected to avoid re-allocation at each iteration of the optimization
///                       but doesn't carry any state information between iterations.
/// @param evaluateEnergy functor with signature float(const Eigen::Vector2i& nextAnchor,const Eigen::Vector2i&
/// currentAnchor),
///                       returning the energy evaluated at currentAnchor
template <int StateDim, typename EnergyEvaluator>
std::pair<std::vector<Eigen::Vector2i>, float> optimizeAnchors(
    const std::vector<Eigen::Vector2i>& anchors, const std::array<Eigen::Vector2i, StateDim>& neighborsOffsets,
    PositionsBuffer<StateDim>& positionsBuffer, EnergyEvaluator&& evaluateEnergy);

}  // namespace detail

}  // namespace vsnakes

#include "optimize_anchors.hpp"
