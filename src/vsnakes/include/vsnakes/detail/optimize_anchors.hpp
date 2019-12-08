#include "optimize_anchors.h"

namespace vsnakes
{
namespace detail
{
template <int StateDim, typename EnergyEvaluator>
std::pair<std::vector<Eigen::Vector2i>, float> optimizeAnchors(
    const std::vector<Eigen::Vector2i>& anchors, const std::array<Eigen::Vector2i, StateDim>& neighborsOffset,
    PositionsBuffer<StateDim>& positionsBuffer, EnergyEvaluator&& evaluateEnergy)
{
    const auto numAnchors = static_cast<int>(anchors.size());

    positionsBuffer.resize(numAnchors - 1, StateDim);

    // Store the minimal energy of the previous state
    std::array<float, StateDim> stateEnergy{};
    std::array<float, StateDim> stateEnergyUpdate{};

    for (int anchorIdx = 0; anchorIdx < numAnchors - 1; ++anchorIdx)
    {
        const auto& currentAnchor = anchors[anchorIdx];

        for (int nextAnchorNeighborIdx = 0; nextAnchorNeighborIdx < StateDim; ++nextAnchorNeighborIdx)
        {
            float minEnergy = std::numeric_limits<float>::max();
            int optimalPosition = 0;
            const auto nextAnchor = anchors[anchorIdx + 1] + neighborsOffset[nextAnchorNeighborIdx];

            for (int currentAnchorNeighborIdx = 0; currentAnchorNeighborIdx < StateDim; ++currentAnchorNeighborIdx)
            {
                const auto energy =
                    stateEnergy[currentAnchorNeighborIdx]
                    + evaluateEnergy(nextAnchor, currentAnchor + neighborsOffset[currentAnchorNeighborIdx]);

                if (energy < minEnergy)
                {
                    minEnergy = energy;
                    optimalPosition = currentAnchorNeighborIdx;
                }
            }

            // Update state of local problem
            stateEnergyUpdate[nextAnchorNeighborIdx] = minEnergy;
            positionsBuffer(anchorIdx, nextAnchorNeighborIdx) = optimalPosition;
        }

        // Update state buffer for next anchor evaluation
        stateEnergy.swap(stateEnergyUpdate);
    }

    // Find optimal position for the last anchor
    const auto& lastAnchor = anchors[numAnchors - 1];
    float minAnchorEnergy =
        stateEnergy[0] + evaluateEnergy(lastAnchor + neighborsOffset[0], lastAnchor + neighborsOffset[0]);
    int optimalPosition = 0;

    for (auto i = 1; i < StateDim; ++i)
    {
        const auto anchor = lastAnchor + neighborsOffset[i];
        const float anchorEnergy = stateEnergy[i] + evaluateEnergy(anchor, anchor);
        if (anchorEnergy < minAnchorEnergy)
        {
            minAnchorEnergy = anchorEnergy;
            optimalPosition = i;
        }
    }

    // Backtrace solution

    std::vector<Eigen::Vector2i> optimizedAnchors(numAnchors);
    optimizedAnchors[numAnchors - 1] = lastAnchor + neighborsOffset[optimalPosition];

    for (int anchorIdx = numAnchors - 2; anchorIdx >= 0; anchorIdx--)
    {
        optimalPosition = positionsBuffer(anchorIdx, optimalPosition);
        optimizedAnchors[anchorIdx] = anchors[anchorIdx] + neighborsOffset[optimalPosition];
    }

    return {optimizedAnchors, minAnchorEnergy};
}
}  // namespace detail

}  // namespace vsnakes
