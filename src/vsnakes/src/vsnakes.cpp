#include <vsnakes/vsnakes.h>

#include <vsnakes/detail/optimize_anchors.h>

#include <iostream>
#include <stdexcept>

namespace
{
/// Neighborhood definitions
static auto constexpr NUM_NEIGHBORS = 9;

const std::array<Eigen::Vector2i, NUM_NEIGHBORS> NEIGHBORS_OFFSETS = {
    Eigen::Vector2i{-1, -1}, Eigen::Vector2i{0, -1}, Eigen::Vector2i{1, -1},
    Eigen::Vector2i{-1, 0},  Eigen::Vector2i{0, 0},  Eigen::Vector2i{1, 0},
    Eigen::Vector2i{-1, 1},  Eigen::Vector2i{0, 1},  Eigen::Vector2i{1, 1}};

inline bool anchorOutsideImage(int rows, int cols, const Eigen::Vector2i& anchor)
{
    return anchor.x() < 0 || anchor.x() >= cols || anchor.y() < 0 || anchor.y() >= rows;
}

}  // namespace

namespace vsnakes
{
std::vector<Eigen::Vector2i> optimizeAnchorsFirstOrder(float alpha, float gamma, int anchorSpacing, int maxIter,
                                                       std::vector<Eigen::Vector2i> anchors,
                                                       const Eigen::MatrixXf& energyMap)
{
    const auto rows = static_cast<int>(energyMap.rows());
    const auto cols = static_cast<int>(energyMap.cols());

    if (rows == 0 || cols == 0)
    {
        throw std::invalid_argument("Invalid energyMap");
    }

    if (maxIter <= 0)
    {
        throw std::invalid_argument("maxIter must be larger than zero");
    }

    const auto numAnchors = static_cast<int>(anchors.size());
    if (numAnchors < 2)
    {
        throw std::invalid_argument("At least two achors are required");
    }

    for (const auto& anchor : anchors)
    {
        if (anchorOutsideImage(rows, cols, anchor))
        {
            throw std::invalid_argument("The input anchors must have coordinates included in the energyMap domain");
        }
    }

    auto energyFunctor = [&energyMap, alpha, gamma, anchorSpacing, rows, cols](const Eigen::Vector2i& nextAnchor,
                                                                               const Eigen::Vector2i& currentAnchor) {
        if (anchorOutsideImage(rows, cols, nextAnchor) || anchorOutsideImage(rows, cols, currentAnchor))
        {
            return std::numeric_limits<float>::max();
        }

        return alpha * static_cast<float>(std::pow((nextAnchor - currentAnchor).norm() - anchorSpacing, 2))
               + gamma * energyMap(currentAnchor.y(), currentAnchor.x());
    };

    detail::PositionsBuffer<NUM_NEIGHBORS> positionsBuffer(anchors.size() - 1, NUM_NEIGHBORS);

    auto anchorsEnergy = std::numeric_limits<float>::max();
    std::vector<Eigen::Vector2i> updatedAnchors;

    for (int iter = 0; iter < maxIter; ++iter)
    {
        float updatedEnergy{};
        std::tie(updatedAnchors, updatedEnergy) =
            detail::optimizeAnchors<NUM_NEIGHBORS>(anchors, NEIGHBORS_OFFSETS, positionsBuffer, energyFunctor);

        if (updatedEnergy >= anchorsEnergy)
        {
            return anchors;
        }

        anchorsEnergy = updatedEnergy;
        anchors = std::move(updatedAnchors);
    }

    return anchors;
}

}  // namespace vsnakes
