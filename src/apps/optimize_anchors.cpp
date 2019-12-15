#include <Eigen/Core>
#include <iostream>

#include <vsnakes/utils.h>
#include <vsnakes/vsnakes.h>

namespace
{
std::vector<Eigen::Vector2i> generateInitialAnchors(int rows = 64, int cols = 64, int numAnchors = 8)
{
    const auto center = cols / 2;
    const auto delta = rows / numAnchors;

    std::vector<Eigen::Vector2i> anchors;

    for (int aidx = 0; aidx < numAnchors; ++aidx)
    {
        Eigen::Vector2i anchor =
            (aidx % 2 == 0) ? Eigen::Vector2i{center - 5, aidx * delta} : Eigen::Vector2i{center + 5, aidx * delta};
        anchors.push_back(std::move(anchor));
    }

    return anchors;
}

Eigen::MatrixXf generateEnergyMap(int rows = 64, int cols = 64)
{
    Eigen::MatrixXf energyMap(rows, cols);

    const int half = cols / 2;

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < half; ++j)
        {
            energyMap(i, j) = static_cast<float>(cols - j);
        }

        for (int j = half; j < cols; ++j)
        {
            energyMap(i, j) = static_cast<float>(cols - half + 1 + j);
        }
    }

    return energyMap;
}

}  // namespace

int main(int argc, char* argv[])
{
    /// TODO: Allow passing a set of anchor and an image for the optimization

    // Inputs for the optimization
    const int rows = 64;
    const int cols = 64;
    const int numAnchors = 8;
    const auto spacing = 8;

    const std::vector<Eigen::Vector2i> anchors = generateInitialAnchors(rows, cols, numAnchors);
    const Eigen::MatrixXf energyMap = generateEnergyMap(rows, cols);

    const auto optimizedAnchors = vsnakes::optimizeAnchorsFirstOrder(0.001f, 1.f, spacing, 200, anchors, energyMap);

    vsnakes::printAnchors("Initial", anchors);
    vsnakes::printAnchors("Optimized", optimizedAnchors);
}