#include <gmock/gmock.h>

#include <cmath>
#include <stdexcept>

#include <vsnakes/vsnakes.h>

using namespace ::testing;
using namespace vsnakes;

namespace
{
std::vector<Eigen::Vector2i> generateAnchors(Eigen::Vector2i firstAnchor, Eigen::Vector2i delta, int numAnchors)
{
    std::vector<Eigen::Vector2i> anchors;

    for (int aidx = 0; aidx < numAnchors; ++aidx)
    {
        Eigen::Vector2i step{delta.x() * aidx, delta.y() * std::pow(-1, aidx)};
        Eigen::Vector2i anchor = firstAnchor.array() + step.array();
        anchors.push_back(std::move(anchor));
    }

    return anchors;
}

void setEnergyAlongAxisAlignedLine(Eigen::Vector2i start, Eigen::Vector2i stop, bool horizontal, float energy,
                                   Eigen::MatrixXf& energyMap)
{
    if (horizontal)
    {
        int startx, stopx, y;
        if (start.x() < stop.x())
        {
            startx = start.x();
            stopx = stop.x();
            y = start.y();
        }
        else
        {
            startx = stop.x();
            stopx = start.x();
            y = stop.y();
        }

        for (int x = startx; x <= stopx; ++x)
        {
            energyMap(y, x) = energy;
        }
        return;
    }

    int starty, stopy, x;
    if (start.y() < stop.y())
    {
        starty = start.y();
        stopy = stop.y();
        x = start.x();
    }
    else
    {
        starty = stop.y();
        stopy = start.y();
        x = stop.x();
    }

    for (int y = starty; y <= stopy; ++y)
    {
        energyMap(y, x) = energy;
    }
}

}  // namespace

class AnOptimizeAnchorsFirstOrder : public Test
{
public:
    void SetUp() override
    {
        anchors = generateAnchors({0, 3}, {2, 2}, static_cast<int>(minOffsets.size()));
        energyMap.resize(rows, cols);
        energyMap.fill(0.f);
    }

    std::vector<Eigen::Vector2i> setEnergyMapMinimum()
    {
        std::vector<Eigen::Vector2i> minEnergyPositions(anchors.size());
        for (int i = 0; i < anchors.size(); ++i)
        {
            minEnergyPositions[i] = anchors[i] + minOffsets[i];
            energyMap(minEnergyPositions[i].y(), minEnergyPositions[i].x()) = -1.f;
        }
        return minEnergyPositions;
    }

    std::vector<Eigen::Vector2i> generateDoubleEnergyProfiles(float energySmooth, float energySharp)
    {
        // Generate smooth energy curve
        setEnergyAlongAxisAlignedLine({0, 4}, {8, 4}, true, energySmooth, energyMap);
        // Generate sharp energy curve
        setEnergyAlongAxisAlignedLine({2, 0}, {2, 7}, false, energySharp, energyMap);
        setEnergyAlongAxisAlignedLine({3, 7}, {7, 7}, true, energySharp, energyMap);
        setEnergyAlongAxisAlignedLine({7, 6}, {7, 0}, false, energySharp, energyMap);

        return {{0, 3}, {1, 2}, {2, 4}, {3, 6}, {4, 7}, {5, 3}, {6, 5}, {7, 2}, {8, 3}};
    }

    float alpha = 1.f;
    float gamma = 1.f;
    int maxIter = 100;
    int rows = 9;
    int cols = 9;
    int anchorSpacing = 1;
    std::vector<Eigen::Vector2i> anchors;
    std::vector<Eigen::Vector2i> minOffsets{{1, 1}, {0, 0}, {1, 0}, {-1, -1}};
    Eigen::MatrixXf energyMap;
};

TEST_F(AnOptimizeAnchorsFirstOrder, throwsIfAnchorOutsideImage)
{
    auto outOfBoundsAnchors = anchors;
    outOfBoundsAnchors[0].x() = -1;
    ASSERT_THROW(optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, outOfBoundsAnchors, energyMap),
                 std::invalid_argument);

    outOfBoundsAnchors = anchors;
    outOfBoundsAnchors[2].y() = 100;
    ASSERT_THROW(optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, outOfBoundsAnchors, energyMap),
                 std::invalid_argument);
}

TEST_F(AnOptimizeAnchorsFirstOrder, throwsForEmptyAndSingleAnchor)
{
    ASSERT_THROW(optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, {}, energyMap), std::invalid_argument);
    ASSERT_THROW(optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, {{2, 2}}, energyMap),
                 std::invalid_argument);
}

TEST_F(AnOptimizeAnchorsFirstOrder, throwsForEmptyEnergyMap)
{
    ASSERT_THROW(optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, anchors, {}), std::invalid_argument);
}

TEST_F(AnOptimizeAnchorsFirstOrder, convergeToUnitSpacedAnchorsForConstantEnergyMap)
{
    const auto optimizedAnchors = optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, anchors, energyMap);

    for (size_t i = 0; i < optimizedAnchors.size() - 1; ++i)
    {
        const auto distance = (optimizedAnchors[i + 1] - optimizedAnchors[i]).norm();
        ASSERT_THAT(distance, Eq(1));
    }
}

TEST_F(AnOptimizeAnchorsFirstOrder, convergeToEnergyMapMinumumWithoutShapeConstraints)
{
    const auto expectedOptimizedAnchors = setEnergyMapMinimum();
    const auto optimizedAnchors = optimizeAnchorsFirstOrder(0.f, gamma, anchorSpacing, maxIter, anchors, energyMap);

    ASSERT_THAT(optimizedAnchors, ContainerEq(expectedOptimizedAnchors));
}

TEST_F(AnOptimizeAnchorsFirstOrder, convergeTowardsTheLinearSolution)
{
    const auto initialAnchors = generateDoubleEnergyProfiles(-1.0001f, -1.f);
    const auto optimizedAnchors =
        optimizeAnchorsFirstOrder(alpha, gamma, anchorSpacing, maxIter, initialAnchors, energyMap);

    for (const auto& anchor : optimizedAnchors)
    {
        ASSERT_THAT(anchor.y(), Eq(4));
    }
}

int main(int argc, char* argv[])
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}