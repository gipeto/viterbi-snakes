#include <vsnakes/utils.h>

#include <iostream>

namespace vsnakes
{
void printAnchors(std::string message, const std::vector<Eigen::Vector2i>& anchors)
{
    std::cout << message << " : ";
    for (const auto& anchor : anchors)
    {
        std::cout << " (" << anchor.x() << "," << anchor.y() << ") ";
    }
    std::cout << std::endl;
}

}  // namespace vsnakes
