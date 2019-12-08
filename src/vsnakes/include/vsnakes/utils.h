#pragma once
#include "vsnakes_export.h"

#include <Eigen/Core>
#include <string>
#include <vector>

namespace vsnakes
{
/// Print anchors points
/// @param message message preceding the anchor printing
/// @param anchors the anchors to print
VSNAKES_EXPORT void printAnchors(std::string message, const std::vector<Eigen::Vector2i>& anchors);

}  // namespace vsnakes
