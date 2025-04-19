#pragma once

#include "LossFunctionDescription.hpp"

#include <utility>

namespace opall
{
struct ModellingConfig
{
    LossFunctionDescription lossFunctionOfPointToPoint{};
    LossFunctionDescription lossFunctionOfObservedPoint{};
};
} // namespace opall