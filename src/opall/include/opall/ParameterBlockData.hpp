#pragma once

#include <vector>

namespace opall
{

struct ParameterBlockData
{
    double* address;
    int position;
    int size;
};


struct ParameterBlockDataContainer
{
    std::vector<ParameterBlockData> data;
};


}