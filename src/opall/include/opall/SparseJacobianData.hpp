#pragma once

#include <Eigen/Sparse>

#include<vector>


namespace opall
{

struct SparseJacobianData
{
  int numberOfRows;
  int numberOfColumns;
  std::vector<Eigen::Triplet<double>> triplets;
};

}