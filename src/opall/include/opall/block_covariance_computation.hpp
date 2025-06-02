#pragma once

#include <Eigen/Dense>

#include <vector>
#include <optional>

namespace opall::covariance
{
    struct BlockMatrixData
    {
        int row;
        int column;
        int numberOfRows;
        int numberOfColumns;

    };
    
    struct BlockMatrixDataContainer
    {
        public:

            void clear()
            {
                data.clear();
            }

            void addData(int row, int column, int numberOfRows, int numberOfColumns)
            {
                data.emplace_back(row, column, numberOfRows, numberOfColumns);
            }

            size_t size() const
            {
                return data.size();
            }

            const std::vector<BlockMatrixData>& getData() const
            {
                return data;
            }

        private:

            std::vector<BlockMatrixData> data;
    };
    

    std::vector<std::optional<Eigen::MatrixXd>> getSubmatrices(const BlockMatrixDataContainer& blockMatrixDataContainer, const Eigen::MatrixXd& fullCovarianceMatrx);

}