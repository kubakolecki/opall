#include "opall/block_covariance_computation.hpp"


namespace opall::covariance
{
    std::vector<std::optional<Eigen::MatrixXd>>  getSubmatrices(const BlockMatrixDataContainer& blockMatrixDataContainer, const Eigen::MatrixXd& fullCovarianceMatrx)
    {
        std::vector<std::optional<Eigen::MatrixXd>> submatrices;
        const auto numberOfSumbatrices{blockMatrixDataContainer.size()};
        submatrices.reserve(numberOfSumbatrices);

        for (const auto& blockMatrixData: blockMatrixDataContainer.getData() )
        {
            if (blockMatrixData.row >= fullCovarianceMatrx.rows() )
            {
                submatrices.push_back(std::nullopt); 
            }

            if (blockMatrixData.column >= fullCovarianceMatrx.cols() )
            {
                submatrices.push_back(std::nullopt); 
            }

            if (blockMatrixData.row + blockMatrixData.numberOfRows >  fullCovarianceMatrx.rows() )
            {
                submatrices.push_back(std::nullopt);
            }

            if (blockMatrixData.column + blockMatrixData.numberOfColumns > fullCovarianceMatrx.cols() )
            {
                submatrices.push_back(std::nullopt);
            }

            submatrices.push_back(fullCovarianceMatrx.block(blockMatrixData.row, blockMatrixData.column,blockMatrixData.numberOfRows,blockMatrixData.numberOfColumns) );
        }

        return submatrices;
    }




}