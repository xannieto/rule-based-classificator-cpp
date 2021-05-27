#ifndef PSEUDO_INVERSE_MOORE_PENROSE
#define PSEUDO_INVERSE_MOORE_PENROSE

#include <Eigen/Core>
#include <Eigen/SVD>

// method for calculating the pseudo-Inverse as recommended by Eigen developers
template<typename _Matrix_Type_>
inline _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &A, double epsilon = std::numeric_limits<double>::epsilon())
{
	// For a square matrix
	//Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV);
    // For a non-square matrix
	Eigen::JacobiSVD<_Matrix_Type_> svd(A ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(A.cols(), A.rows()) * svd.singularValues().array().abs()(0);
	
	return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

#endif // PSEUDO_INVERSER_MOORE_PENROSE