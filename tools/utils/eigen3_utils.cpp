#include "utils/eigen3_utils.h"

#include <iostream>

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

void normalizeFloatVector(std::vector<float> &values)
{
	float min_sal = *min_element(values.begin(), values.end());
    float max_sal = *max_element(values.begin(), values.end());

    for (size_t i = 0; i < values.size(); ++i)
    	values[i] = (values[i]-min_sal)/(max_sal - min_sal);
}

void logNormalizeFloatVector(std::vector<float> &values, float a)
{
	float min_sal = *min_element(values.begin(), values.end());
    float max_sal = *max_element(values.begin(), values.end());

    for (size_t i = 0; i < values.size(); ++i)
    	values[i] = log(((values[i]-min_sal)/(max_sal - min_sal))*a + 1)/log(a+1);
}

void removeOutliers(std::vector<float> &values, 
                    float min_percent, 
                    float max_percent) {
  Eigen::Map<Eigen::VectorXf> vec(&values[0], values.size());
  float min_val = vec.minCoeff();
  float mean = vec.mean();
  float stddev = sqrt((vec.array()-mean).array().pow(2).sum()/values.size());
  // std::cout << vec.minCoeff() << " " << vec.maxCoeff() << " " << vec.mean() << " " << min_percent*stddev << " " << max_percent*stddev << "\n";
  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i] < mean-min_percent*stddev)  values[i] = mean-min_percent*stddev;
    if (values[i] > mean+max_percent*stddev)  values[i] = mean+max_percent*stddev;
  }
}

// http://en.wikipedia.org/wiki/Principal_component_analysis
// Specially the section on Singular value decomposition
// Also check http://forum.kde.org/viewtopic.php?f=74&t=110265
void reduceDimension(Eigen::MatrixXf &data, int new_dim)
{
	Eigen::RowVectorXf mean = data.colwise().mean();
	Eigen::MatrixXf centered = data.rowwise() - mean;
	// Note: we should not divide by std for normalization

	std::cout << "Dimension reduction\n";
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::VectorXf diag = svd.singularValues();
	if (new_dim == 0)
	{
		for (size_t k = 0; k < diag.size(); ++k)
		{
			if (diag[k] > 1e-3)
				new_dim += 1;
			else
				break;
		}
	}
	if (new_dim >= data.cols())	return;
	diag = svd.singularValues().topRows(new_dim);
	// std::cout << "SVD : " << svd.matrixU().cols() << " | " << diag.transpose() << "\n";
	data = svd.matrixU().leftCols(new_dim)*diag.asDiagonal();

	
	/*Eigen::MatrixXf cov = (centered.adjoint() * centered)/(centered.rows()-1); // or (ctr*ctr.transpose)/(n-1)
	std::cout << "Eigendecomposition\n";
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);
	Eigen::VectorXf diag = eig.eigenvalues();
	if (new_dim == 0)
	{
		for (int k = ((int) diag.size()) - 1; k >= 0; --k)
		{
			if (diag[k] > 1e-9)
				new_dim += 1;
			else
				break;
		}
	}
	if (new_dim >= data.cols())	return;
	data = data*eig.eigenvectors().rightCols(new_dim);*/
	// std::cout << "Eigenvalues " << eig.eigenvalues() << "\n";

	std::cout << "Reduced data: " << data.rows() << " x " << data.cols() << "\n\n";
}

void normalizeData(Eigen::MatrixXf &data)
{
    Eigen::RowVectorXf mean = data.colwise().mean();
    Eigen::RowVectorXf std = ((data.rowwise() - mean).array().square().colwise().sum() / (data.rows() - 1)).sqrt();
    data = (data.rowwise() - mean);
    for (size_t k = 0; k < std.size(); ++k)
        data.col(k) /= std[k];
    // observations_ = (observations_.rowwise() - mean).array().rowwise() / std.array();
}

float compute_variation(
	  const Eigen::MatrixXf &data_points, Eigen::RowVectorXf mean) {
	Eigen::MatrixXf centered = data_points.rowwise() - mean;
	Eigen::MatrixXf cov_mat =  (centered.adjoint() * centered)/(centered.rows()); 
	// Eigen::MatrixXf cov_mat =  (centered.transpose() * centered); 
	Eigen::RowVectorXf eigenvalues = 
			cov_mat.selfadjointView<Eigen::Lower>().eigenvalues();
  // std::cout << eigenvalues.minCoeff() << " " << eigenvalues.sum() << " " << centered.array().pow(2).sum() << "\n";
  return eigenvalues.minCoeff()/eigenvalues.sum();
	// return es.eigenvalues().sum();
}

Eigen::MatrixXf distL1(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y)
{
	int m = x.rows(); int n = y.rows();
	Eigen::MatrixXf D = Eigen::MatrixXf::Zero(m, n);

	Eigen::RowVectorXf yi;
	for (size_t i = 0; i < n; ++i)
	{	
		yi = y.row(i); 
		D.col(i) = (x.rowwise() - yi).cwiseAbs().rowwise().sum();
	}
	return D;
}

Eigen::MatrixXf distEucSq(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y)
{
	int m = x.rows(); int n = y.rows();

	Eigen::MatrixXf XX = (x.cwiseProduct(x)).rowwise().sum();
	Eigen::MatrixXf YY = (y.transpose().cwiseProduct(y.transpose())).colwise().sum();
	
	return XX*Eigen::RowVectorXf::Ones(n) + Eigen::VectorXf::Ones(m)*YY - 2*x*y.transpose();
}

Eigen::MatrixXf distChiSq(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y)
{
	int m = x.rows(); int n = y.rows();
	Eigen::MatrixXf D = Eigen::MatrixXf::Zero(m, n);

	Eigen::RowVectorXf yi;
	for (size_t i = 0; i < n; ++i)
	{	
		yi = y.row(i); 

		Eigen::MatrixXf d = -(x.rowwise() - yi);
		Eigen::MatrixXf s = x.rowwise() + yi;
		Eigen::RowVectorXf eps = Eigen::RowVectorXf::Constant(s.cols(), 1e-10);
		D.col(i) = ((d.cwiseProduct(d)).array()/(s.rowwise() + eps).array()).rowwise().sum();
	}
	return D*0.5;
}

/*
http://www.cs.columbia.edu/~mmerler/project/code/pdist2.m

function D = distL1( X, Y )

m = size(X,1);  n = size(Y,1);
mOnes = ones(1,m); D = zeros(m,n);
for i=1:n
  yi = Y(i,:);  yi = yi( mOnes, : );
  D(:,i) = sum( abs( X-yi),2 );
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = distCosine( X, Y )

if( ~isa(X,'double') || ~isa(Y,'double'))
  error( 'Inputs must be of type double'); end;

p=size(X,2);
XX = sqrt(sum(X.*X,2)); X = X ./ XX(:,ones(1,p));
YY = sqrt(sum(Y.*Y,2)); Y = Y ./ YY(:,ones(1,p));
D = 1 - X*Y';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = distEmd( X, Y )

Xcdf = cumsum(X,2);
Ycdf = cumsum(Y,2);

m = size(X,1);  n = size(Y,1);
mOnes = ones(1,m); D = zeros(m,n);
for i=1:n
  ycdf = Ycdf(i,:);
  ycdfRep = ycdf( mOnes, : );
  D(:,i) = sum(abs(Xcdf - ycdfRep),2);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = distChiSq( X, Y )

%%% supposedly it's possible to implement this without a loop!
m = size(X,1);  n = size(Y,1);
mOnes = ones(1,m); D = zeros(m,n);
for i=1:n
  yi = Y(i,:);  yiRep = yi( mOnes, : );
  s = yiRep + X;    d = yiRep - X;
  D(:,i) = sum( d.^2 ./ (s+eps), 2 );
end
D = D/2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function D = distEucSq( X, Y )

%if( ~isa(X,'double') || ~isa(Y,'double'))
 % error( 'Inputs must be of type double'); end;
m = size(X,1); n = size(Y,1);
%Yt = Y';
XX = sum(X.*X,2);
YY = sum(Y'.*Y',1);
D = XX(:,ones(1,n)) + YY(ones(1,m),:) - 2*X*Y';



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function D = distEucSq( X, Y )
%%%% code from Charles Elkan with variables renamed
% m = size(X,1); n = size(Y,1);
% D = sum(X.^2, 2) * ones(1,n) + ones(m,1) * sum(Y.^2, 2)' - 2.*X*Y';
*/
