#include <Eigen/Core>
#include <Eigen/SVD>

template <class MatrixType>
class TruncatedSVD
{
    typedef typename MatrixType::Scalar Scalar;
    Eigen::JacobiSVD<MatrixType> svd;
    Eigen::DiagonalMatrix<Scalar, Eigen::Dynamic> sinv;
    Scalar eigenValueThreshold;
    int numTruncated;

public:

    static double defaultEigenValueThreshold() { return 1.2e2; }

    TruncatedSVD() {
        eigenValueThreshold = defaultEigenValueThreshold();
        numTruncated = 0;
    }

    void setEigenValueThreshold(Scalar r){
        if(r <= 0.0){
            eigenValueThreshold = std::numeric_limits<double>::infinity();
        } else {
            eigenValueThreshold = r;
        }
    }

    TruncatedSVD& compute(const MatrixType& matrix){
        
        // We dont truncate
        numTruncated = 0;
            
        svd.compute(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        sinv.diagonal() = svd.singularValues();

        int lastNonZeroSingularValues = svd.nonzeroSingularValues() - 1;
        if(lastNonZeroSingularValues > 0){
            // Get the Biggest eigenValue
            Scalar& s0 = sinv.diagonal()(0);

            // Get the Smallest eigenValue
            Scalar& s_last = sinv.diagonal()(lastNonZeroSingularValues);

            // Just skip operation if the last eigen value is too big.
            if(s_last > Scalar(250.0)){
                return *this;
            }

            int j;
            for(j = lastNonZeroSingularValues; j > 0; --j){
                Scalar& s = sinv.diagonal()(j);
                if(s < eigenValueThreshold){
                    //std::cout << "SMALL EIGENVALUE: " << s << std::endl;
                    s = Scalar(0.0);
                    ++numTruncated;
                } else {
                    break;
                }
            }


            while(j >= 0){
                Scalar& s = sinv.diagonal()(j);
                s = Scalar(1.0) / s;
                --j;
            }
            

            // Relative Condition number based, bad idea since rotation vs translation scale
            /*
            if((s0 / s_last) > eigenValueThreshold){
                s_last = Scalar(0.0);
                ++numTruncated;
                int j;
                for(j = lastNonZeroSingularValues - 1; j > 0; --j){
                    Scalar& s = sinv.diagonal()(j);
                    if((s0 / s) > eigenValueThreshold){
                        s = Scalar(0.0);
                        ++numTruncated;
                    } else {
                        break;
                    }
                }
                while(j >= 0){
                    Scalar& s = sinv.diagonal()(j);
                    s = Scalar(1.0) / s;
                    --j;
                }
            }*/
        }
        return *this;
    }

    int numTruncatedSingularValues() const {
        return numTruncated;
    }

    const typename Eigen::JacobiSVD<MatrixType>::SingularValuesType& singularValues() const {
        return svd.singularValues();
    }
        
    template <class VectorType1, class VectorType2>
    void solve(const Eigen::MatrixBase<VectorType1>& b, Eigen::MatrixBase<VectorType2>& out_x) const {
        if(numTruncated > 0){
            // Use the re-mapped singular values
            out_x = svd.matrixV() * sinv * svd.matrixU().transpose() * b;
        } else {
            out_x = svd.solve(b);
        }
    }
};