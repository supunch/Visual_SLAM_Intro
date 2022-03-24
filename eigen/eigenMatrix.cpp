#include <iostream>
using namespace std;

#include <ctime>
// Eigen core
#include <Eigen/Core>
// Algebraic operations of dense matrices (inverse, eigenvalues, etc.)
#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

int main(int argc, char **argv) {
    
    Matrix<float, 2, 3> matrix_23;
    cout << "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;
    

    return 0;
}
