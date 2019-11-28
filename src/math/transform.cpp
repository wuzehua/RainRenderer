//
// Created by rainzhong on 2019/11/28.
//

#include <math/transform.h>
#include <cstring>

namespace rzpbr{

    Matrix Matrix::transpose(const Matrix& m){
        auto matrix = Matrix(0);
        for(auto i = 0;i < 4;i++){
            for(auto j = 0;j < 4;j++){
                matrix.data[i][j] = m.data[j][i];
            }
        }
        return matrix;
    }

    Matrix Matrix::inverse(const Matrix& m){
        //LU
        Float L[4][4],U[4][4];
        Float L_inverse[4][4], U_inverse[4][4];
        auto matrix = Matrix(0);

        Float sum;

        memset(L, 0, sizeof(L));
        memset(U, 0, sizeof(U));
        memset(L_inverse, 0, sizeof(L_inverse));
        memset(U_inverse,0 , sizeof(U_inverse));

        for(auto i = 0;i < 4;i++){
            L[i][i] = 1;
        }

        for(auto i = 0;i < 4;i++){
            for(auto j = i;j < 4;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[i][k] * U[k][j];
                }

                U[i][j] = m.data[i][j] - sum;
            }

            for(auto j = i + 1; j < 4;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[j][k] * U[k][i];
                }

                if(U[i][i] == 0)
                    return matrix;

                L[j][i] = (m.data[j][i] - sum) / U[i][i];
            }
        }

        //LU inverse
        for(auto i = 0;i < 4;i++){
            L_inverse[i][i] = 1;
        }

        for (auto i = 1;i < 4;i++) {
            for(auto j = 0;j < i;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[i][k] * L_inverse[k][j];
                }

                L_inverse[i][j] = -sum;
            }
        }

        for(auto i = 0;i < 4;i++){
            U_inverse[i][i] = 1 / U[i][i];
        }

        for(auto i = 1;i < 4;i++){
            for(auto j = i -1; j >= 0;j--){
                sum = 0;
                for(auto k = j + 1;k <= i;k++){
                    sum += U[j][k] * U_inverse[k][i];
                }
                U_inverse[j][i] = -sum / U[j][j];
            }
        }

        for(auto i = 0;i < 4;i++){
            for(auto j = 0;j < 4;j++){
                for(auto k = 0;k < 4;k++){
                    matrix.data[i][j] += U_inverse[i][k] * L_inverse[k][j];
                }
            }
        }

        return matrix;
    }
}