//
// Created by rainzhong on 2019/11/28.
//

#ifndef RAINRENDERER_TRANSFORM_H
#define RAINRENDERER_TRANSFORM_H

#include <cstring>
#include <math/vector.h>

namespace rzpbr{

    class Matrix{

    public:

        Matrix(Float scale = 1.0){
            for(auto i = 0;i < 4;i++){
                for(auto j = 0; j < 4;j++){
                    if(i == j){
                        data[i][j] = scale;
                    }else {
                        data[i][j] = 0;
                    }
                }
            }
        }




        friend Matrix operator+(const Matrix& m1, const Matrix& m2){
            auto m = Matrix();
            for(auto i = 0; i < 4;i++){
                for(auto j = 0;j < 4;j++){
                    m.data[i][j] = m1.data[i][j] + m2.data[i][j];
                }
            }
            return m;
        }

        friend Matrix operator-(const Matrix& m1, const Matrix& m2){
            auto m = Matrix();
            for(auto i = 0; i < 4;i++){
                for(auto j = 0;j < 4;j++){
                    m.data[i][j] = m1.data[i][j] - m2.data[i][j];
                }
            }
            return m;
        }

        friend Matrix operator*(const Matrix& m1, const Matrix& m2){
            auto m = Matrix(0.0);
            for(auto i = 0;i < 4;i++){
                for(auto j = 0;j < 4;j++){
                    for(auto k = 0;k < 4;k++){
                        m.data[i][j] += m1.data[i][k] * m2.data[k][j];
                    }
                }
            }
            return m;
        }

        friend std::ostream& operator<<(std::ostream& os, const Matrix& m){
            for(auto i = 0;i < 4;i++){
                os << "| ";
                for(auto j = 0;j < 4;j++){
                    os << m.data[i][j] << " ";
                }
                os <<"|\n";
            }
            return os;
        }

        static Matrix transpose(const Matrix& m);
        static Matrix inverse(const Matrix& m);



        void set(int i, int j, Float value){
            assert(i >= 0 && i < 4 && j >= 0 && j < 4);
            data[i][j] = value;
        }



    private:
        Float data[4][4];
    };
}

#endif //RAINRENDERER_TRANSFORM_H
