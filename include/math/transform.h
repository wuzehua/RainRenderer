//
// Created by rainzhong on 2019/11/28.
//

#ifndef RAINRENDERER_TRANSFORM_H
#define RAINRENDERER_TRANSFORM_H

#include <cstring>
#include <utils/bound.h>
#include <math/vector.h>

namespace rzpbr{

    struct Quaternion{

    };



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

        bool operator==(const Matrix& m) const {
            auto result = true;
            for(auto i = 0;i < 4;i++){
                for(auto j = 0; j < 4;j++){
                    if(data[i][j] != m.data[i][j]){
                        result = false;
                        break;
                    }
                }

                if(!result)
                    break;
            }

            return result;
        }

        bool operator!=(const Matrix& m) const {
            return !(*this == m);
        }


        void set(int i, int j, Float value){
            assert(i >= 0 && i < 4 && j >= 0 && j < 4);
            data[i][j] = value;
        }


        Float data[4][4];
    };

    Matrix transpose(const Matrix& m);
    Matrix inverse(const Matrix& m);

    class Transform{
    public:
        Transform(){}
        Transform(const Matrix& m):m(m), invM(inverse(m)){}
        Transform(const Matrix& m, const Matrix& invM):m(m),invM(invM){}

        friend Transform inverse(const Transform& t){
            return Transform(t.invM, t.m);
        }

        friend Transform transpose(const Transform& t){
            return Transform(transpose(t.m), transpose(t.invM));
        }


        bool operator==(const Transform& t) const {
            return m == t.m && invM == t.invM;
        }

        bool operator!=(const Transform& t) const {
            return !(*this == t);
        }

        friend Transform operator*(const Transform& t1, const Transform& t2){
            return Transform(t1.m * t2.m, t2.invM * t1.invM);
        }

        template <typename T>
        Vector3<T> transformPosition(const Vector3<T>& p) const {
            T v[4] = {0,0,0,0};
            for(auto i = 0;i < 4;i++){
                for(auto j = 0;j < 4;j++){
                    if(j == 3){
                        v[i] += m.data[i][j];
                    } else{
                        v[i] += m.data[i][j] * p[j];
                    }
                }
            }

            auto result = Vector3<T>(v[0],v[1],v[2]);
            if(v[3] != 0 && v[3] != 1)
                result /= v[3];

            return result;
        }


        template <typename T>
        Vector3<T> transformDirection(const Vector3<T>& d) const {
            T v[3] = {0,0,0};
            for(auto i = 0;i < 3;i++){
                for(auto j = 0;j < 3;j++){
                    v[i] += m.data[i][j] * d[j];
                }
            }

            return Vector3<T>(v[0],v[1],v[2]);
        }

        template <typename T>
        Vector3<T> transformNormal(const Vector3<T>& n) const {
            T v[3] = {0,0,0};
            for(auto i = 0;i < 3;i++){
                for(auto j = 0;j < 3;j++){
                    v[i] += invM.data[j][i] * n[j];
                }
            }

            return Vector3<T>(v[0],v[1],v[2]);
        }

        const Matrix& matrix(){ return m;}
        const Matrix& invTMatrix() { return invM;}


    private:
        Matrix m,invM;
        friend struct Quaternion;
        friend class AnimatedTransform;
    };

    Transform translate(const Vector3f& d);
    Transform scale(const Vector3f& s);
    Transform rotateX(const Float& theta);
    Transform rotateY(const Float& theta);
    Transform rotateZ(const Float& theta);
    Transform rotateAxis(const Float& theta, const Vector3f& axis);

    class AnimatedTransform{};

}

#endif //RAINRENDERER_TRANSFORM_H
