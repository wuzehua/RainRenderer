//
// Created by rainzhong on 2019/11/28.
//

#ifndef RAINRENDERER_TRANSFORM_H
#define RAINRENDERER_TRANSFORM_H

#include <cstring>
#include <utils/bound.h>
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


        Bound3f operator()(const Bound3f& b) const {}

        bool operator==(const Transform& t) const {
            return m == t.m && invM == t.invM;
        }

        bool operator!=(const Transform& t) const {
            return !(*this == t);
        }

        friend Transform operator*(const Transform& t1, const Transform& t2){
            return Transform(t1.m * t2.m, t2.invM * t1.invM);
        }

        inline Ray operator()(const Ray& ray) const;

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
        inline Vector3<T> transformPosition(const Vector3<T>& p, Vector3<T>* error) const{

            /*code from pbrt*/
            T x = p.x(), y = p.y(), z = p.z();

            // Compute absolute error for transformed point
            T xAbsSum = (std::abs(m.data[0][0] * x) + std::abs(m.data[0][1] * y) +
                         std::abs(m.data[0][2] * z) + std::abs(m.data[0][3]));
            T yAbsSum = (std::abs(m.data[1][0] * x) + std::abs(m.data[1][1] * y) +
                         std::abs(m.data[1][2] * z) + std::abs(m.data[1][3]));
            T zAbsSum = (std::abs(m.data[2][0] * x) + std::abs(m.data[2][1] * y) +
                         std::abs(m.data[2][2] * z) + std::abs(m.data[2][3]));
            *error = gamma(3) * Vector3<T>(xAbsSum, yAbsSum, zAbsSum);

            return this->transformPosition(p);
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
    Transform lookAt(const Vector3f& pos, const Vector3f& look, const Vector3f& up);








    /*code from pbrt*/
    struct Quaternion{
        // Quaternion Public Methods
        Quaternion() : v(0, 0, 0), w(1) {}
        Quaternion &operator+=(const Quaternion &q) {
            v += q.v;
            w += q.w;
            return *this;
        }
        friend Quaternion operator+(const Quaternion &q1, const Quaternion &q2) {
            Quaternion ret = q1;
            return ret += q2;
        }
        Quaternion &operator-=(const Quaternion &q) {
            v -= q.v;
            w -= q.w;
            return *this;
        }
        Quaternion operator-() const {
            Quaternion ret;
            ret.v = -v;
            ret.w = -w;
            return ret;
        }
        friend Quaternion operator-(const Quaternion &q1, const Quaternion &q2) {
            Quaternion ret = q1;
            return ret -= q2;
        }
        Quaternion &operator*=(Float f) {
            v *= f;
            w *= f;
            return *this;
        }
        Quaternion operator*(Float f) const {
            Quaternion ret = *this;
            ret.v *= f;
            ret.w *= f;
            return ret;
        }
        Quaternion &operator/=(Float f) {
            v /= f;
            w /= f;
            return *this;
        }
        Quaternion operator/(Float f) const {
            Quaternion ret = *this;
            ret.v /= f;
            ret.w /= f;
            return ret;
        }
        Transform toTransform() const;
        Quaternion(const Transform &t);

        friend std::ostream &operator<<(std::ostream &os, const Quaternion &q) {
            os << "quan[ "<<q.v.x()<<", "<<q.v.y()<<", "<<q.v.z()<<", "<<q.w<<" ]";
            return os;
        }

        // Quaternion Public Data
        Vector3f v;
        Float w;
    };

    Quaternion slerp(Float t, const Quaternion &q1, const Quaternion &q2);

// Quaternion Inline Functions
    inline Quaternion operator*(Float f, const Quaternion &q) { return q * f; }

    inline Float dot(const Quaternion &q1, const Quaternion &q2) {
        return Vector3f::dot(q1.v, q2.v) + q1.w * q2.w;
    }

    inline Quaternion normalize(const Quaternion &q) {
        return q / std::sqrt(dot(q, q));
    }







    class Interval{
    public:
        Interval(Float v0, Float v1){}
    };


    void intervalFindZeros(Float c1, Float c2, Float c3, Float c4, Float c5,
                           Float theta, Interval tInterval, Float *zeros,
                           int *zeroCount, int depth = 8);







    // AnimatedTransform Declarations
    /*code from pbrt*/
    class AnimatedTransform {
    public:
        // AnimatedTransform Public Methods
        AnimatedTransform(const Transform *startTransform, Float startTime,
                          const Transform *endTransform, Float endTime);
        static void decompose(const Matrix &m, Vector3f *T, Quaternion *R,
                              Matrix *S);
        void interpolate(Float time, Transform *t) const;
        Ray operator()(const Ray &r) const;
        RayDifferential operator()(const RayDifferential &r) const;
        //Point3f operator()(Float time, const Point3f &p) const;
        //Vector3f operator()(Float time, const Vector3f &v) const;
        Point3f transformPosition(Float time, const Point3f &p) const;
        Vector3f transformDirection(Float time, const Point3f &v) const;


        Bound3f motionBounds(const Bound3f &b) const;
        Bound3f boundPointMotion(const Point3f &p) const;

    private:
        // AnimatedTransform Private Data
        const Transform *startTransform, *endTransform;
        const Float startTime, endTime;
        const bool actuallyAnimated;
        Vector3f T[2];
        Quaternion R[2];
        Matrix S[2];
        bool hasRotation;
        struct DerivativeTerm {
            DerivativeTerm() {}
            DerivativeTerm(Float c, Float x, Float y, Float z)
                    : kc(c), kx(x), ky(y), kz(z) {}
            Float kc, kx, ky, kz;
            Float eval(const Point3f &p) const {
                return kc + kx * p.x() + ky * p.y() + kz * p.z();
            }
        };
        DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
    };

}

#endif //RAINRENDERER_TRANSFORM_H
