//
// Created by rainzhong on 2019/11/28.
//

#ifndef RAINRENDERER_VECTOR_H
#define RAINRENDERER_VECTOR_H

#include <iostream>
#include <cassert>
#include <cmath>


namespace rzpbr {

#ifdef FLOAT_64
    typedef double Float;
#else
    typedef float Float;
#endif


    template<typename T>
    class Vector2 {

    public:
        //Vector2() { data[0] = data[1] = 0; }

        Vector2(const T &x = 0, const T &y = 0) {
            data[0] = x;
            data[1] = y;
        }

        Vector2<T> &operator=(const Vector2<T> vector) {
            data[0] = vector.data[0];
            data[1] = vector.data[1];
            return *this;
        }


        template <typename U>
        explicit operator Vector2<U>() const {
            return Vector2<U>((U)data[0],(U)data[1]);
        }

        friend Vector2<T> operator+(const Vector2<T> &v1, const Vector2<T> &v2) {
            return Vector2(v1.data[0] + v2.data[0], v1.data[1] + v2.data[1]);
        }

        friend Vector2<T> operator-(const Vector2<T> &v1, const Vector2<T> &v2) {
            return Vector2(v1.data[0] - v2.data[0], v1.data[1] - v2.data[1]);
        }

        friend Vector2<T> operator*(const Vector2<T> &v1, const Vector2<T> &v2) {
            return Vector2(v1.data[0] * v2.data[0], v1.data[1] * v2.data[1]);
        }


        friend Vector2<T> operator*(const Vector2<T> &v1, const T& u) {
            return Vector2(v1.data[0] * u, v1.data[1] * u);
        }


        friend Vector2<T> operator*(const T& u, const Vector2<T> &v1) {
            return Vector2(v1.data[0] * u, v1.data[1] * u);
        }


        friend Vector2<T> operator/(const Vector2<T>& v, const T& u){
            assert(u != 0);
            return Vector2(v.data[0] / u, v.data[1] / u);
        }

        Vector2<T>& operator+=(const Vector2<T>& v){
            data[0] += v.data[0];
            data[1] += v.data[1];
            return *this;
        }

        Vector2<T>& operator-=(const Vector2<T>& v){
            data[0] -= v.data[0];
            data[1] -= v.data[1];
            return *this;
        }

        Vector2<T>& operator*=(const Vector2<T>& v){
            data[0] *= v.data[0];
            data[1] *= v.data[1];
            return *this;
        }

        Vector2<T>& operator-(){
            return Vector2(-data[0], -data[1]);
        }


        Vector2<T>& operator*=(const T& u){
            data[0] *= u;
            data[1] *= u;
            return *this;
        }


        Vector2<T>& operator/=(const T& u){
            assert(u != 0);
            data[0] /= u;
            data[1] /= u;
            return *this;
        }


        bool operator==(const Vector2<T>& v) const {
            return data[0] == v.data[0] && data[1] == v.data[1];
        }

        bool operator!=(const Vector2<T>& v) const {
            return data[0] != v.data[0] || data[1] != v.data[1];
        }

        friend std::ostream &operator<<(std::ostream &os, const Vector2<T>& vector){
            os << "vec2(x: " << vector.data[0] << ", y: " << vector.data[1] << ")";
            return os;
        }


        T &operator[](int index) {
            assert(index >= 0 && index < 2);
            return data[index];
        }


        static T dot(const Vector2<T>& v1, const Vector2<T>& v2){
            return v1.data[0] * v2.data[0] + v1.data[1] * v2.data[1];
        }

        void normalize(){
            Float length = this->length();
            if(length == 0)
                return;
            data[0] /= length;
            data[1] /= length;
        }

        static Vector2<T> normalize(const Vector2<T>& v){
            auto result = v;
            return result.normalize();
        }

        Float lengthPow2() const { return data[0] * data[0] + data[1] * data[1];}
        Float length() const { return std::sqrt(lengthPow2());}

        //getter
        inline const T &x() const { return data[0]; }
        inline const T &y() const { return data[1]; }

        //setter
        inline void setX(const T &x) { data[0] = x; }
        inline void setY(const T &y) { data[1] = y; }


    private:
        T data[2];
    };


    typedef Vector2<Float> Point2f;
    typedef Vector2<int> Point2i;
    typedef Vector2<Float> Vector2f;
    typedef Vector2<int> Vector2i;


    template <typename T>
    class Vector3{

    public:

        //Vector3(){ data[0] = data[1] = data[2] = 0;}
        Vector3(const T& x = 0, const T& y = 0, const T& z = 0){
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }

        template <typename U>
        explicit operator Vector3<U>() const {
            return Vector3<U>((U)data[0],(U)data[1],(U)data[2]);
        }

        friend Vector3<T> operator+(const Vector3<T>& v1, const Vector3<T>& v2){
            return Vector3(v1.data[0] + v2.data[0], v1.data[1] + v2.data[1], v1.data[2] + v2.data[2]);
        }

        friend Vector3<T> operator-(const Vector3<T>& v1, const Vector3<T>& v2){
            return Vector3(v1.data[0] - v2.data[0], v1.data[1] - v2.data[1], v1.data[2] - v2.data[2]);
        }

        friend Vector3<T> operator*(const Vector3<T>& v1, const Vector3<T>& v2){
            return Vector3(v1.data[0] * v2.data[0], v1.data[1] * v2.data[1], v1.data[2] * v2.data[2]);
        }


        friend Vector3<T> operator*(const Vector3<T>& v1, const T& u){
            return Vector3(v1.data[0] * u, v1.data[1] * u, v1.data[2] * u);
        }


        friend Vector3<T> operator*(const T& u, const Vector3<T>& v1){
            return Vector3(v1.data[0] * u, v1.data[1] * u, v1.data[2] * u);
        }


        friend Vector3<T> operator/(const Vector3<T>& v1, const T& u){
            assert(u != 0);
            return Vector3(v1.data[0] / u, v1.data[1] / u, v1.data[2] / u);
        }


        Vector3<T>& operator+=(const Vector3<T>& v){
            data[0] += v.data[0];
            data[1] += v.data[1];
            data[2] += v.data[2];
            return *this;
        }

        Vector3<T>& operator-=(const Vector3<T>& v){
            data[0] -= v.data[0];
            data[1] -= v.data[1];
            data[2] -= v.data[2];
            return *this;
        }

        Vector3<T>& operator*=(const Vector3<T>& v){
            data[0] *= v.data[0];
            data[1] *= v.data[1];
            data[2] *= v.data[2];
            return *this;
        }

        Vector3<T>& operator-() const {
            return Vector3(-data[0], -data[1], -data[2]);
        }


        Vector3<T>& operator*=(const T& u){
            data[0] *= u;
            data[1] *= u;
            data[2] *= u;
            return *this;
        }


        Vector3<T>& operator/=(const T& u){
            assert(u != 0);
            data[0] /= u;
            data[1] /= u;
            data[2] /= u;
            return *this;
        }


        bool operator==(const Vector3<T>& v) const {
            return data[0] == v.data[0] && data[1] == v.data[1] && data[2] == v.data[2];
        }

        bool operator!=(const Vector3<T>& v) const {
            return data[0] != v.data[0] || data[1] != v.data[1] || data[2] != v.data[2];
        }


        friend std::ostream &operator<<(std::ostream &os, const Vector3<T>& vector){
            os << "vec3(x: " << vector.data[0] << ", y: " << vector.data[1] <<", z: "<<vector.data[2] <<")";
            return os;
        }

        const T& operator[](int index) const {
            assert(index >= 0 && index < 3);
            return data[index];
        }

        T& operator[](int index){
            assert(index >= 0 && index < 3);
            return data[index];
        }


        friend T dot(const Vector3<T>& v1, const Vector3<T>& v2){
            return v1.data[0] * v2.data[0] + v1.data[1] * v2.data[1] + v1.data[2] * v2.data[2];
        }


        void normalize(){
            auto length = this->length();
            if(length == 0)
                return;
            data[0] /= length;
            data[1] /= length;
            data[2] /= length;
        }

        static Vector3<T> normalize(const Vector3<T>& v){
            auto vec = v;
            vec.normalize();
            return vec;
        }

        static Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b){
            auto x = a.data[1] * b.data[2] - b.data[1] * a.data[2];
            auto y = a.data[2] * b.data[0] - a.data[0] * b.data[2];
            auto z = a.data[0] * b.data[1] - b.data[0] * a.data[1];
            return Vector3(x,y,z);
        }

        Float lengthPow2() const { return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];}
        Float length() const { return std::sqrt(lengthPow2());}

        //getter
        inline const T& x() const { return data[0];}
        inline const T& y() const { return data[1];}
        inline const T& z() const { return data[3];}

        //setter
        inline void setX(const T& x){ data[0] = x;}
        inline void setY(const T& y){ data[1] = y;}
        inline void setZ(const T& z){ data[2] = z;}


    private:
        T data[3];
    };

    typedef Vector3<Float> Point3f;
    typedef Vector3<int> Point3i;
    typedef Vector3<Float> Vector3f;
    typedef Vector3<int> Vector3i;
    typedef Vector3<Float> Normal3f;
    typedef Vector3<int> Normal3i;

    Float lerp(const Float& t, const Float& v0, const Float& v1);

    template <typename T>
    Vector2<T> lerp(const Float& t, const Vector2<T>& p0, const Vector2<T>& p1){
        return (1 - t) * p0 + t * p1;
    }

    template <typename T>
    Vector3<T> lerp(const Float& t, const Vector3<T>& p0, const Vector3<T>& p1){
        return (1 - t) * p0 + t * p1;
    }

    template <typename T>
    T distance(const Vector2<T>& v0, const Vector2<T>& v1){
        auto l = v0 - v1;
        return l.length();
    }

    template <typename T>
    T distance(const Vector3<T>& v0, const Vector3<T>& v1){
        auto l = v0 - v1;
        return l.length();
    }

}

#endif //RAINRENDERER_VECTOR_H
