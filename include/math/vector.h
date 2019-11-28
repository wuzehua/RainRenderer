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
        Vector2() { data[0] = data[1] = 0; }

        Vector2(const T &x = 0, const T &y = 0) {
            data[0] = x;
            data[1] = y;
        }

        Vector2<T> &operator=(const Vector2<T> vector) {
            data[0] = vector.data[0];
            data[1] = vector.data[1];
            return *this;
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

        Float lengthPow2(){ return data[0] * data[0] + data[1] * data[1];}
        Float length(){ return std::sqrt(lengthPow2());}

        //getter
        const T &x() { return data[0]; }
        const T &y() { return data[1]; }

        //setter
        void setX(const T &x) { data[0] = x; }
        void setY(const T &y) { data[1] = y; }


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

        Vector3(){ data[0] = data[1] = data[2] = 0;}
        Vector3(const T& x = 0, const T& y = 0, const T& z = 0){
            data[0] = x;
            data[1] = y;
            data[2] = z;
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
            return vec.normalize();
        }

        static Vector3<T> cross(const Vector3<T>& a, const Vector3<T>& b){
            auto x = a.data[1] * b.data[2] - b.data[1] * a.data[2];
            auto y = a.data[2] * b.data[0] - a.data[0] * b.data[2];
            auto z = a.data[0] * b.data[1] - b.data[0] * a.data[1];
            return Vector3(x,y,z);
        }

        Float lengthPow2(){ return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];}
        Float length() { return std::sqrt(lengthPow2());}

        //getter
        const T& x(){ return data[0];}
        const T& y(){ return data[1];}
        const T& z(){ return data[3];}

        //setter
        void setX(const T& x){ data[0] = x;}
        void setY(const T& y){ data[1] = y;}
        void setZ(const T& z){ data[2] = z;}


    private:
        T data[3];
    };

    typedef Vector3<Float> Point3f;
    typedef Vector3<int> Point3i;
    typedef Vector3<Float> Vector3f;
    typedef Vector3<int> Vector3i;
    typedef Vector3<Float> Normal3f;
    typedef Vector3<int> Normal3i;

}

#endif //RAINRENDERER_VECTOR_H
