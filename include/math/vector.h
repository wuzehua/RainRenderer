//
// Created by rainzhong on 2019/11/28.
//

#ifndef RAINRENDERER_VECTOR_H
#define RAINRENDERER_VECTOR_H

#include <iostream>
#include <cassert>
#include <cmath>

#ifdef FLOAT_64
    typedef double Float;
#else
    typedef float Float;
#endif

namespace rainr {

    template<typename T>
    class Vector2 {

    public:
        Vector2() { data[0] = data[1] = 0; }

        Vector2(const T &x, const T &y) {
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

        template <typename U>
        friend Vector2<T> operator*(const Vector2<T> &v1, const U& u) {
            return Vector2(v1.data[0] * u, v1.data[1] * u);
        }

        template <typename U>
        friend Vector2<T> operator*(const U& u, const Vector2<T> &v1) {
            return Vector2(v1.data[0] * u, v1.data[1] * u);
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


        Float lengthPow2(){ return data[0] * data[0] + data[1] * data[1];}
        Float length(){ return std::sqrt(lengthPow2());}


        const T &x() { return data[0]; }

        const T &y() { return data[1]; }

        void setX(const T &x) { data[0] = x; }

        void setY(const T &y) { data[1] = y; }


    private:
        T data[2];
    };
}

#endif //RAINRENDERER_VECTOR_H
