//
// Created by rainzhong on 2019/11/29.
//

#ifndef RAINRENDERER_BOUND_H
#define RAINRENDERER_BOUND_H

#include <math/vector.h>
#include <utils/ray.h>
#include <cmath>

namespace rzpbr{


    template<typename T>
    class Bound2{
    public:
        Bound2(){
            T max = std::numeric_limits<T>::max();
            T min = std::numeric_limits<T>::lowest();
            minP = Vector2<T>(max,max);
            maxP = Vector2<T>(min,min);
        }

        Bound2(const Vector2<T>& p): minP(p),maxP(p){}
        Bound2(const Vector2<T>& p1, const Vector2<T>& p2){
            minP = Vector2<T>(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()));
            maxP = Vector2<T>(std::max(p1.x(),p2.x()), std::max(p1.y(), p2.y()));
        }

        template <typename U>
        explicit operator Bound2<U>() const {
            return Bound2<U>((Vector2<U>)minP,(Vector2<U>)maxP);
        }

        const Vector2<T>& operator[](int index) const {
            assert(index == 0 || index == 1);
            return (index == 0) ? minP : maxP;
        }

        Vector2<T>& operator[](int index){
            assert(index == 0 || index == 1);
            return (index == 0) ? minP : maxP;
        }

        bool operator==(const Bound2<T>& b) const {
            return minP == b.minP && maxP == b.maxP;
        }

        bool operator!=(const Bound2<T>& b) const {
            return minP != b.minP || maxP != b.maxP;
        }

        Vector2<T> diagonal(){
            return maxP - minP;
        }

        T area(){
            auto d = maxP - minP;
            return (d.x() * d.y());
        }


        int maxExtent() const {
            auto d = diagonal();
            return d.x > d.y ? 0 : 1;
        }

        Vector2<T> lerp(const Point2f& t) const {
            return Vector2<T>(rzpbr::lerp(t.x(), minP.x(), maxP.x()),
                    rzpbr::lerp(t.y(), minP.y(), maxP.y()));
        }

        Vector2<T> offset(const Vector2<T>& p) const {
            auto o = p - minP;
            if(maxP.x() > minP.x()) o[0] /= maxP.x() - minP.x();
            if(maxP.y() > minP.y()) o[1] /= maxP.y() - minP.y();
            return o;
        }

        void boundingSphere(Vector2<T>& c, Float& rad) const{
            c = (maxP + minP) / 2;
            rad = inside(c, *this) ? distance(c,maxP) : 0;
        }

        Vector2<T> minP,maxP;
    };




    template <typename T>
    class Bound3{
    public:
        Bound3(){
            T max = std::numeric_limits<T>::max();
            T min = std::numeric_limits<T>::lowest();
            minP = Vector3<T>(max,max,max);
            maxP = Vector3<T>(min,min,min);
        }

        Bound3(const Vector3<T>& p): minP(p),maxP(p){}
        Bound3(const Vector3<T>& p1, const Vector3<T>& p2){
            minP = Vector3<T>(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()), std::min(p1.z(), p2.z()));
            maxP = Vector3<T>(std::max(p1.x(),p2.x()), std::max(p1.y(), p2.y()), std::max(p1.z(), p2.z()));
        }

        template <typename U>
        explicit operator Bound3<U>() const {
            return Bound3<U>((Vector3<U>)minP,(Vector3<U>)maxP);
        }

        const Vector3<T>& operator[](int index) const {
            assert(index == 0 || index == 1);
            return (index == 0) ? minP : maxP;
        }

        Vector3<T>& operator[](int index) {
            assert(index == 0 || index == 1);
            return (index == 0) ? minP : maxP;
        }

        bool operator==(const Bound3<T>& b) const {
            return minP == b.minP && maxP == b.maxP;
        }

        bool operator!=(const Bound3<T>& b) const {
            return minP != b.minP || maxP != b.maxP;
        }

        Vector3<T> corner(int index) const {
            return Vector3<T>((*this)[index & 1].x(),
                    (*this)[(index & 2) ? 1 : 0].y(),
                    (*this)[(index & 4) ? 1 : 0].z());
        }

        Vector3<T> diagonal() const {
            return maxP - minP;
        }

        T surfaceArea() const {
            auto d = diagonal();
            return (d.x() * d.x() + d.y() * d.y() + d.z() * d.z());
        }

        T volume() const {
            auto d = diagonal();
            return d.x() * d.y() * d.z();
        }

        int maxExtent(){
            auto d = diagonal();
            if(d.x() > d.y() && d.x() > d.z())
                return 0;
            else if(d.y() > d.z())
                return 1;
            else
                return 2;
        }

        Vector3<T> lerp(const Point3f& t) const {
            return Vector3<T>(rzpbr::lerp(t.x(), minP.x(), maxP.x()),
                    rzpbr::lerp(t.y(),minP.y(),maxP.y()),
                    rzpbr::lerp(t.z(),minP.z(),maxP.z()));
        }


        Vector3<T> offset(const Vector3<T>& p) const {
            auto o = p - minP;
            if(maxP.x() > minP.x()) o[0] /= maxP.x() - minP.x();
            if(maxP.y() > minP.y()) o[1] /= maxP.y() - minP.y();
            if(maxP.z() > minP.z()) o[2] /= maxP.z() - minP.z();
            return o;
        }

        void boundingSphere(Vector3<T>& c, Float& rad) const{
            c = (maxP + minP) / 2;
            rad = inside(c, *this) ? distance(c,maxP) : 0;
        }

        bool insersectP(const Ray& ray, Float* hitt0 = nullptr, Float* hitt1 = nullptr) const{
        }

        inline bool insersectP(const Ray& ray, const Vector3f& invDir, const int dirIsNeg[3]) const{

        }


        Vector3<T> minP,maxP;
    };

    template <typename T>
    bool inside(const Vector2<T>& p, const Bound2<T>& b){
        return (p.x() >= b.minP.x() && p.x() <= b.maxP.x()
        && p.y() >= b.minP.y() && p.y() <= b.maxP.y());
    }

    template <typename T>
    bool inside(const Vector3<T>& p, const Bound3<T>& b){
        return (p.x() >= b.minP.x() && p.x() <= b.maxP.x()
                && p.y() >= b.minP.y() && p.y() <= b.maxP.y()
                &&p.z() >= b.minP.z() && p.z() <= b.maxP.z());
    }

    typedef Bound2<Float> Bound2f;
    typedef Bound2<int> Bound2i;
    typedef Bound3<Float> Bound3f;
    typedef Bound3<int> Bound3i;

    template <typename T>
    Bound3<T> unionBound(const Bound3<T>& b, const Vector3<T>& p){
        return Bound3<T>(
                Vector3<T>(std::min(b.minP.x(), p.x()),std::min(b.minP.y(), p.y()), std::min(b.minP.z(), p.z())),
                Vector3<T>(std::max(b.maxP.x(), p.x()),std::max(b.maxP.y(), p.y()), std::max(b.maxP.z(), p.z()))
                );
    }


}


#endif //RAINRENDERER_BOUND_H
