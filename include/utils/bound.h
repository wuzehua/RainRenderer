//
// Created by rainzhong on 2019/11/29.
//

#ifndef RAINRENDERER_BOUND_H
#define RAINRENDERER_BOUND_H

#include <math/vector.h>

namespace rzpbr{

    template<typename T>
    class Bound2{
    public:
        Bound2(){
            T max = std::numeric_limits<T>::max();
            T min = std::numeric_limits<T>::lowest();
            minP = Point2<T>(min,min);
            maxP = Point2<T>(max,max);
        }

        Bound2(const Point2<T>& p): minP(p),maxP(p){}
        Bound2(const Point2<T>& p1, const Point2<T>& p2){
            minP = Point2<T>(std::min(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
            maxP = Point2<T>(std::max(p1.x(),p2.x()), std::max(p1.y(), p2.y()));
        }

        const Point2<T>& operator[](int index) const {
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




        Point2<T> minP,maxP;
    };

    class Bound3{
    public:

        Point3<T> minP,maxP;
    };
}


#endif //RAINRENDERER_BOUND_H
