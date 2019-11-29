//
// Created by rainzhong on 2019/11/29.
//

#ifndef RAINRENDERER_RAY_H
#define RAINRENDERER_RAY_H

#include <memory>
#include <media/medium.h>
#include <math/vector.h>

namespace rzpbr {

    static constexpr Float Infinity = std::numeric_limits<Float>::infinity();

    class Ray {

    public:
        Ray(): tMax(Infinity),medium(nullptr),time(0.0){}
        Ray(const Point3f& origin, const Vector3f& direction, Float tMax = Infinity, Float time = 0.0, const Medium* medium = nullptr):
                origin(origin), direction(direction), tMax(tMax), time(time), medium(medium)
        {}

        Point3f operator()(Float t) const { return origin + t * direction;}

        friend std::ostream& operator<<(std::ostream& os, const Ray& ray){
            os<<"Ray( origin: "<<ray.origin<<", direction: "<<ray.direction<<", tMax:"<<ray.tMax<<", time: "<<ray.time<<") \n";
            return os;
        }

        Point3f origin;
        Vector3f direction;
        mutable Float tMax;
        Float time;
        std::shared_ptr<const Medium> medium;
    };

    class RayDifferential: public Ray{

    public:
        RayDifferential(): Ray(){ hasDifferential = false;}
        RayDifferential(const Point3f& origin, const Vector3f& direction, Float tMax = Infinity, Float time = 0.0, const Medium* medium = nullptr):
                Ray(origin, direction, tMax, time, medium)
        {
            hasDifferential = false;
        }
        RayDifferential(const Ray& ray): Ray(ray){
            hasDifferential = false;
        }

        void scaleDifferentials(Float scale){
            rxOrigin = origin + (rxOrigin - origin) * scale;
            ryOrigin = origin + (ryOrigin - origin) * scale;
            rxDirection = direction + (rxDirection - direction) * scale;
            ryDirection = direction + (ryDirection - direction) * scale;
        }

        bool hasDifferential;
        Point3f rxOrigin,ryOrigin;
        Vector3f rxDirection,ryDirection;
    };

}

#endif //RAINRENDERER_RAY_H
