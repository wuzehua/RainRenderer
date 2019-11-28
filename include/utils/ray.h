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



        Point3f origin;
        Vector3f direction;
        mutable Float tMax;
        Float time;
        std::shared_ptr<const Medium> medium;
    };
}

#endif //RAINRENDERER_RAY_H
