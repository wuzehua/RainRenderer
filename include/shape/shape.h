//
// Created by RainZhong on 2019/12/1.
//

#ifndef RAINRENDERER_SHAPE_H
#define RAINRENDERER_SHAPE_H

#include <math/transform.h>

namespace rzpbr{
    class Shape{
    public:

        Shape(){}
        ~Shape(){}

        Shape(const Transform* o2w, const Transform* w2o): objectToWorld(o2w), worldToObject(w2o){}

        virtual Bound3f getObjectBound() const = 0;
        virtual Bound3f getWorldBound() const;



        const Transform* objectToWorld, *worldToObject;
    };
}

#endif //RAINRENDERER_SHAPE_H
