//
// Created by RainZhong on 2019/12/1.
//

#ifndef RAINRENDERER_INTERACTION_H
#define RAINRENDERER_INTERACTION_H

#include <math/vector.h>

namespace rzpbr{

    struct MediumInterface{};

    //TODO: Interaction
    struct Interaction{

        Interaction(): time(0){}
        Interaction(const Point3f& p, const Float& time, const Normal3f& n,
                    const Vector3f& wo, const Vector3f& pe, const MediumInterface& m):
                    point(p),time(time),normal(n),wo(wo),pError(pe), mediumInterface(m){}




        Point3f point;
        Float time;
        Normal3f normal;
        Vector3f wo;
        Vector3f pError;
        MediumInterface mediumInterface;
    };
}

#endif //RAINRENDERER_INTERACTION_H
