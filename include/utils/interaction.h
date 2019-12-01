//
// Created by RainZhong on 2019/12/1.
//

#ifndef RAINRENDERER_INTERACTION_H
#define RAINRENDERER_INTERACTION_H

#include <math/vector.h>
#include <utils/ray.h>
#include <shape/shape.h>
#include <media/medium.h>

namespace rzpbr{

    const Float epsilon = 1e-5;



    //TODO: Interaction
    struct Interaction{

        Interaction(): time(0){}
        Interaction(const Point3f& p, const Float& time, const Normal3f& n,
                    const Vector3f& wo, const Vector3f& pe, const MediumInterface& m):
                    point(p),time(time),normal(n),wo(wo),pError(pe), mediumInterface(m){}

        bool isSurfaceInteraction() const {
            return normal != Normal3f();
        }

        bool isMediumInteraction() const {
            return !isSurfaceInteraction();
        }

        //TODO:pError and the media(for refraction)
        Ray generateRay(const Vector3f& direction) const {
            return Ray(point, direction, Infinity, time);
        }

        Ray generateRayTo(const Interaction& i) const {
            auto direction = i.point - point;
            return Ray(point,direction, 1 - epsilon, time);
        }

        Ray generateRayTo(const Vector3f& p) const {
            auto direction = p - point;
            return Ray(point, direction, 1- epsilon, time);
        }

        const Medium* getMedium(const Vector3f& direction) const {
            return (Vector3f::dot(normal, direction) > 0) ? mediumInterface.outside : mediumInterface.inside;
        }

        const Medium* getMedium() const {
            assert(mediumInterface.inside == mediumInterface.outside);
            return mediumInterface.inside;
        }

        Point3f point;
        Float time;
        Normal3f normal;
        Vector3f wo;
        Vector3f pError;
        MediumInterface mediumInterface;
    };

    class SurfaceInteraction: public Interaction{

    public:



        Point2f uv;
        Normal3f dndu,dndv;
        Vector3f dpdu,dpdv;
        const Shape* shape = nullptr;
    };

}

#endif //RAINRENDERER_INTERACTION_H
