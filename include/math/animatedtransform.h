//
// Created by rainzhong on 2019/11/30.
//

#ifndef RAINRENDERER_ANIMATEDTRANSFORM_H
#define RAINRENDERER_ANIMATEDTRANSFORM_H

#include <math/quaternion.h>
#include <math/transform.h>

namespace rzpbr{

    // AnimatedTransform Declarations
    /*code from pbrt*/
    class AnimatedTransform {
    public:
        // AnimatedTransform Public Methods
        AnimatedTransform(const Transform *startTransform, Float startTime,
                          const Transform *endTransform, Float endTime);
        static void decompose(const Matrix &m, Vector3f *T, Quaternion *R,
                              Matrix *S);
        void interpolate(Float time, Transform *t) const;
        Ray operator()(const Ray &r) const;
        RayDifferential operator()(const RayDifferential &r) const;
        //Point3f operator()(Float time, const Point3f &p) const;
        //Vector3f operator()(Float time, const Vector3f &v) const;
        Point3f transformPosition(Float time, const Point3f &p) const;
        Vector3f transformDirection(Float time, const Point3f &v) const;


        Bound3f motionBounds(const Bound3f &b) const;
        Bound3f boundPointMotion(const Point3f &p) const;

    private:
        // AnimatedTransform Private Data
        const Transform *startTransform, *endTransform;
        const Float startTime, endTime;
        const bool actuallyAnimated;
        Vector3f T[2];
        Quaternion R[2];
        Matrix S[2];
        bool hasRotation;
        struct DerivativeTerm {
            DerivativeTerm() {}
            DerivativeTerm(Float c, Float x, Float y, Float z)
                    : kc(c), kx(x), ky(y), kz(z) {}
            Float kc, kx, ky, kz;
            Float eval(const Point3f &p) const {
                return kc + kx * p.x() + ky * p.y() + kz * p.z();
            }
        };
        DerivativeTerm c1[3], c2[3], c3[3], c4[3], c5[3];
    };
}

#endif //RAINRENDERER_ANIMATEDTRANSFORM_H
