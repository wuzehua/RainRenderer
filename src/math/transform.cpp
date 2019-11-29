//
// Created by rainzhong on 2019/11/28.
//

#include <math/transform.h>
#include <cstring>

namespace rzpbr{


    Matrix transpose(const Matrix& m){
        auto matrix = Matrix(0);
        for(auto i = 0;i < 4;i++){
            for(auto j = 0;j < 4;j++){
                matrix.data[i][j] = m.data[j][i];
            }
        }
        return matrix;
    }

    Matrix inverse(const Matrix& m){
        //LU
        Float L[4][4],U[4][4];
        Float L_inverse[4][4], U_inverse[4][4];
        auto matrix = Matrix(0);

        Float sum;

        memset(L, 0, sizeof(L));
        memset(U, 0, sizeof(U));
        memset(L_inverse, 0, sizeof(L_inverse));
        memset(U_inverse,0 , sizeof(U_inverse));

        for(auto i = 0;i < 4;i++){
            L[i][i] = 1;
        }

        for(auto i = 0;i < 4;i++){
            for(auto j = i;j < 4;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[i][k] * U[k][j];
                }

                U[i][j] = m.data[i][j] - sum;
            }

            for(auto j = i + 1; j < 4;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[j][k] * U[k][i];
                }

                if(U[i][i] == 0)
                    return matrix;

                L[j][i] = (m.data[j][i] - sum) / U[i][i];
            }
        }

        //LU inverse
        for(auto i = 0;i < 4;i++){
            L_inverse[i][i] = 1;
        }

        for (auto i = 1;i < 4;i++) {
            for(auto j = 0;j < i;j++){
                sum = 0;
                for(auto k = 0;k < i;k++){
                    sum += L[i][k] * L_inverse[k][j];
                }

                L_inverse[i][j] = -sum;
            }
        }

        for(auto i = 0;i < 4;i++){
            U_inverse[i][i] = 1 / U[i][i];
        }

        for(auto i = 1;i < 4;i++){
            for(auto j = i -1; j >= 0;j--){
                sum = 0;
                for(auto k = j + 1;k <= i;k++){
                    sum += U[j][k] * U_inverse[k][i];
                }
                U_inverse[j][i] = -sum / U[j][j];
            }
        }

        for(auto i = 0;i < 4;i++){
            for(auto j = 0;j < 4;j++){
                for(auto k = 0;k < 4;k++){
                    matrix.data[i][j] += U_inverse[i][k] * L_inverse[k][j];
                }
            }
        }

        return matrix;
    }


    Ray Transform::operator()(const rzpbr::Ray &ray) const{
        Vector3f error;
        auto o = transformPosition(ray.origin, &error);
        auto dir = transformDirection(ray.direction);

        auto lensq = dir.lengthPow2();
        auto tMax = ray.tMax;

        if(lensq > 0){
            auto dt = Vector3f::dot(abs(dir), error) / lensq;
            o += dir * dt;
            tMax -= dt;
        }

        return Ray(o,dir,tMax,ray.time,ray.medium);
    }

    Transform translate(const Vector3f& d){
        Matrix m(1.0);
        auto invM = Matrix(1.0);
        for(auto i = 0;i < 3;i++){
            m.data[i][3] = d[i];
            invM.data[i][3] = -d[i];
        }

        return Transform(m,invM);
    }

    Transform scale(const Vector3f& s){
        Matrix m(1.0);
        auto invM = Matrix(1.0);
        for(auto i = 0;i < 3;i++){
            assert(s[i] != 0);
            m.data[i][i] = s[i];
            invM.data[i][i] = 1 / s[i];
        }

        return Transform(m,invM);
    }

    Transform rotateX(const Float& theta){
        Matrix m(1.0);
        auto sint = std::sin(theta);
        auto cost = std::cos(theta);
        m.data[1][1] = cost;
        m.data[1][2] = -sint;
        m.data[2][1] = sint;
        m.data[2][2] = cost;

        return Transform(m, transpose(m));
    }

    Transform rotateY(const Float& theta){
        Matrix m(1.0);
        auto sint = std::sin(theta);
        auto cost = std::cos(theta);
        m.data[0][0] = cost;
        m.data[2][0] = -sint;
        m.data[0][2] = sint;
        m.data[2][2] = cost;

        return Transform(m, transpose(m));
    }

    Transform rotateZ(const Float& theta){
        Matrix m(1.0);
        auto sint = std::sin(theta);
        auto cost = std::cos(theta);
        m.data[0][0] = cost;
        m.data[0][1] = -sint;
        m.data[1][0] = sint;
        m.data[1][1] = cost;

        return Transform(m, transpose(m));
    }

    Transform rotateAxis(const Float& theta, const Vector3f& axis){
        auto a = Vector3f::normalize(axis);
        auto sint = std::sin(theta);
        auto cost = std::cos(theta);
        Matrix m(1.0);

        m.data[0][0] = a.x() * a.x() + (1 - a.x() * a.x()) * cost;
        m.data[0][1] = a.x() * a.y() * (1 - cost) - a.z() * sint;
        m.data[0][2] = a.x() * a.z() * (1 - cost) + a.y() * sint;
        m.data[0][3] = 0;


        m.data[1][0] = a.x() * a.y() * (1 - cost) + a.z() * sint;
        m.data[1][1] = a.y() * a.y() + (1 - a.y() * a.y()) * cost;
        m.data[1][2] = a.y() * a.z() * (1 - cost) - a.x() * sint;
        m.data[1][3] = 0;

        m.data[2][0] = a.x() * a.z() * (1 - cost) - a.y() * sint;
        m.data[2][1] = a.y() * a.z() * (1 - cost) + a.x() * sint;
        m.data[2][2] = a.z() * a.z() + (1 - a.z() * a.z()) * cost;
        m.data[2][3] = 0;

        return Transform(m, transpose(m));

    }

    Transform lookAt(const Vector3f& pos, const Vector3f& look, const Vector3f& up){
        Matrix m(1.0);
        auto dir = Vector3f::normalize(look - pos);
        auto right = Vector3f::normalize(Vector3f::cross(dir,up));
        auto newUp = Vector3f::normalize(Vector3f::cross(right,dir));

        for(auto i = 0;i < 3;i++){
            m.data[i][3] = pos[i];
            m.data[i][2] = dir[i];
            m.data[i][1] = newUp[i];
            m.data[i][0] = right[i];
        }

        return Transform(inverse(m),m);
    }



    /*code from pbrt*/
    Transform Quaternion::toTransform() const {
        Float xx = v.x() * v.x(), yy = v.y() * v.y(), zz = v.z() * v.z();
        Float xy = v.x() * v.y(), xz = v.x() * v.z(), yz = v.y() * v.z();
        Float wx = v.x() * w, wy = v.y() * w, wz = v.z() * w;

        Matrix m;
        m.data[0][0] = 1 - 2 * (yy + zz);
        m.data[0][1] = 2 * (xy + wz);
        m.data[0][2] = 2 * (xz - wy);
        m.data[1][0] = 2 * (xy - wz);
        m.data[1][1] = 1 - 2 * (xx + zz);
        m.data[1][2] = 2 * (yz + wx);
        m.data[2][0] = 2 * (xz + wy);
        m.data[2][1] = 2 * (yz - wx);
        m.data[2][2] = 1 - 2 * (xx + yy);

        // Transpose since we are left-handed.  Ugh.
        return Transform(transpose(m), m);
    }

    Quaternion::Quaternion(const Transform &t) {
        const Matrix &m = t.m;
        Float trace = m.data[0][0] + m.data[1][1] + m.data[2][2];
        if (trace > 0.f) {
            // Compute w from matrix trace, then xyz
            // 4w^2 = m[0][0] + m[1][1] + m[2][2] + m[3][3] (but m[3][3] == 1)
            Float s = std::sqrt(trace + 1.0f);
            w = s / 2.0f;
            s = 0.5f / s;
            v[0] = (m.data[2][1] - m.data[1][2]) * s;
            v[1] = (m.data[0][2] - m.data[2][0]) * s;
            v[2] = (m.data[1][0] - m.data[0][1]) * s;
        } else {
            // Compute largest of $x$, $y$, or $z$, then remaining components
            const int nxt[3] = {1, 2, 0};
            Float q[3];
            int i = 0;
            if (m.data[1][1] > m.data[0][0]) i = 1;
            if (m.data[2][2] > m.data[i][i]) i = 2;
            int j = nxt[i];
            int k = nxt[j];
            Float s = std::sqrt((m.data[i][i] - (m.data[j][j] + m.data[k][k])) + 1.0f);
            q[i] = s * 0.5f;
            if (s != 0.f) s = 0.5f / s;
            w = (m.data[k][j] - m.data[j][k]) * s;
            q[j] = (m.data[j][i] + m.data[i][j]) * s;
            q[k] = (m.data[k][i] + m.data[i][k]) * s;
            v[0] = q[0];
            v[1] = q[1];
            v[2] = q[2];
        }
    }

    Quaternion slerp(Float t, const Quaternion &q1, const Quaternion &q2) {
        Float cosTheta = dot(q1, q2);
        if (cosTheta > .9995f)
            return normalize((1 - t) * q1 + t * q2);
        else {
            Float lower = -1;
            Float upper = 1;
            Float theta = std::acos(std::clamp(cosTheta,lower,upper));
            Float thetap = theta * t;
            Quaternion qperp = normalize(q2 - q1 * cosTheta);
            return q1 * std::cos(thetap) + qperp * std::sin(thetap);
        }
    }

    /*code from pbrt*/
    AnimatedTransform::AnimatedTransform(const Transform *startTransform,
                                         Float startTime,
                                         const Transform *endTransform,
                                         Float endTime)
            : startTransform(startTransform),
              endTransform(endTransform),
              startTime(startTime),
              endTime(endTime),
              actuallyAnimated(*startTransform != *endTransform) {
        if (!actuallyAnimated)
            return;
        decompose(startTransform->m, &T[0], &R[0], &S[0]);
        decompose(endTransform->m, &T[1], &R[1], &S[1]);
        // Flip _R[1]_ if needed to select shortest path
        if (dot(R[0], R[1]) < 0) R[1] = -R[1];
        hasRotation =dot(R[0], R[1]) < 0.9995f;
        // Compute terms of motion derivative function
        if (hasRotation) {
            Float cosTheta = dot(R[0], R[1]);
            Float lower = -1;
            Float upper = 1;
            Float theta = std::acos(std::clamp(cosTheta, lower, upper));
            Quaternion qperp = normalize(R[1] - R[0] * cosTheta);

            Float t0x = T[0].x();
            Float t0y = T[0].y();
            Float t0z = T[0].z();
            Float t1x = T[1].x();
            Float t1y = T[1].y();
            Float t1z = T[1].z();
            Float q0x = R[0].v.x();
            Float q0y = R[0].v.y();
            Float q0z = R[0].v.z();
            Float q0w = R[0].w;
            Float qperpx = qperp.v.x();
            Float qperpy = qperp.v.y();
            Float qperpz = qperp.v.z();
            Float qperpw = qperp.w;
            Float s000 = S[0].data[0][0];
            Float s001 = S[0].data[0][1];
            Float s002 = S[0].data[0][2];
            Float s010 = S[0].data[1][0];
            Float s011 = S[0].data[1][1];
            Float s012 = S[0].data[1][2];
            Float s020 = S[0].data[2][0];
            Float s021 = S[0].data[2][1];
            Float s022 = S[0].data[2][2];
            Float s100 = S[1].data[0][0];
            Float s101 = S[1].data[0][1];
            Float s102 = S[1].data[0][2];
            Float s110 = S[1].data[1][0];
            Float s111 = S[1].data[1][1];
            Float s112 = S[1].data[1][2];
            Float s120 = S[1].data[2][0];
            Float s121 = S[1].data[2][1];
            Float s122 = S[1].data[2][2];

            c1[0] = DerivativeTerm(
                    -t0x + t1x,
                    (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s000 +
                    q0w * q0z * s010 - qperpx * qperpy * s010 +
                    qperpw * qperpz * s010 - q0w * q0y * s020 -
                    qperpw * qperpy * s020 - qperpx * qperpz * s020 + s100 -
                    q0y * q0y * s100 - q0z * q0z * s100 - qperpy * qperpy * s100 -
                    qperpz * qperpz * s100 - q0w * q0z * s110 +
                    qperpx * qperpy * s110 - qperpw * qperpz * s110 +
                    q0w * q0y * s120 + qperpw * qperpy * s120 +
                    qperpx * qperpz * s120 +
                    q0x * (-(q0y * s010) - q0z * s020 + q0y * s110 + q0z * s120),
                    (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s001 +
                    q0w * q0z * s011 - qperpx * qperpy * s011 +
                    qperpw * qperpz * s011 - q0w * q0y * s021 -
                    qperpw * qperpy * s021 - qperpx * qperpz * s021 + s101 -
                    q0y * q0y * s101 - q0z * q0z * s101 - qperpy * qperpy * s101 -
                    qperpz * qperpz * s101 - q0w * q0z * s111 +
                    qperpx * qperpy * s111 - qperpw * qperpz * s111 +
                    q0w * q0y * s121 + qperpw * qperpy * s121 +
                    qperpx * qperpz * s121 +
                    q0x * (-(q0y * s011) - q0z * s021 + q0y * s111 + q0z * s121),
                    (-1 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s002 +
                    q0w * q0z * s012 - qperpx * qperpy * s012 +
                    qperpw * qperpz * s012 - q0w * q0y * s022 -
                    qperpw * qperpy * s022 - qperpx * qperpz * s022 + s102 -
                    q0y * q0y * s102 - q0z * q0z * s102 - qperpy * qperpy * s102 -
                    qperpz * qperpz * s102 - q0w * q0z * s112 +
                    qperpx * qperpy * s112 - qperpw * qperpz * s112 +
                    q0w * q0y * s122 + qperpw * qperpy * s122 +
                    qperpx * qperpz * s122 +
                    q0x * (-(q0y * s012) - q0z * s022 + q0y * s112 + q0z * s122));

            c2[0] = DerivativeTerm(
                    0.,
                    -(qperpy * qperpy * s000) - qperpz * qperpz * s000 +
                    qperpx * qperpy * s010 - qperpw * qperpz * s010 +
                    qperpw * qperpy * s020 + qperpx * qperpz * s020 +
                    q0y * q0y * (s000 - s100) + q0z * q0z * (s000 - s100) +
                    qperpy * qperpy * s100 + qperpz * qperpz * s100 -
                    qperpx * qperpy * s110 + qperpw * qperpz * s110 -
                    qperpw * qperpy * s120 - qperpx * qperpz * s120 +
                    2 * q0x * qperpy * s010 * theta -
                    2 * q0w * qperpz * s010 * theta +
                    2 * q0w * qperpy * s020 * theta +
                    2 * q0x * qperpz * s020 * theta +
                    q0y *
                    (q0x * (-s010 + s110) + q0w * (-s020 + s120) +
                     2 * (-2 * qperpy * s000 + qperpx * s010 + qperpw * s020) *
                     theta) +
                    q0z * (q0w * (s010 - s110) + q0x * (-s020 + s120) -
                           2 * (2 * qperpz * s000 + qperpw * s010 - qperpx * s020) *
                           theta),
                    -(qperpy * qperpy * s001) - qperpz * qperpz * s001 +
                    qperpx * qperpy * s011 - qperpw * qperpz * s011 +
                    qperpw * qperpy * s021 + qperpx * qperpz * s021 +
                    q0y * q0y * (s001 - s101) + q0z * q0z * (s001 - s101) +
                    qperpy * qperpy * s101 + qperpz * qperpz * s101 -
                    qperpx * qperpy * s111 + qperpw * qperpz * s111 -
                    qperpw * qperpy * s121 - qperpx * qperpz * s121 +
                    2 * q0x * qperpy * s011 * theta -
                    2 * q0w * qperpz * s011 * theta +
                    2 * q0w * qperpy * s021 * theta +
                    2 * q0x * qperpz * s021 * theta +
                    q0y *
                    (q0x * (-s011 + s111) + q0w * (-s021 + s121) +
                     2 * (-2 * qperpy * s001 + qperpx * s011 + qperpw * s021) *
                     theta) +
                    q0z * (q0w * (s011 - s111) + q0x * (-s021 + s121) -
                           2 * (2 * qperpz * s001 + qperpw * s011 - qperpx * s021) *
                           theta),
                    -(qperpy * qperpy * s002) - qperpz * qperpz * s002 +
                    qperpx * qperpy * s012 - qperpw * qperpz * s012 +
                    qperpw * qperpy * s022 + qperpx * qperpz * s022 +
                    q0y * q0y * (s002 - s102) + q0z * q0z * (s002 - s102) +
                    qperpy * qperpy * s102 + qperpz * qperpz * s102 -
                    qperpx * qperpy * s112 + qperpw * qperpz * s112 -
                    qperpw * qperpy * s122 - qperpx * qperpz * s122 +
                    2 * q0x * qperpy * s012 * theta -
                    2 * q0w * qperpz * s012 * theta +
                    2 * q0w * qperpy * s022 * theta +
                    2 * q0x * qperpz * s022 * theta +
                    q0y *
                    (q0x * (-s012 + s112) + q0w * (-s022 + s122) +
                     2 * (-2 * qperpy * s002 + qperpx * s012 + qperpw * s022) *
                     theta) +
                    q0z * (q0w * (s012 - s112) + q0x * (-s022 + s122) -
                           2 * (2 * qperpz * s002 + qperpw * s012 - qperpx * s022) *
                           theta));

            c3[0] = DerivativeTerm(
                    0.,
                    -2 * (q0x * qperpy * s010 - q0w * qperpz * s010 +
                          q0w * qperpy * s020 + q0x * qperpz * s020 -
                          q0x * qperpy * s110 + q0w * qperpz * s110 -
                          q0w * qperpy * s120 - q0x * qperpz * s120 +
                          q0y * (-2 * qperpy * s000 + qperpx * s010 + qperpw * s020 +
                                 2 * qperpy * s100 - qperpx * s110 - qperpw * s120) +
                          q0z * (-2 * qperpz * s000 - qperpw * s010 + qperpx * s020 +
                                 2 * qperpz * s100 + qperpw * s110 - qperpx * s120)) *
                    theta,
                    -2 * (q0x * qperpy * s011 - q0w * qperpz * s011 +
                          q0w * qperpy * s021 + q0x * qperpz * s021 -
                          q0x * qperpy * s111 + q0w * qperpz * s111 -
                          q0w * qperpy * s121 - q0x * qperpz * s121 +
                          q0y * (-2 * qperpy * s001 + qperpx * s011 + qperpw * s021 +
                                 2 * qperpy * s101 - qperpx * s111 - qperpw * s121) +
                          q0z * (-2 * qperpz * s001 - qperpw * s011 + qperpx * s021 +
                                 2 * qperpz * s101 + qperpw * s111 - qperpx * s121)) *
                    theta,
                    -2 * (q0x * qperpy * s012 - q0w * qperpz * s012 +
                          q0w * qperpy * s022 + q0x * qperpz * s022 -
                          q0x * qperpy * s112 + q0w * qperpz * s112 -
                          q0w * qperpy * s122 - q0x * qperpz * s122 +
                          q0y * (-2 * qperpy * s002 + qperpx * s012 + qperpw * s022 +
                                 2 * qperpy * s102 - qperpx * s112 - qperpw * s122) +
                          q0z * (-2 * qperpz * s002 - qperpw * s012 + qperpx * s022 +
                                 2 * qperpz * s102 + qperpw * s112 - qperpx * s122)) *
                    theta);

            c4[0] = DerivativeTerm(
                    0.,
                    -(q0x * qperpy * s010) + q0w * qperpz * s010 - q0w * qperpy * s020 -
                    q0x * qperpz * s020 + q0x * qperpy * s110 -
                    q0w * qperpz * s110 + q0w * qperpy * s120 +
                    q0x * qperpz * s120 + 2 * q0y * q0y * s000 * theta +
                    2 * q0z * q0z * s000 * theta -
                    2 * qperpy * qperpy * s000 * theta -
                    2 * qperpz * qperpz * s000 * theta +
                    2 * qperpx * qperpy * s010 * theta -
                    2 * qperpw * qperpz * s010 * theta +
                    2 * qperpw * qperpy * s020 * theta +
                    2 * qperpx * qperpz * s020 * theta +
                    q0y * (-(qperpx * s010) - qperpw * s020 +
                           2 * qperpy * (s000 - s100) + qperpx * s110 +
                           qperpw * s120 - 2 * q0x * s010 * theta -
                           2 * q0w * s020 * theta) +
                    q0z * (2 * qperpz * s000 + qperpw * s010 - qperpx * s020 -
                           2 * qperpz * s100 - qperpw * s110 + qperpx * s120 +
                           2 * q0w * s010 * theta - 2 * q0x * s020 * theta),
                    -(q0x * qperpy * s011) + q0w * qperpz * s011 - q0w * qperpy * s021 -
                    q0x * qperpz * s021 + q0x * qperpy * s111 -
                    q0w * qperpz * s111 + q0w * qperpy * s121 +
                    q0x * qperpz * s121 + 2 * q0y * q0y * s001 * theta +
                    2 * q0z * q0z * s001 * theta -
                    2 * qperpy * qperpy * s001 * theta -
                    2 * qperpz * qperpz * s001 * theta +
                    2 * qperpx * qperpy * s011 * theta -
                    2 * qperpw * qperpz * s011 * theta +
                    2 * qperpw * qperpy * s021 * theta +
                    2 * qperpx * qperpz * s021 * theta +
                    q0y * (-(qperpx * s011) - qperpw * s021 +
                           2 * qperpy * (s001 - s101) + qperpx * s111 +
                           qperpw * s121 - 2 * q0x * s011 * theta -
                           2 * q0w * s021 * theta) +
                    q0z * (2 * qperpz * s001 + qperpw * s011 - qperpx * s021 -
                           2 * qperpz * s101 - qperpw * s111 + qperpx * s121 +
                           2 * q0w * s011 * theta - 2 * q0x * s021 * theta),
                    -(q0x * qperpy * s012) + q0w * qperpz * s012 - q0w * qperpy * s022 -
                    q0x * qperpz * s022 + q0x * qperpy * s112 -
                    q0w * qperpz * s112 + q0w * qperpy * s122 +
                    q0x * qperpz * s122 + 2 * q0y * q0y * s002 * theta +
                    2 * q0z * q0z * s002 * theta -
                    2 * qperpy * qperpy * s002 * theta -
                    2 * qperpz * qperpz * s002 * theta +
                    2 * qperpx * qperpy * s012 * theta -
                    2 * qperpw * qperpz * s012 * theta +
                    2 * qperpw * qperpy * s022 * theta +
                    2 * qperpx * qperpz * s022 * theta +
                    q0y * (-(qperpx * s012) - qperpw * s022 +
                           2 * qperpy * (s002 - s102) + qperpx * s112 +
                           qperpw * s122 - 2 * q0x * s012 * theta -
                           2 * q0w * s022 * theta) +
                    q0z * (2 * qperpz * s002 + qperpw * s012 - qperpx * s022 -
                           2 * qperpz * s102 - qperpw * s112 + qperpx * s122 +
                           2 * q0w * s012 * theta - 2 * q0x * s022 * theta));

            c5[0] = DerivativeTerm(
                    0.,
                    2 * (qperpy * qperpy * s000 + qperpz * qperpz * s000 -
                         qperpx * qperpy * s010 + qperpw * qperpz * s010 -
                         qperpw * qperpy * s020 - qperpx * qperpz * s020 -
                         qperpy * qperpy * s100 - qperpz * qperpz * s100 +
                         q0y * q0y * (-s000 + s100) + q0z * q0z * (-s000 + s100) +
                         qperpx * qperpy * s110 - qperpw * qperpz * s110 +
                         q0y * (q0x * (s010 - s110) + q0w * (s020 - s120)) +
                         qperpw * qperpy * s120 + qperpx * qperpz * s120 +
                         q0z * (-(q0w * s010) + q0x * s020 + q0w * s110 - q0x * s120)) *
                    theta,
                    2 * (qperpy * qperpy * s001 + qperpz * qperpz * s001 -
                         qperpx * qperpy * s011 + qperpw * qperpz * s011 -
                         qperpw * qperpy * s021 - qperpx * qperpz * s021 -
                         qperpy * qperpy * s101 - qperpz * qperpz * s101 +
                         q0y * q0y * (-s001 + s101) + q0z * q0z * (-s001 + s101) +
                         qperpx * qperpy * s111 - qperpw * qperpz * s111 +
                         q0y * (q0x * (s011 - s111) + q0w * (s021 - s121)) +
                         qperpw * qperpy * s121 + qperpx * qperpz * s121 +
                         q0z * (-(q0w * s011) + q0x * s021 + q0w * s111 - q0x * s121)) *
                    theta,
                    2 * (qperpy * qperpy * s002 + qperpz * qperpz * s002 -
                         qperpx * qperpy * s012 + qperpw * qperpz * s012 -
                         qperpw * qperpy * s022 - qperpx * qperpz * s022 -
                         qperpy * qperpy * s102 - qperpz * qperpz * s102 +
                         q0y * q0y * (-s002 + s102) + q0z * q0z * (-s002 + s102) +
                         qperpx * qperpy * s112 - qperpw * qperpz * s112 +
                         q0y * (q0x * (s012 - s112) + q0w * (s022 - s122)) +
                         qperpw * qperpy * s122 + qperpx * qperpz * s122 +
                         q0z * (-(q0w * s012) + q0x * s022 + q0w * s112 - q0x * s122)) *
                    theta);

            c1[1] = DerivativeTerm(
                    -t0y + t1y,
                    -(qperpx * qperpy * s000) - qperpw * qperpz * s000 - s010 +
                    q0z * q0z * s010 + qperpx * qperpx * s010 +
                    qperpz * qperpz * s010 - q0y * q0z * s020 +
                    qperpw * qperpx * s020 - qperpy * qperpz * s020 +
                    qperpx * qperpy * s100 + qperpw * qperpz * s100 +
                    q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) + s110 -
                    q0z * q0z * s110 - qperpx * qperpx * s110 -
                    qperpz * qperpz * s110 +
                    q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
                    q0y * q0z * s120 - qperpw * qperpx * s120 +
                    qperpy * qperpz * s120,
                    -(qperpx * qperpy * s001) - qperpw * qperpz * s001 - s011 +
                    q0z * q0z * s011 + qperpx * qperpx * s011 +
                    qperpz * qperpz * s011 - q0y * q0z * s021 +
                    qperpw * qperpx * s021 - qperpy * qperpz * s021 +
                    qperpx * qperpy * s101 + qperpw * qperpz * s101 +
                    q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) + s111 -
                    q0z * q0z * s111 - qperpx * qperpx * s111 -
                    qperpz * qperpz * s111 +
                    q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
                    q0y * q0z * s121 - qperpw * qperpx * s121 +
                    qperpy * qperpz * s121,
                    -(qperpx * qperpy * s002) - qperpw * qperpz * s002 - s012 +
                    q0z * q0z * s012 + qperpx * qperpx * s012 +
                    qperpz * qperpz * s012 - q0y * q0z * s022 +
                    qperpw * qperpx * s022 - qperpy * qperpz * s022 +
                    qperpx * qperpy * s102 + qperpw * qperpz * s102 +
                    q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) + s112 -
                    q0z * q0z * s112 - qperpx * qperpx * s112 -
                    qperpz * qperpz * s112 +
                    q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
                    q0y * q0z * s122 - qperpw * qperpx * s122 +
                    qperpy * qperpz * s122);

            c2[1] = DerivativeTerm(
                    0.,
                    qperpx * qperpy * s000 + qperpw * qperpz * s000 + q0z * q0z * s010 -
                    qperpx * qperpx * s010 - qperpz * qperpz * s010 -
                    q0y * q0z * s020 - qperpw * qperpx * s020 +
                    qperpy * qperpz * s020 - qperpx * qperpy * s100 -
                    qperpw * qperpz * s100 + q0x * q0x * (s010 - s110) -
                    q0z * q0z * s110 + qperpx * qperpx * s110 +
                    qperpz * qperpz * s110 + q0y * q0z * s120 +
                    qperpw * qperpx * s120 - qperpy * qperpz * s120 +
                    2 * q0z * qperpw * s000 * theta +
                    2 * q0y * qperpx * s000 * theta -
                    4 * q0z * qperpz * s010 * theta +
                    2 * q0z * qperpy * s020 * theta +
                    2 * q0y * qperpz * s020 * theta +
                    q0x * (q0w * s020 + q0y * (-s000 + s100) - q0w * s120 +
                           2 * qperpy * s000 * theta - 4 * qperpx * s010 * theta -
                           2 * qperpw * s020 * theta) +
                    q0w * (-(q0z * s000) + q0z * s100 + 2 * qperpz * s000 * theta -
                           2 * qperpx * s020 * theta),
                    qperpx * qperpy * s001 + qperpw * qperpz * s001 + q0z * q0z * s011 -
                    qperpx * qperpx * s011 - qperpz * qperpz * s011 -
                    q0y * q0z * s021 - qperpw * qperpx * s021 +
                    qperpy * qperpz * s021 - qperpx * qperpy * s101 -
                    qperpw * qperpz * s101 + q0x * q0x * (s011 - s111) -
                    q0z * q0z * s111 + qperpx * qperpx * s111 +
                    qperpz * qperpz * s111 + q0y * q0z * s121 +
                    qperpw * qperpx * s121 - qperpy * qperpz * s121 +
                    2 * q0z * qperpw * s001 * theta +
                    2 * q0y * qperpx * s001 * theta -
                    4 * q0z * qperpz * s011 * theta +
                    2 * q0z * qperpy * s021 * theta +
                    2 * q0y * qperpz * s021 * theta +
                    q0x * (q0w * s021 + q0y * (-s001 + s101) - q0w * s121 +
                           2 * qperpy * s001 * theta - 4 * qperpx * s011 * theta -
                           2 * qperpw * s021 * theta) +
                    q0w * (-(q0z * s001) + q0z * s101 + 2 * qperpz * s001 * theta -
                           2 * qperpx * s021 * theta),
                    qperpx * qperpy * s002 + qperpw * qperpz * s002 + q0z * q0z * s012 -
                    qperpx * qperpx * s012 - qperpz * qperpz * s012 -
                    q0y * q0z * s022 - qperpw * qperpx * s022 +
                    qperpy * qperpz * s022 - qperpx * qperpy * s102 -
                    qperpw * qperpz * s102 + q0x * q0x * (s012 - s112) -
                    q0z * q0z * s112 + qperpx * qperpx * s112 +
                    qperpz * qperpz * s112 + q0y * q0z * s122 +
                    qperpw * qperpx * s122 - qperpy * qperpz * s122 +
                    2 * q0z * qperpw * s002 * theta +
                    2 * q0y * qperpx * s002 * theta -
                    4 * q0z * qperpz * s012 * theta +
                    2 * q0z * qperpy * s022 * theta +
                    2 * q0y * qperpz * s022 * theta +
                    q0x * (q0w * s022 + q0y * (-s002 + s102) - q0w * s122 +
                           2 * qperpy * s002 * theta - 4 * qperpx * s012 * theta -
                           2 * qperpw * s022 * theta) +
                    q0w * (-(q0z * s002) + q0z * s102 + 2 * qperpz * s002 * theta -
                           2 * qperpx * s022 * theta));

            c3[1] = DerivativeTerm(
                    0., 2 * (-(q0x * qperpy * s000) - q0w * qperpz * s000 +
                             2 * q0x * qperpx * s010 + q0x * qperpw * s020 +
                             q0w * qperpx * s020 + q0x * qperpy * s100 +
                             q0w * qperpz * s100 - 2 * q0x * qperpx * s110 -
                             q0x * qperpw * s120 - q0w * qperpx * s120 +
                             q0z * (2 * qperpz * s010 - qperpy * s020 +
                                    qperpw * (-s000 + s100) - 2 * qperpz * s110 +
                                    qperpy * s120) +
                             q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
                                    qperpz * s120)) *
                        theta,
                    2 * (-(q0x * qperpy * s001) - q0w * qperpz * s001 +
                         2 * q0x * qperpx * s011 + q0x * qperpw * s021 +
                         q0w * qperpx * s021 + q0x * qperpy * s101 +
                         q0w * qperpz * s101 - 2 * q0x * qperpx * s111 -
                         q0x * qperpw * s121 - q0w * qperpx * s121 +
                         q0z * (2 * qperpz * s011 - qperpy * s021 +
                                qperpw * (-s001 + s101) - 2 * qperpz * s111 +
                                qperpy * s121) +
                         q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
                                qperpz * s121)) *
                    theta,
                    2 * (-(q0x * qperpy * s002) - q0w * qperpz * s002 +
                         2 * q0x * qperpx * s012 + q0x * qperpw * s022 +
                         q0w * qperpx * s022 + q0x * qperpy * s102 +
                         q0w * qperpz * s102 - 2 * q0x * qperpx * s112 -
                         q0x * qperpw * s122 - q0w * qperpx * s122 +
                         q0z * (2 * qperpz * s012 - qperpy * s022 +
                                qperpw * (-s002 + s102) - 2 * qperpz * s112 +
                                qperpy * s122) +
                         q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
                                qperpz * s122)) *
                    theta);

            c4[1] = DerivativeTerm(
                    0.,
                    -(q0x * qperpy * s000) - q0w * qperpz * s000 +
                    2 * q0x * qperpx * s010 + q0x * qperpw * s020 +
                    q0w * qperpx * s020 + q0x * qperpy * s100 +
                    q0w * qperpz * s100 - 2 * q0x * qperpx * s110 -
                    q0x * qperpw * s120 - q0w * qperpx * s120 +
                    2 * qperpx * qperpy * s000 * theta +
                    2 * qperpw * qperpz * s000 * theta +
                    2 * q0x * q0x * s010 * theta + 2 * q0z * q0z * s010 * theta -
                    2 * qperpx * qperpx * s010 * theta -
                    2 * qperpz * qperpz * s010 * theta +
                    2 * q0w * q0x * s020 * theta -
                    2 * qperpw * qperpx * s020 * theta +
                    2 * qperpy * qperpz * s020 * theta +
                    q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
                           qperpz * s120 - 2 * q0x * s000 * theta) +
                    q0z * (2 * qperpz * s010 - qperpy * s020 +
                           qperpw * (-s000 + s100) - 2 * qperpz * s110 +
                           qperpy * s120 - 2 * q0w * s000 * theta -
                           2 * q0y * s020 * theta),
                    -(q0x * qperpy * s001) - q0w * qperpz * s001 +
                    2 * q0x * qperpx * s011 + q0x * qperpw * s021 +
                    q0w * qperpx * s021 + q0x * qperpy * s101 +
                    q0w * qperpz * s101 - 2 * q0x * qperpx * s111 -
                    q0x * qperpw * s121 - q0w * qperpx * s121 +
                    2 * qperpx * qperpy * s001 * theta +
                    2 * qperpw * qperpz * s001 * theta +
                    2 * q0x * q0x * s011 * theta + 2 * q0z * q0z * s011 * theta -
                    2 * qperpx * qperpx * s011 * theta -
                    2 * qperpz * qperpz * s011 * theta +
                    2 * q0w * q0x * s021 * theta -
                    2 * qperpw * qperpx * s021 * theta +
                    2 * qperpy * qperpz * s021 * theta +
                    q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
                           qperpz * s121 - 2 * q0x * s001 * theta) +
                    q0z * (2 * qperpz * s011 - qperpy * s021 +
                           qperpw * (-s001 + s101) - 2 * qperpz * s111 +
                           qperpy * s121 - 2 * q0w * s001 * theta -
                           2 * q0y * s021 * theta),
                    -(q0x * qperpy * s002) - q0w * qperpz * s002 +
                    2 * q0x * qperpx * s012 + q0x * qperpw * s022 +
                    q0w * qperpx * s022 + q0x * qperpy * s102 +
                    q0w * qperpz * s102 - 2 * q0x * qperpx * s112 -
                    q0x * qperpw * s122 - q0w * qperpx * s122 +
                    2 * qperpx * qperpy * s002 * theta +
                    2 * qperpw * qperpz * s002 * theta +
                    2 * q0x * q0x * s012 * theta + 2 * q0z * q0z * s012 * theta -
                    2 * qperpx * qperpx * s012 * theta -
                    2 * qperpz * qperpz * s012 * theta +
                    2 * q0w * q0x * s022 * theta -
                    2 * qperpw * qperpx * s022 * theta +
                    2 * qperpy * qperpz * s022 * theta +
                    q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
                           qperpz * s122 - 2 * q0x * s002 * theta) +
                    q0z * (2 * qperpz * s012 - qperpy * s022 +
                           qperpw * (-s002 + s102) - 2 * qperpz * s112 +
                           qperpy * s122 - 2 * q0w * s002 * theta -
                           2 * q0y * s022 * theta));

            c5[1] = DerivativeTerm(
                    0., -2 * (qperpx * qperpy * s000 + qperpw * qperpz * s000 +
                              q0z * q0z * s010 - qperpx * qperpx * s010 -
                              qperpz * qperpz * s010 - q0y * q0z * s020 -
                              qperpw * qperpx * s020 + qperpy * qperpz * s020 -
                              qperpx * qperpy * s100 - qperpw * qperpz * s100 +
                              q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) -
                              q0z * q0z * s110 + qperpx * qperpx * s110 +
                              qperpz * qperpz * s110 +
                              q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
                              q0y * q0z * s120 + qperpw * qperpx * s120 -
                              qperpy * qperpz * s120) *
                        theta,
                    -2 * (qperpx * qperpy * s001 + qperpw * qperpz * s001 +
                          q0z * q0z * s011 - qperpx * qperpx * s011 -
                          qperpz * qperpz * s011 - q0y * q0z * s021 -
                          qperpw * qperpx * s021 + qperpy * qperpz * s021 -
                          qperpx * qperpy * s101 - qperpw * qperpz * s101 +
                          q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) -
                          q0z * q0z * s111 + qperpx * qperpx * s111 +
                          qperpz * qperpz * s111 +
                          q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
                          q0y * q0z * s121 + qperpw * qperpx * s121 -
                          qperpy * qperpz * s121) *
                    theta,
                    -2 * (qperpx * qperpy * s002 + qperpw * qperpz * s002 +
                          q0z * q0z * s012 - qperpx * qperpx * s012 -
                          qperpz * qperpz * s012 - q0y * q0z * s022 -
                          qperpw * qperpx * s022 + qperpy * qperpz * s022 -
                          qperpx * qperpy * s102 - qperpw * qperpz * s102 +
                          q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) -
                          q0z * q0z * s112 + qperpx * qperpx * s112 +
                          qperpz * qperpz * s112 +
                          q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
                          q0y * q0z * s122 + qperpw * qperpx * s122 -
                          qperpy * qperpz * s122) *
                    theta);

            c1[2] = DerivativeTerm(
                    -t0z + t1z, (qperpw * qperpy * s000 - qperpx * qperpz * s000 -
                                 q0y * q0z * s010 - qperpw * qperpx * s010 -
                                 qperpy * qperpz * s010 - s020 + q0y * q0y * s020 +
                                 qperpx * qperpx * s020 + qperpy * qperpy * s020 -
                                 qperpw * qperpy * s100 + qperpx * qperpz * s100 +
                                 q0x * q0z * (-s000 + s100) + q0y * q0z * s110 +
                                 qperpw * qperpx * s110 + qperpy * qperpz * s110 +
                                 q0w * (q0y * (s000 - s100) + q0x * (-s010 + s110)) +
                                 q0x * q0x * (s020 - s120) + s120 - q0y * q0y * s120 -
                                 qperpx * qperpx * s120 - qperpy * qperpy * s120),
                    (qperpw * qperpy * s001 - qperpx * qperpz * s001 -
                     q0y * q0z * s011 - qperpw * qperpx * s011 -
                     qperpy * qperpz * s011 - s021 + q0y * q0y * s021 +
                     qperpx * qperpx * s021 + qperpy * qperpy * s021 -
                     qperpw * qperpy * s101 + qperpx * qperpz * s101 +
                     q0x * q0z * (-s001 + s101) + q0y * q0z * s111 +
                     qperpw * qperpx * s111 + qperpy * qperpz * s111 +
                     q0w * (q0y * (s001 - s101) + q0x * (-s011 + s111)) +
                     q0x * q0x * (s021 - s121) + s121 - q0y * q0y * s121 -
                     qperpx * qperpx * s121 - qperpy * qperpy * s121),
                    (qperpw * qperpy * s002 - qperpx * qperpz * s002 -
                     q0y * q0z * s012 - qperpw * qperpx * s012 -
                     qperpy * qperpz * s012 - s022 + q0y * q0y * s022 +
                     qperpx * qperpx * s022 + qperpy * qperpy * s022 -
                     qperpw * qperpy * s102 + qperpx * qperpz * s102 +
                     q0x * q0z * (-s002 + s102) + q0y * q0z * s112 +
                     qperpw * qperpx * s112 + qperpy * qperpz * s112 +
                     q0w * (q0y * (s002 - s102) + q0x * (-s012 + s112)) +
                     q0x * q0x * (s022 - s122) + s122 - q0y * q0y * s122 -
                     qperpx * qperpx * s122 - qperpy * qperpy * s122));

            c2[2] = DerivativeTerm(
                    0.,
                    (q0w * q0y * s000 - q0x * q0z * s000 - qperpw * qperpy * s000 +
                     qperpx * qperpz * s000 - q0w * q0x * s010 - q0y * q0z * s010 +
                     qperpw * qperpx * s010 + qperpy * qperpz * s010 +
                     q0x * q0x * s020 + q0y * q0y * s020 - qperpx * qperpx * s020 -
                     qperpy * qperpy * s020 - q0w * q0y * s100 + q0x * q0z * s100 +
                     qperpw * qperpy * s100 - qperpx * qperpz * s100 +
                     q0w * q0x * s110 + q0y * q0z * s110 - qperpw * qperpx * s110 -
                     qperpy * qperpz * s110 - q0x * q0x * s120 - q0y * q0y * s120 +
                     qperpx * qperpx * s120 + qperpy * qperpy * s120 -
                     2 * q0y * qperpw * s000 * theta + 2 * q0z * qperpx * s000 * theta -
                     2 * q0w * qperpy * s000 * theta + 2 * q0x * qperpz * s000 * theta +
                     2 * q0x * qperpw * s010 * theta + 2 * q0w * qperpx * s010 * theta +
                     2 * q0z * qperpy * s010 * theta + 2 * q0y * qperpz * s010 * theta -
                     4 * q0x * qperpx * s020 * theta - 4 * q0y * qperpy * s020 * theta),
                    (q0w * q0y * s001 - q0x * q0z * s001 - qperpw * qperpy * s001 +
                     qperpx * qperpz * s001 - q0w * q0x * s011 - q0y * q0z * s011 +
                     qperpw * qperpx * s011 + qperpy * qperpz * s011 +
                     q0x * q0x * s021 + q0y * q0y * s021 - qperpx * qperpx * s021 -
                     qperpy * qperpy * s021 - q0w * q0y * s101 + q0x * q0z * s101 +
                     qperpw * qperpy * s101 - qperpx * qperpz * s101 +
                     q0w * q0x * s111 + q0y * q0z * s111 - qperpw * qperpx * s111 -
                     qperpy * qperpz * s111 - q0x * q0x * s121 - q0y * q0y * s121 +
                     qperpx * qperpx * s121 + qperpy * qperpy * s121 -
                     2 * q0y * qperpw * s001 * theta + 2 * q0z * qperpx * s001 * theta -
                     2 * q0w * qperpy * s001 * theta + 2 * q0x * qperpz * s001 * theta +
                     2 * q0x * qperpw * s011 * theta + 2 * q0w * qperpx * s011 * theta +
                     2 * q0z * qperpy * s011 * theta + 2 * q0y * qperpz * s011 * theta -
                     4 * q0x * qperpx * s021 * theta - 4 * q0y * qperpy * s021 * theta),
                    (q0w * q0y * s002 - q0x * q0z * s002 - qperpw * qperpy * s002 +
                     qperpx * qperpz * s002 - q0w * q0x * s012 - q0y * q0z * s012 +
                     qperpw * qperpx * s012 + qperpy * qperpz * s012 +
                     q0x * q0x * s022 + q0y * q0y * s022 - qperpx * qperpx * s022 -
                     qperpy * qperpy * s022 - q0w * q0y * s102 + q0x * q0z * s102 +
                     qperpw * qperpy * s102 - qperpx * qperpz * s102 +
                     q0w * q0x * s112 + q0y * q0z * s112 - qperpw * qperpx * s112 -
                     qperpy * qperpz * s112 - q0x * q0x * s122 - q0y * q0y * s122 +
                     qperpx * qperpx * s122 + qperpy * qperpy * s122 -
                     2 * q0y * qperpw * s002 * theta + 2 * q0z * qperpx * s002 * theta -
                     2 * q0w * qperpy * s002 * theta + 2 * q0x * qperpz * s002 * theta +
                     2 * q0x * qperpw * s012 * theta + 2 * q0w * qperpx * s012 * theta +
                     2 * q0z * qperpy * s012 * theta + 2 * q0y * qperpz * s012 * theta -
                     4 * q0x * qperpx * s022 * theta -
                     4 * q0y * qperpy * s022 * theta));

            c3[2] = DerivativeTerm(
                    0., -2 * (-(q0w * qperpy * s000) + q0x * qperpz * s000 +
                              q0x * qperpw * s010 + q0w * qperpx * s010 -
                              2 * q0x * qperpx * s020 + q0w * qperpy * s100 -
                              q0x * qperpz * s100 - q0x * qperpw * s110 -
                              q0w * qperpx * s110 +
                              q0z * (qperpx * s000 + qperpy * s010 - qperpx * s100 -
                                     qperpy * s110) +
                              2 * q0x * qperpx * s120 +
                              q0y * (qperpz * s010 - 2 * qperpy * s020 +
                                     qperpw * (-s000 + s100) - qperpz * s110 +
                                     2 * qperpy * s120)) *
                        theta,
                    -2 * (-(q0w * qperpy * s001) + q0x * qperpz * s001 +
                          q0x * qperpw * s011 + q0w * qperpx * s011 -
                          2 * q0x * qperpx * s021 + q0w * qperpy * s101 -
                          q0x * qperpz * s101 - q0x * qperpw * s111 -
                          q0w * qperpx * s111 +
                          q0z * (qperpx * s001 + qperpy * s011 - qperpx * s101 -
                                 qperpy * s111) +
                          2 * q0x * qperpx * s121 +
                          q0y * (qperpz * s011 - 2 * qperpy * s021 +
                                 qperpw * (-s001 + s101) - qperpz * s111 +
                                 2 * qperpy * s121)) *
                    theta,
                    -2 * (-(q0w * qperpy * s002) + q0x * qperpz * s002 +
                          q0x * qperpw * s012 + q0w * qperpx * s012 -
                          2 * q0x * qperpx * s022 + q0w * qperpy * s102 -
                          q0x * qperpz * s102 - q0x * qperpw * s112 -
                          q0w * qperpx * s112 +
                          q0z * (qperpx * s002 + qperpy * s012 - qperpx * s102 -
                                 qperpy * s112) +
                          2 * q0x * qperpx * s122 +
                          q0y * (qperpz * s012 - 2 * qperpy * s022 +
                                 qperpw * (-s002 + s102) - qperpz * s112 +
                                 2 * qperpy * s122)) *
                    theta);

            c4[2] = DerivativeTerm(
                    0.,
                    q0w * qperpy * s000 - q0x * qperpz * s000 - q0x * qperpw * s010 -
                    q0w * qperpx * s010 + 2 * q0x * qperpx * s020 -
                    q0w * qperpy * s100 + q0x * qperpz * s100 +
                    q0x * qperpw * s110 + q0w * qperpx * s110 -
                    2 * q0x * qperpx * s120 - 2 * qperpw * qperpy * s000 * theta +
                    2 * qperpx * qperpz * s000 * theta -
                    2 * q0w * q0x * s010 * theta +
                    2 * qperpw * qperpx * s010 * theta +
                    2 * qperpy * qperpz * s010 * theta +
                    2 * q0x * q0x * s020 * theta + 2 * q0y * q0y * s020 * theta -
                    2 * qperpx * qperpx * s020 * theta -
                    2 * qperpy * qperpy * s020 * theta +
                    q0z * (-(qperpx * s000) - qperpy * s010 + qperpx * s100 +
                           qperpy * s110 - 2 * q0x * s000 * theta) +
                    q0y * (-(qperpz * s010) + 2 * qperpy * s020 +
                           qperpw * (s000 - s100) + qperpz * s110 -
                           2 * qperpy * s120 + 2 * q0w * s000 * theta -
                           2 * q0z * s010 * theta),
                    q0w * qperpy * s001 - q0x * qperpz * s001 - q0x * qperpw * s011 -
                    q0w * qperpx * s011 + 2 * q0x * qperpx * s021 -
                    q0w * qperpy * s101 + q0x * qperpz * s101 +
                    q0x * qperpw * s111 + q0w * qperpx * s111 -
                    2 * q0x * qperpx * s121 - 2 * qperpw * qperpy * s001 * theta +
                    2 * qperpx * qperpz * s001 * theta -
                    2 * q0w * q0x * s011 * theta +
                    2 * qperpw * qperpx * s011 * theta +
                    2 * qperpy * qperpz * s011 * theta +
                    2 * q0x * q0x * s021 * theta + 2 * q0y * q0y * s021 * theta -
                    2 * qperpx * qperpx * s021 * theta -
                    2 * qperpy * qperpy * s021 * theta +
                    q0z * (-(qperpx * s001) - qperpy * s011 + qperpx * s101 +
                           qperpy * s111 - 2 * q0x * s001 * theta) +
                    q0y * (-(qperpz * s011) + 2 * qperpy * s021 +
                           qperpw * (s001 - s101) + qperpz * s111 -
                           2 * qperpy * s121 + 2 * q0w * s001 * theta -
                           2 * q0z * s011 * theta),
                    q0w * qperpy * s002 - q0x * qperpz * s002 - q0x * qperpw * s012 -
                    q0w * qperpx * s012 + 2 * q0x * qperpx * s022 -
                    q0w * qperpy * s102 + q0x * qperpz * s102 +
                    q0x * qperpw * s112 + q0w * qperpx * s112 -
                    2 * q0x * qperpx * s122 - 2 * qperpw * qperpy * s002 * theta +
                    2 * qperpx * qperpz * s002 * theta -
                    2 * q0w * q0x * s012 * theta +
                    2 * qperpw * qperpx * s012 * theta +
                    2 * qperpy * qperpz * s012 * theta +
                    2 * q0x * q0x * s022 * theta + 2 * q0y * q0y * s022 * theta -
                    2 * qperpx * qperpx * s022 * theta -
                    2 * qperpy * qperpy * s022 * theta +
                    q0z * (-(qperpx * s002) - qperpy * s012 + qperpx * s102 +
                           qperpy * s112 - 2 * q0x * s002 * theta) +
                    q0y * (-(qperpz * s012) + 2 * qperpy * s022 +
                           qperpw * (s002 - s102) + qperpz * s112 -
                           2 * qperpy * s122 + 2 * q0w * s002 * theta -
                           2 * q0z * s012 * theta));

            c5[2] = DerivativeTerm(
                    0., 2 * (qperpw * qperpy * s000 - qperpx * qperpz * s000 +
                             q0y * q0z * s010 - qperpw * qperpx * s010 -
                             qperpy * qperpz * s010 - q0y * q0y * s020 +
                             qperpx * qperpx * s020 + qperpy * qperpy * s020 +
                             q0x * q0z * (s000 - s100) - qperpw * qperpy * s100 +
                             qperpx * qperpz * s100 +
                             q0w * (q0y * (-s000 + s100) + q0x * (s010 - s110)) -
                             q0y * q0z * s110 + qperpw * qperpx * s110 +
                             qperpy * qperpz * s110 + q0y * q0y * s120 -
                             qperpx * qperpx * s120 - qperpy * qperpy * s120 +
                             q0x * q0x * (-s020 + s120)) *
                        theta,
                    2 * (qperpw * qperpy * s001 - qperpx * qperpz * s001 +
                         q0y * q0z * s011 - qperpw * qperpx * s011 -
                         qperpy * qperpz * s011 - q0y * q0y * s021 +
                         qperpx * qperpx * s021 + qperpy * qperpy * s021 +
                         q0x * q0z * (s001 - s101) - qperpw * qperpy * s101 +
                         qperpx * qperpz * s101 +
                         q0w * (q0y * (-s001 + s101) + q0x * (s011 - s111)) -
                         q0y * q0z * s111 + qperpw * qperpx * s111 +
                         qperpy * qperpz * s111 + q0y * q0y * s121 -
                         qperpx * qperpx * s121 - qperpy * qperpy * s121 +
                         q0x * q0x * (-s021 + s121)) *
                    theta,
                    2 * (qperpw * qperpy * s002 - qperpx * qperpz * s002 +
                         q0y * q0z * s012 - qperpw * qperpx * s012 -
                         qperpy * qperpz * s012 - q0y * q0y * s022 +
                         qperpx * qperpx * s022 + qperpy * qperpy * s022 +
                         q0x * q0z * (s002 - s102) - qperpw * qperpy * s102 +
                         qperpx * qperpz * s102 +
                         q0w * (q0y * (-s002 + s102) + q0x * (s012 - s112)) -
                         q0y * q0z * s112 + qperpw * qperpx * s112 +
                         qperpy * qperpz * s112 + q0y * q0y * s122 -
                         qperpx * qperpx * s122 - qperpy * qperpy * s122 +
                         q0x * q0x * (-s022 + s122)) *
                    theta);
        }
    }

    void AnimatedTransform::decompose(const Matrix &m, Vector3f *T,
                                      Quaternion *Rquat, Matrix *S) {
        // Extract translation _T_ from transformation matrix
        (*T)[0] = m.data[0][3];
        (*T)[1] = m.data[1][3];
        (*T)[2] = m.data[2][3];

        // Compute new transformation matrix _M_ without translation
        Matrix M = m;
        for (int i = 0; i < 3; ++i) M.data[i][3] = M.data[3][i] = 0.f;
        M.data[3][3] = 1.f;

        // Extract rotation _R_ from transformation matrix
        Float norm;
        int count = 0;
        Matrix R = M;
        do {
            // Compute next matrix _Rnext_ in series
            Matrix Rnext;
            Matrix Rit = inverse(transpose(R));
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    Rnext.data[i][j] = 0.5f * (R.data[i][j] + Rit.data[i][j]);

            // Compute norm of difference between _R_ and _Rnext_
            norm = 0;
            for (int i = 0; i < 3; ++i) {
                Float n = std::abs(R.data[i][0] - Rnext.data[i][0]) +
                          std::abs(R.data[i][1] - Rnext.data[i][1]) +
                          std::abs(R.data[i][2] - Rnext.data[i][2]);
                norm = std::max(norm, n);
            }
            R = Rnext;
        } while (++count < 100 && norm > .0001);
        // XXX TODO FIXME deal with flip...
        *Rquat = Quaternion(R);

        // Compute scale _S_ using rotation and original matrix
        *S = inverse(R) * M;
    }

    void AnimatedTransform::interpolate(Float time, Transform *t) const {
        // Handle boundary conditions for matrix interpolation
        if (!actuallyAnimated || time <= startTime) {
            *t = *startTransform;
            return;
        }
        if (time >= endTime) {
            *t = *endTransform;
            return;
        }
        Float dt = (time - startTime) / (endTime - startTime);
        // Interpolate translation at _dt_
        Vector3f trans = (1 - dt) * T[0] + dt * T[1];

        // Interpolate rotation at _dt_
        Quaternion rotate = slerp(dt, R[0], R[1]);

        // Interpolate scale at _dt_
        Matrix scale;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                scale.data[i][j] = lerp(dt, S[0].data[i][j], S[1].data[i][j]);

        // Compute interpolated matrix as product of interpolated components
        *t = translate(trans) * rotate.toTransform() * Transform(scale);
    }

    Ray AnimatedTransform::operator()(const Ray &r) const {
        if (!actuallyAnimated || r.time <= startTime)
            return (*startTransform)(r);
        else if (r.time >= endTime)
            return (*endTransform)(r);
        else {
            Transform t;
            interpolate(r.time, &t);
            return t(r);
        }
    }

    RayDifferential AnimatedTransform::operator()(const RayDifferential &r) const {
        if (!actuallyAnimated || r.time <= startTime)
            return (*startTransform)(r);
        else if (r.time >= endTime)
            return (*endTransform)(r);
        else {
            Transform t;
            interpolate(r.time, &t);
            return t(r);
        }
    }

    Point3f AnimatedTransform::transformPosition(rzpbr::Float time, const rzpbr::Point3f & p) const{
        if (!actuallyAnimated || time <= startTime)
            return startTransform->transformPosition(p);
        else if (time >= endTime)
            return endTransform->transformPosition(p);
        Transform t;
        interpolate(time, &t);
        return t.transformPosition(p);
    }

    Vector3f AnimatedTransform::transformDirection(rzpbr::Float time, const rzpbr::Point3f & v) const {
        if (!actuallyAnimated || time <= startTime)
            return startTransform->transformDirection(v);
        else if (time >= endTime)
            return endTransform->transformDirection(v);
        Transform t;
        interpolate(time, &t);
        return t.transformDirection(v);
    }


    Bound3f AnimatedTransform::motionBounds(const Bound3f &b) const {
        if (!actuallyAnimated) return (*startTransform)(b);
        if (!hasRotation)
            return unionBound((*startTransform)(b), (*endTransform)(b));
        // Return motion bounds accounting for animated rotation
        Bound3f bounds;
        for (int corner = 0; corner < 8; ++corner)
            bounds = unionBound(bounds, boundPointMotion(b.corner(corner)));
        return bounds;
    }

    Bound3f AnimatedTransform::boundPointMotion(const Point3f &p) const {
        if (!actuallyAnimated) return Bound3f(startTransform->transformPosition(p));
        Bound3f bounds(startTransform->transformPosition(p), endTransform->transformPosition(p));
        Float cosTheta = dot(R[0], R[1]);
        Float lower = -1;
        Float upper = 1;
        Float theta = std::acos(std::clamp(cosTheta, lower, upper));
        for (int c = 0; c < 3; ++c) {
            // Find any motion derivative zeros for the component _c_
            Float zeros[8];
            int nZeros = 0;


            intervalFindZeros(c1[c].eval(p), c2[c].eval(p), c3[c].eval(p),
                              c4[c].eval(p), c5[c].eval(p), theta, Interval(0., 1.),
                              zeros, &nZeros);
            //CHECK_LE(nZeros, sizeof(zeros) / sizeof(zeros[0]));

            // Expand bounding box for any motion derivative zeros found
            for (int i = 0; i < nZeros; ++i) {
                Point3f pz = this->transformPosition(lerp(zeros[i], startTime, endTime), p);
                bounds = unionBound(bounds, pz);
            }
        }
        return bounds;
    }

}