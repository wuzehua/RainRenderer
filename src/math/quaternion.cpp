//
// Created by rainzhong on 2019/11/30.
//

#include <algorithm>
#include <math/quaternion.h>

namespace rzpbr{
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

}