//
// Created by rainzhong on 2019/11/28.
//

#include <math/vector.h>

namespace rzpbr{
    Float lerp(const Float& t, const Float& v0, const Float& v1){
        return (1 - t) * v0 + t * v1;
    }
}