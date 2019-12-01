//
// Created by rainzhong on 2019/11/29.
//

#ifndef RAINRENDERER_MEDIUM_H
#define RAINRENDERER_MEDIUM_H

namespace rzpbr {


    class Medium {

    };

    struct MediumInterface{
        const Medium* inside;
        const Medium* outside;
    };
}


#endif //RAINRENDERER_MEDIUM_H
