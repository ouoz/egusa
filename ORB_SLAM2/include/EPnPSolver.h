//
// Created by Ryota Egusa on 2020/02/12.
//

#ifndef ORB_SLAM2_EPNPSOLVER_H
#define ORB_SLAM2_EPNPSOLVER_H

#include "Frame.h"

namespace ORB_SLAM2
{
    class EPnPSolver
    {
    public:
        static int Optimize(Frame &frame);
    };
}


#endif //ORB_SLAM2_EPNPSOLVER_H
