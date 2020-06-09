//
// Created by gedorinku on 2020/02/13.
//

#ifndef ORB_SLAM2_REPPNPSOLVER_H
#define ORB_SLAM2_REPPNPSOLVER_H


namespace ORB_SLAM2
{
    class Frame;

    class REPPnPSolver
    {
    public:
        static int Optimize(Frame &frame, int nullspaceEtimationItr);
    };
}


#endif //ORB_SLAM2_REPPNPSOLVER_H
