//
// Created by Ryota Egusa on 2019/12/16.
//

#include "Pose.h"


Pose::Pose() : mTime(0)
{}

Pose::Pose(const Pose &p) : mTime(p.getTime()), mRotation(p.getRotation()), mTrans(p.getTrans())
{}

Pose::Pose(const double time, cv::Matx33d rotation, cv::Matx<double, 3, 1> trans) : mTime(time), mRotation(rotation),
                                                                                    mTrans(trans)
{}

bool Pose::operator<(const Pose &p) const
{
    return mTime < p.mTime;
}

double Pose::getTime() const
{
    return mTime;
}

const cv::Matx33d &Pose::getRotation() const
{
    return mRotation;
}

const cv::Matx<double, 3, 1> &Pose::getTrans() const
{
    return mTrans;
}
