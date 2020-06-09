//
// Created by Ryota Egusa on 2019/12/16.
//

#ifndef TRAJECTORY_EVALUATOR_POSE_H
#define TRAJECTORY_EVALUATOR_POSE_H


#include <opencv2/core.hpp>

class Pose
{
public:
    Pose();

    Pose(const Pose &p);

    Pose(double time, cv::Matx33d rotation, cv::Matx<double, 3, 1> trans);

    bool operator<(const Pose &p) const;

    [[nodiscard]] double getTime() const;

    [[nodiscard]] const cv::Matx33d &getRotation() const;

    [[nodiscard]] const cv::Matx<double, 3, 1> &getTrans() const;

private:
    const double mTime;
    const cv::Matx33d mRotation;
    const cv::Matx<double, 3, 1> mTrans;
};


#endif //TRAJECTORY_EVALUATOR_POSE_H
