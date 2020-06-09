//
// Created by Ryota Egusa on 2020/02/12.
//

#include <opencv2/opencv.hpp>
#include <random>
#include "EPnPSolver.h"

namespace ORB_SLAM2
{

    int EPnPSolver::Optimize(Frame &frame)
    {
        int nInitialCorrespondences = 0;

        // Set Frame vertex

        // Set MapPoint vertices
        const int N = frame.N;

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        std::vector<std::size_t> mapPointIndexes;
        std::vector<cv::Point3f> mapPoints;
        std::vector<cv::Point2f> imagePoints;

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = frame.mvpMapPoints[i];
                if (pMP)
                {
                    // Monocular observation
                    if (frame.mvuRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        frame.mvbOutlier[i] = false;

                        const auto &kp = frame.mvKeysUn[i];
                        imagePoints.emplace_back(kp.pt.x, kp.pt.y);

                        const cv::Mat_<float> pt = pMP->GetWorldPos();
                        mapPoints.emplace_back(pt(0), pt(1), pt(2));

                        mapPointIndexes.emplace_back(i);
                    }
                    else  // Stereo observation
                    {
                        std::cerr << "stereo not supported" << std::endl;
                        std::exit(1);
//                        nInitialCorrespondences++;
//                        frame.mvbOutlier[i] = false;
//
//                        //SET EDGE
//                        Eigen::Matrix<double, 3, 1> obs;
//                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
//                        const float &kp_ur = pFrame->mvuRight[i];
//                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
//
//                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
//
//                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
//                        e->setMeasurement(obs);
//                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
//                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
//                        e->setInformation(Info);
//
//                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
//                        e->setRobustKernel(rk);
//                        rk->setDelta(deltaStereo);
//
//                        e->fx = pFrame->fx;
//                        e->fy = pFrame->fy;
//                        e->cx = pFrame->cx;
//                        e->cy = pFrame->cy;
//                        e->bf = pFrame->mbf;
//                        cv::Mat Xw = pMP->GetWorldPos();
//                        e->Xw[0] = Xw.at<float>(0);
//                        e->Xw[1] = Xw.at<float>(1);
//                        e->Xw[2] = Xw.at<float>(2);
//
//                        optimizer.addEdge(e);
//
//                        vpEdgesStereo.push_back(e);
//                        vnIndexEdgeStereo.push_back(i);
                    }
                }

            }
        }


        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono = 5.991;
        // const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};

        const cv::Mat_<float> cam = (cv::Mat_<float>(3, 3) << frame.fx, 0, frame.cx,
                0, frame.fy, frame.cy,
                0, 0, 1);
        const cv::Mat_<float> dist;

        std::vector<bool> isBad(mapPoints.size());

        std::vector<cv::Point2f> projected;
        projected.reserve(mapPoints.size());

        std::vector<cv::Point3f> inlierMapPoints;
        std::vector<cv::Point2f> inlierImagePoints;

        std::vector<bool> exclude(mapPoints.size());
        std::vector<size_t> indexes(mapPoints.size());
        for (int i = 0; i < indexes.size(); i++)
        {
            indexes[i] = i;
        }

        std::mt19937 engine(std::time(nullptr));

        int outlierCount = indexes.size() * 0.2;

        double bestEval = INFINITY;
        std::vector<bool> bestExclude;
        cv::Mat_<float> bestPose = cv::Mat::eye(4, 4, CV_32F);

        for (size_t it = 0; it < 100; it++)
        {
            inlierMapPoints.clear();
            inlierImagePoints.clear();
            std::fill(exclude.begin(), exclude.end(), false);
            std::shuffle(indexes.begin(), indexes.end(), engine);
            std::cout.flush();
            for (int i = 0; i < outlierCount; i++)
            {
                exclude[indexes[i]] = true;
            }
            //outlierCount = 0;

            for (std::size_t i = 0; i < mapPoints.size(); i++)
            {
                if (exclude[i])
                {
                    continue;
                }
                inlierMapPoints.emplace_back(mapPoints[i]);
                inlierImagePoints.emplace_back(imagePoints[i]);
            }

            auto pose = frame.mTcw.clone();
            auto rot = pose.colRange(0, 3).rowRange(0, 3);
            cv::Mat rotVec;
            cv::Rodrigues(rot, rotVec);
            auto trans = pose.colRange(3, 4).rowRange(0, 3);

            cv::solvePnP(inlierMapPoints, inlierImagePoints, cam, dist, rotVec, trans, true, cv::SOLVEPNP_EPNP);

            cv::projectPoints(mapPoints, rotVec, trans, cam, dist, projected);

            double eval = 0;
            for (size_t i = 0; i < mapPoints.size();  i++)
            {
                const auto idx = mapPointIndexes[i];
                const auto kp = frame.mvKeysUn[idx];
                const auto delta = kp.pt - projected[i];
                const auto delta2 = delta.dot(delta);
                const auto chi2 = frame.mvInvLevelSigma2[kp.octave] * delta2;
                eval += std::sqrt(delta2);

                if (chi2Mono < chi2)
                {
                    //outlierCount++;
                }
            }

            if (eval < bestEval)
            {
                bestEval = eval;
                bestExclude = exclude;
                cv::Rodrigues(rotVec, rot);
                trans.copyTo(bestPose.colRange(3, 4).rowRange(0, 3));
                rot.copyTo(bestPose.colRange(0, 3).rowRange(0, 3));
            }
        }

        for (int i = 0; i < bestExclude.size(); i++)
        {
            frame.mvbOutlier[mapPointIndexes[i]] = bestExclude[i];
        }

        // Recover optimized pose and return number of inliers
        frame.SetPose(bestPose);

        //std::cerr << nInitialCorrespondences << " " << outlierCount << std::endl;
        return nInitialCorrespondences - outlierCount;
    }
}
