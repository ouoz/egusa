//
// Created by gedorinku on 2020/02/13.
//

#include <cmath>
#include <include/Converter.h>

#include "reppnp.h"
#include "REPPnPSolver.h"
#include "MapPoint.h"

namespace ORB_SLAM2
{

    int REPPnPSolver::Optimize(Frame &frame, int nullspaceEtimationItr)
    {
        int nInitialCorrespondences = 0;

        REPPnP::Coordinates3D_t mapPoints;
        REPPnP::Coordinates2D_t imagePoints;
        std::vector<size_t> mapPointIndexes;
        std::vector<double> sigma2;
        REPPnP::CameraCalibration_t cam;
        cam << frame.fx, 0, frame.cx,
                0, frame.fy, frame.cy,
                0, 0, 1;

        {
            std::lock_guard<mutex> lock(MapPoint::mGlobalMutex);

            for (auto mp : frame.mvpMapPoints)
            {
                if (mp == nullptr)
                {
                    continue;
                }
                nInitialCorrespondences++;
            }

            mapPoints.conservativeResize(Eigen::NoChange, nInitialCorrespondences);
            imagePoints.conservativeResize(Eigen::NoChange, nInitialCorrespondences);
            sigma2.reserve(nInitialCorrespondences);

            for (int i = 0; i < frame.N; i++)
            {
                auto mp = frame.mvpMapPoints[i];
                if (mp == nullptr)
                {
                    continue;
                }
                if (0 <= frame.mvuRight[i])
                {
                    std::cerr << "Stereo is not supported" << std::endl;
                    return 0;
                }

                mapPointIndexes.emplace_back(i);
                frame.mvbOutlier[i] = false;

                Eigen::Matrix<double, 2, 1> obs;
                const cv::KeyPoint &kpUn = frame.mvKeysUn[i];
                obs << kpUn.pt.x, kpUn.pt.y;
                imagePoints.col(mapPointIndexes.size() - 1) = obs;

                cv::Mat_<float> Xw = mp->GetWorldPos();
                Eigen::Matrix<double, 3, 1> mapPoint;
                mapPoint << Xw(0), Xw(1), Xw(2);
                mapPoints.col(mapPointIndexes.size() - 1) = mapPoint;

                sigma2.emplace_back(frame.mvLevelSigma2[kpUn.octave]);
            }
        }


        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};

        REPPnP::REPPnP_Options opt;
        opt.use_chi2 = true;
        //opt.chi2 = 3.84;
        opt.chi2 = 5.991;
        // Luis Ferraz, Xavier Binefa, Francesc Moreno-Noguer, Very Fast Solution to the PnP Problem with Algebraic Outlier Rejection
        // によるパラメータ
        opt.k_per_f = 0.5f / frame.fx;
        //opt.k_per_f = 1.0f / frame.fx;
        //opt.k_per_f = 1.0f;
        opt.refine = true;
        opt.nullspace_etimation_itr = nullspaceEtimationItr;
        const auto [tf, inliers] = REPPnP::REPPnP(imagePoints, mapPoints, cam, opt, sigma2);

        for (const auto idx : mapPointIndexes)
        {
            frame.mvbOutlier[idx] = true;
        }
        for (const auto idx : inliers)
        {
            frame.mvbOutlier[mapPointIndexes[idx]] = false;
        }
        frame.SetPose(Converter::toCvMat(tf.GetHomogeniousTF()));

        //std::cout << "inliers: " << inliers.size() << " nBad: " << (nInitialCorrespondences - inliers.size()) << " " << ((nInitialCorrespondences - inliers.size()) / (double)nInitialCorrespondences) << std::endl;
        return inliers.size();
    }
}
