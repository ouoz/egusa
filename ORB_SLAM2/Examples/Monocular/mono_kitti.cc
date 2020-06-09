/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include <include/Optimizer.h>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void SaveTrackingTimes(const std::vector<float> times)
{
    constexpr auto filename = "TrackTime.txt";
    std::ofstream str(filename);
    if (!str.is_open())
    {
        std::cerr << "failed to open: " << filename << std::endl;
        return;
    }

    double sum = 0.0;
    str.precision(10);
    for (const auto t : times)
    {
        str << t << std::endl;
        sum += t;
    }
    std::cout << "Track time ave: " << (sum / (double) times.size()) << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 6)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence nullspace_etimation_itr lm|epnp|reppnp" << endl;
        return 1;
    }

    ORB_SLAM2::Optimizer::mNullspaceEtimationItr = std::atoi(argv[4]);
    std::cout << "nullspace etimation itr: " << ORB_SLAM2::Optimizer::mNullspaceEtimationItr << std::endl;

    {
        const auto pnpAlgorithm = argv[5];
        const auto ok = ORB_SLAM2::Optimizer::SetAlgorithm(pnpAlgorithm);
        if (!ok)
        {
            std::cerr << "invalid PnP algorithm: " << pnpAlgorithm << std::endl;
            return 1;
        }
        std::cout << "PnP algorithm: " << pnpAlgorithm << std::endl;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    bool main_error = false;
    std::thread runthread(
        [&]()
        {  // Start in new thread
            // Main loop
            cv::Mat im;
            for (int ni = 0; ni < nImages; ni++)
            {
                // Read image from file
                im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
                double tframe = vTimestamps[ni];

                if (im.empty())
                {
                    cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
                    main_error = true;
                    return;
                }

#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
                std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

                // Pass the image to the SLAM system
                SLAM.TrackMonocular(im, tframe);

#ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
                std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

                double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(
                    t2 - t1).count();

                vTimesTrack[ni] = ttrack;

                // Wait to load the next frame
                double T = 0;
                if (ni < nImages - 1)
                    T = vTimestamps[ni + 1] - tframe;
                else if (ni > 0)
                    T = tframe - vTimestamps[ni - 1];

                if (ttrack < T)
                    usleep((T - ttrack) * 1e6);
            }

            SLAM.Shutdown();
        });

    // Start the visualization thread
    SLAM.StartViewer();

    cout << "Viewer started, waiting for thread." << endl;
    runthread.join();
    if (main_error != 0)
        return main_error;
    cout << "Tracking thread joined..." << endl;

    // Stop all threads
    // SLAM.Shutdown();

    // Tracking time statistics
    SaveTrackingTimes(vTimesTrack);
    sort(vTimesTrack.begin(), vTimesTrack.end());
    float totaltime = 0;
    for (int ni = 0; ni < nImages; ni++)
    {
        totaltime += vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
    cout << "mean tracking time: " << totaltime / nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while (!fTimes.eof())
    {
        string s;
        getline(fTimes, s);
        if (!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for (int i = 0; i < nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
