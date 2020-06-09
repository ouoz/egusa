#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <map>
#include "Pose.h"

std::vector<Pose> loadPoses(const std::string &filename, int end)
{
    std::ifstream strm(filename);
    if (!strm.is_open())
    {
        std::cerr << "failed to open: " << filename << std::endl;
        return std::vector<Pose>();
    }

    std::vector<Pose> res;
    double t;
    cv::Matx33d rot;
    cv::Matx<double, 3, 1> trans;
    int i = 0;
    while (strm >> t
                >> rot(0, 0) >> rot(0, 1) >> rot(0, 2) >> trans(0)
                >> rot(1, 0) >> rot(1, 1) >> rot(1, 2) >> trans(1)
                >> rot(2, 0) >> rot(2, 1) >> rot(2, 2) >> trans(2) && i < end )
    {
        res.emplace_back(t, rot, trans);
        i++;
    }

    return res;
}

std::vector<Pose> loadPoses(const std::string &timesFilename, const std::string &posesFilename)
{
    std::ifstream timesStrm(timesFilename);
    if (!timesStrm.is_open())
    {
        std::cerr << "failed to open: " << timesFilename << std::endl;
        return std::vector<Pose>();
    }
    std::ifstream posesStrm(posesFilename);
    if (!posesStrm.is_open())
    {
        std::cerr << "failed to open: " << posesFilename << std::endl;
        return std::vector<Pose>();
    }

    std::vector<Pose> res;
    double t;
    cv::Matx33d rot;
    cv::Matx<double, 3, 1> trans;
    while (timesStrm >> t &&
           posesStrm >> rot(0, 0) >> rot(0, 1) >> rot(0, 2) >> trans(0)
                     >> rot(1, 0) >> rot(1, 1) >> rot(1, 2) >> trans(1)
                     >> rot(2, 0) >> rot(2, 1) >> rot(2, 2) >> trans(2))
    {
        res.emplace_back(t, rot, trans);
    }

    return res;
}

std::vector<std::pair<Pose, Pose>> syncPoses(const std::vector<Pose> &truth, const std::vector<Pose> &given)
{
    std::vector<std::pair<Pose, Pose>> res;
    res.reserve(given.size());

    size_t idx = 0;
    for (size_t i = 1; i < truth.size() && idx < given.size(); i++)
    {
        const double t1 = truth[i - 1].getTime();
        const double t2 = truth[i].getTime();
        const double g = given[idx].getTime();
        if (t2 < g)
        {
            continue;
        }

        Pose first = truth[i - (abs(t1 - g) < abs(t2 - g) ? 1 : 0)];
        res.emplace_back(first, given[idx]);

        idx++;
    }

    return res;
}

std::vector<double> rotationErrorsInDegree(const std::vector<std::pair<Pose, Pose>> &poses)
{
    std::vector<double> res;
    for (const auto &pos : poses)
    {
        cv::Mat ind = (cv::Mat_<double>(3, 1) << 0, 0, 1);
        auto truth = pos.first.getRotation();
        auto a = static_cast<cv::Mat>(truth * ind);
        auto b = static_cast<cv::Mat>(static_cast<cv::Mat>(pos.second.getRotation()) * ind);

        const auto dot = a.dot(b);
        const auto rad = (1.0 - 1e-6 < dot) ? 0.0 : std::acos(dot);
        res.emplace_back(rad / std::acos(-1.0) * 180.0);
    }

    return res;
}


double transErrors(const std::vector<std::pair<Pose, Pose>> &poses, const double scale, std::vector<double> &out)
{
    assert(poses.size() == out.size());

    double sum = 0;
    for (size_t i = 0; i < poses.size(); i++)
    {
        const auto truth = poses[i].first.getTrans();
        const auto given = scale * poses[i].second.getTrans();
        const auto tmp = truth - given;
        const double norm = std::sqrt(tmp.dot(tmp));
        out[i] = norm;
        sum += norm;
    }

    return sum;
}

std::vector<double> transErrors(const std::vector<std::pair<Pose, Pose>> &poses, double &outScale)
{
    std::vector<double> res(poses.size());

    double l = 0.00001, r = 100000;
    for (int i = 0; i < 100; i++)
    {
        const double len = (r - l) / 3.0;
        const double lscale = l + len;
        const double rscale = r - len;
        const double lsum = transErrors(poses, lscale, res);
        const double rsum = transErrors(poses, rscale, res);

        if (lsum < rsum)
        {
            r = rscale;
        }
        else
        {
            l = lscale;
        }
    }

    outScale = l;
    std::cerr << outScale << std::endl;

    return res;
}

double
saveErrors(const std::vector<Pose> &poses, const std::vector<double> &errors, const std::string &filename)
{
    std::ofstream strm(filename);
    strm.precision(8);
    double sum = 0;
    for (size_t i = 0; i < poses.size(); i++)
    {
        strm << i << " " << poses[i].getTime() << " " << errors[i] << std::endl;
        sum += errors[i];
    }

    const double ave = sum / (double) poses.size();
    std::cout << "average " << filename << ": " << ave << std::endl;
    return ave;
}

void
saveTrans(const std::vector<std::pair<Pose, Pose>> &poses, const std::string &filename, const double scale)
{
    std::ofstream strm(filename);
    strm.precision(8);
    for (size_t i = 0; i < poses.size(); i++)
    {
        const auto truth = poses[i].first.getTrans();
        const auto given = scale * poses[i].second.getTrans();
        strm << poses[i].second.getTime() << " " << truth(0) << " " << truth(1) << " " << truth(2) << " "
             << given(0) << " " << given(1) << " " << given(2) << std::endl;
    }
}

void saveStats(const double rotAve, const double transAve, const std::string &filename, const std::map<std::string, double> &values)
{
    std::ofstream strm(filename);
    strm << "trans error[m]: " << transAve << std::endl;
    strm << "rotation error[deg]: " << rotAve << std::endl;
    for (const auto& [key, value] : values)
    {
        strm << key << ": " << value << std::endl;
    }
}

double loadAverageTimes(const std::string &filename)
{
    std::ifstream strm(filename);
    if (!strm.is_open())
    {
        std::cerr << "failed to open: " << filename << std::endl;
        return 0;
    }

    double sum = 0;
    int i = 0;
    double t;
    while (strm >> t)
    {
        sum += t;
        i++;
    }

    return sum / (double) i;
}

// [average of trans error, average of rotation error, average of pnp time, ok]
std::tuple<double, double, double, bool> evaluateDirectory(const std::string &truthFilename, const std::string &directory)
{
    const auto truth = loadPoses(truthFilename, 10000);
    const auto given = loadPoses(directory + "KeyFrameTrajectory.txt", 10000);

    if (truth.empty() || given.empty())
    {
        return {0, 0, 0, false};
    }

    const auto poses = syncPoses(truth, given);

    const auto rotErrors = rotationErrorsInDegree(poses);

    double scale;
    const auto transErr = transErrors(poses, scale);

    const auto rotAve = saveErrors(given, rotErrors, directory + "rot_errors.txt");
    const auto transAve = saveErrors(given, transErr, directory + "trans_errors.txt");
    saveTrans(poses, directory + "trans.txt", scale);

    std::map<std::string, double> values;
    const auto pnpTime = loadAverageTimes(directory + "PnPTime.txt");
    values.emplace("PnPTime", pnpTime);

    saveStats(rotAve, transAve, directory + "stats.txt", values);

    return {transAve, rotAve, pnpTime, true};
}

void bulkEvaluateDirectory(const std::string &truthFilename, const std::string &directory)
{
    const auto statsFilename = "./stats.csv";
    std::ofstream str(statsFilename);
    if (!str.is_open())
    {
        std::cerr << "failed to open: " << statsFilename << std::endl;
        return;
    }

    str << "回数, 軌跡の誤差[m], 回転の誤差[deg], PnP問題の処理時間[ms]" << std::endl;

    for (int i = 1; i <= 40; ++i)
    {
        const auto resDir = directory + std::to_string(i) + "/";
        const auto [transAve, rotAve, pnpTime, ok] = evaluateDirectory(truthFilename, resDir);
        if (!ok)
        {
            continue;
        }

        str << i << ", " << transAve << ", " << rotAve << ", " << pnpTime * 1000 << std::endl;
    }
}

int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "./trajectory-evaluator truth_filename given_directory" << std::endl;
        return 1;
    }

    if (std::strcmp(argv[1], "bulk") == 0)
    {
        bulkEvaluateDirectory(argv[2], argv[3]);
        return 0;
    }

    const std::string truthFilename = argv[1];
    const std::string givenDirectory = argv[2];

    const auto [transAve, rotAve, pnpTime, ok] = evaluateDirectory(argv[1], argv[2]);
    if (!ok)
    {
        return 1;
    }

    return 0;
}
