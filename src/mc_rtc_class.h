#ifndef HOGE_H
#define HOGE_H

#include <Eigen/Eigen>

typedef Eigen::Vector3d vec3;
typedef Eigen::Vector4d vec4;

class PyMC_RTC_Controller
{
public:
    PyMC_RTC_Controller();
    ~PyMC_RTC_Controller();

    ////
public:
    bool checkControllerVersion() const;
    void checkModel(std::string &result) const;
    size_t numJoints() const;
    void setRunning(bool run);

    ////
public:
    bool initController(const std::vector<double> &in_vec);
    bool setSensorPose(const vec3 &pos, const vec4 &quat);
    bool setSensorVelocity(const vec3 &vel, const vec3 &rate);
    bool setSensorAcceleration(const vec3 &acc);
    bool setEncoderValues(const std::vector<double> &in_vec);
    bool setEncoderVelocities(const std::vector<double> &in_vec);
    bool setWrenches(const std::vector<std::string> &names, const std::vector<vec3> &f, const::std::vector<vec3> &tau);
    bool setJointTorques(const std::vector<double> &in_vec);

    ////
public:
    bool getTargetValues(std::vector<double> &in_vec);

    ////
public:
    bool run();

private:
    class Impl;
    Impl *impl;
};

#endif
