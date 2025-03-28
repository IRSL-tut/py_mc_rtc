#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <vector>

#include "mc_rtc_class.h"

class PyMC_RTC_Controller::Impl
{
public:
    Impl() : verbose(false) {
        numJoints = controller.robot().refJointOrder().size();
    };
public:
    mc_control::MCGlobalController controller;
    size_t numJoints;

    inline bool checkSize(const std::vector<double> &vec) {
        return (vec.size() == numJoints);
    }
    bool verbose;
};

PyMC_RTC_Controller::PyMC_RTC_Controller () {
    impl = new PyMC_RTC_Controller::Impl();
};

PyMC_RTC_Controller::~PyMC_RTC_Controller () {
    if (!!impl) {
        delete impl;
    }
};

////
bool PyMC_RTC_Controller::checkControllerVersion() const
{
    if(mc_rtc::MC_RTC_VERSION != mc_rtc::version()) {
        mc_rtc::log::error("MCControl was compiled with {}  but mc_rtc is at version {}, you might face subtle issues or unexpected crashes, please recompile mc_openrtm", mc_rtc::MC_RTC_VERSION, mc_rtc::version());
        return false;
    }
    return true;
}
void PyMC_RTC_Controller::checkModel(std::string &result) const
{
    std::ostringstream strm;
    strm << "start" << std::endl;
    strm << "rm" << std::endl;
    const auto & rm = impl->controller.robot().module();
    strm << "rjo" << std::endl;
    const auto & rjo = impl->controller.robot().refJointOrder();
    ////
    strm << "<ref:joint>" << std::endl;
    for(size_t i = 0; i < rjo.size(); ++i) {
        strm << rjo[i] << std::endl;
    }
    strm << std::endl;
    ////
    strm << "<bodies>" << std::endl;
    const auto mb = impl->controller.robot().mb();
    for(const auto & b : mb.bodies()) {
        strm << b.name() << std::endl;
    }
    strm << std::endl;
    ////
    strm << "<forceSensors>" << std::endl;
    for(const auto & fs : rm.forceSensors()) {
        strm << fs.name() << std::endl;
    }
    strm << std::endl;
    ////
    strm << "<jointSensors>" << std::endl;
    for(const auto & js : rm.jointSensors()) {
        strm << js.joint();
        auto rjo_it = std::find(rjo.begin(), rjo.end(), js.joint());
        if(rjo_it != rjo.end()) {
            int rjo_idx = std::distance(rjo.begin(), rjo_it);
            strm << " / " << rjo_idx << std::endl;
        } else {
            strm << std::endl;
        }
    }
    result = strm.str();
}
size_t PyMC_RTC_Controller::numJoints() const
{
    return impl->controller.robot().refJointOrder().size();
}
void PyMC_RTC_Controller::setRunning(bool run)
{
    impl->controller.running = run;
}
#define pv3(vec) vec(0) << ", " << vec(1) << ", " << vec(2)
////
bool PyMC_RTC_Controller::initController(const std::vector<double> &in_vec)
{
    if (impl->checkSize(in_vec)) {
        impl->controller.init(in_vec);
        return true;
    }
    return false;
}
bool PyMC_RTC_Controller::setSensorPose(const vec3 &pos, const vec4 &quat)
{
    Eigen::Quaterniond qIn(quat);
    if (impl->verbose) {
        std::cerr << "rpyIn(q) : " << qIn.x() << ", " << qIn.y() << ", " << qIn.z() << ", " << qIn.w() << std::endl;
        std::cerr << "pIn : " << pv3(pos) << std::endl;
    }
    impl->controller.setSensorOrientation(qIn);
    impl->controller.setSensorPosition(pos);
    return true;
}
bool PyMC_RTC_Controller::setSensorLinearVelocity(const vec3 &vel)
{
    impl->controller.setSensorLinearVelocity(vel);
    return true;
}
bool PyMC_RTC_Controller::setSensorAngularVelocity(const vec3 &rate)
{
    if (impl->verbose) {
        std::cerr << "rateIn : " << pv3(rate) << std::endl;
    }
    impl->controller.setSensorAngularVelocity(rate);
    return true;
}
bool PyMC_RTC_Controller::setSensorVelocity(const vec3 &vel, const vec3 &rate)
{
    impl->controller.setSensorLinearVelocity(vel);
    impl->controller.setSensorAngularVelocity(rate);
    return true;
}
bool PyMC_RTC_Controller::setSensorAcceleration(const vec3 &acc)
{
    if (impl->verbose) {
        std::cerr << "accIn : " << pv3(acc) << std::endl;
    }
    impl->controller.setSensorLinearAcceleration(acc);
    return true;
}
bool PyMC_RTC_Controller::setEncoderValues(const std::vector<double> &in_vec)
{
    if (impl->checkSize(in_vec)) {
        if (impl->verbose) {
            std::cerr << "qIn : ";
            for (size_t i = 0; i < in_vec.size(); i++) {
                std::cerr << in_vec[i] << " ";
            }
            std::cerr << std::endl;
        }
        impl->controller.setEncoderValues(in_vec);
        return true;
    }
    return false;
}
bool PyMC_RTC_Controller::setEncoderVelocities(const std::vector<double> &in_vec)
{
    if (impl->checkSize(in_vec)) {
        impl->controller.setEncoderVelocities(in_vec);
        return true;
    }
    return false;
}
bool PyMC_RTC_Controller::setJointTorques(const std::vector<double> &in_vec)
{
    if (impl->checkSize(in_vec)) {
        impl->controller.setJointTorques(in_vec);
        return true;
    }
    return false;
}
bool PyMC_RTC_Controller::setWrenches(const std::vector<std::string> &names, const std::vector<vec3> &f, const::std::vector<vec3> &tau)
{
    if (names.size() == f.size() && names.size() == tau.size()) {
        std::map<std::string, sva::ForceVecd> m_wrenches;
        for (size_t i = 0; i < names.size(); i++) {
            const auto wname = names[i];
            m_wrenches[ wname ] = sva::ForceVecd(tau[i], f[i]);
        }
        if (impl->verbose) {
            for(int i = 0; i < 2; i++) {
                std::cerr << "f" << i << " : " << pv3(m_wrenches[names[i]].force()) << std::endl;
                std::cerr << "tau" << i << " : " << pv3(m_wrenches[names[i]].couple()) << std::endl;
            }
        }
        impl->controller.setWrenches(m_wrenches);
        return true;
    }
    return false;
}

////
bool PyMC_RTC_Controller::getTargetValues(std::vector<double> &result)
{
    if (!impl->checkSize(result)) {
        result.resize(impl->numJoints);
    }
    const auto & ref_joint_order = impl->controller.robot().refJointOrder();
    const auto & qOut = impl->controller.robot().mbc().q;
    for(size_t i = 0; i < ref_joint_order.size(); ++i) {
        auto jIdx = impl->controller.robot().jointIndexInMBC(i);
        if(jIdx != -1 && qOut[static_cast<size_t>(jIdx)].size() == 1) {
            result[i] = qOut[ static_cast<size_t>(jIdx) ][0];
        } else {
            //// warning
            std::cerr << "WARN: i = " << i <<", jIdx = " << jIdx << ", qOut.size = " << qOut[static_cast<size_t>(jIdx)].size() << std::endl;
        }
    }
    return true;
}
bool PyMC_RTC_Controller::getPose(std::vector<double> &result)
{
    if (result.size() != 7) {
        result.resize(7);
    }
    const auto & qOut = impl->controller.robot().mbc().q;
    const auto & ff_state = qOut[0];
    if(ff_state.size()) {
        Eigen::Quaterniond q(ff_state[0], ff_state[1], ff_state[2], ff_state[3]);
        result[3] = q.x();
        result[4] = q.y();
        result[5] = q.z();
        result[6] = q.w();
        //
        result[0] = ff_state[4];
        result[1] = ff_state[5];
        result[2] = ff_state[6];
        return true;
    }
    return false;
}

////
bool PyMC_RTC_Controller::run()
{
    return impl->controller.run();
}

////
void PyMC_RTC_Controller::setVerbose(bool on)
{
    impl->verbose = on;
}
