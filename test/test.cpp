
#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <fstream>
#include <iomanip>

namespace
{
  bool init_controller()
  {
    if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
    {
      mc_rtc::log::error("MCControl was compiled with {}  but mc_rtc is at version {}, you might face subtle issues or unexpected crashes, please recompile mc_openrtm", mc_rtc::MC_RTC_VERSION, mc_rtc::version());
      return false;
    }
    return true;
  }
}

int main(int argc, char **argv)
{
#if 1
    Eigen::Vector4d v(0, 0, 0, 1.0);
    {
        Eigen::Quaterniond quat(v);
        std::cerr << quat.x() << ", " <<  quat.y() << ", " <<  quat.z() << ", " <<  quat.w() << std::endl;
    }
    {
        Eigen::Quaterniond quat(0, 0, 0, 1.0);
        std::cerr << quat.x() << ", " <<  quat.y() << ", " <<  quat.z() << ", " <<  quat.w() << std::endl;
    }
#endif
    bool res = init_controller();

    if (res) {
        std::cerr << "start" << std::endl;
        mc_control::MCGlobalController controller;
        std::cerr << "rm" << std::endl;
        const auto & rm = controller.robot().module();
        std::cerr << "rjo" << std::endl;
        const auto & rjo = controller.robot().refJointOrder();
        ////
        std::cerr << "<ref:joint>" << std::endl;
        for(size_t i = 0; i < rjo.size(); ++i) {
            std::cerr << rjo[i] << std::endl;
        }
        std::cerr << std::endl;
        ////
        std::cerr << "<bodies>" << std::endl;
        const auto mb = controller.robot().mb();
        for(const auto & b : mb.bodies()) {
            std::cerr << b.name() << std::endl;
        }
        std::cerr << std::endl;
        ////
        std::cerr << "<forceSensors>" << std::endl;
        for(const auto & fs : rm.forceSensors()) {
            std::cerr << fs.name() << std::endl;
        }
        std::cerr << std::endl;
        ////
        std::cerr << "<jointSensors>" << std::endl;
        for(const auto & js : rm.jointSensors()) {
            std::cerr << js.joint();
            auto rjo_it = std::find(rjo.begin(), rjo.end(), js.joint());
            if(rjo_it != rjo.end()) {
                int rjo_idx = std::distance(rjo.begin(), rjo_it);
                std::cerr << " / " << rjo_idx << std::endl;
            } else {
                std::cerr << std::endl;
            }
        }
        std::cerr << "<start>" << std::endl;
        ////
        //controller.init(qInit);
        controller.running = true;
        for (int i = 0; i < 10; i++) {
            if (controller.running) {
                //
                // set values
                //
                bool res = controller.run();
                std::cerr << "  run(" << res << ") " << i << std::endl;
                // get values
                const auto & ref_joint_order = controller.robot().refJointOrder();
                const auto & qOut = controller.robot().mbc().q;
                std::vector<double> qref;
                std::cerr << "    ";
                for(size_t i = 0; i < ref_joint_order.size(); ++i) {
                    auto jIdx = controller.robot().jointIndexInMBC(i);
                    std::cerr << jIdx << " / ";
                    if(jIdx != -1 && qOut[static_cast<size_t>(jIdx)].size() == 1) {
                        //m_qOut.data[i] = qOut[ static_cast<size_t>(jIdx) ][0];
                        std::cerr << qOut[ static_cast<size_t>(jIdx) ][0] << ", ";
                    } else {
                        //m_qOut.data[i] = m_qIn.data[i];//// same value
                    }
                }
                std::cerr << std::endl;
            }
        }
#if 0
        controller.init(qInit);
        controller.run();
        // qInit: std::vector<double>
        ////
        controller.setSensorOrientation(Eigen::Quaterniond(mc_rbdyn::rpyToMat(rpyIn)).normalized());
        // Eigen::Vector3d rpyIn = Eigen::Vector3d::Zero();
        controller.setSensorPosition(pIn);
        // Eigen::Vector3d pIn = Eigen::Vector3d::Zero();
        controller.setSensorAngularVelocity(rateIn);
        // Eigen::Vector3d rateIn = Eigen::Vector3d::Zero();
        controller.setSensorLinearVelocity(velIn.linear());
        // sva::MotionVecd velIn;
        controller.setSensorLinearAcceleration(accIn);
        // Eigen::Vector3d accIn = Eigen::Vector3d::Zero();
        ////
        controller.setEncoderValues(qIn);
        // std::vector<double> qIn;
        controller.setEncoderVelocities(alphaIn);
        // std::vector<double> alphaIn;
        ////
        controller.setWrenches(m_wrenches);
        // std::map<std::string, sva::ForceVecd> m_wrenches;
        controller.setJointTorques(taucIn);
        // std::vector<double> taucIn;
        controller.setJointMotorTemperatures(motorTempIn);
        // std::map<std::string, double> motorTempIn;
        for (int i = 0; i < 1; i++) {
            const auto & ref_joint_order = controller.robot().refJointOrder();
            const auto & qOut = controller.robot().mbc().q;
            std::vector<double> qref;
            for(size_t i = 0; i < ref_joint_order.size(); ++i) {
                auto jIdx = controller.robot().jointIndexInMBC(i);
                if(jIdx != -1 && qOut[static_cast<size_t>(jIdx)].size() == 1) {
                    //m_qOut.data[i] = qOut[ static_cast<size_t>(jIdx) ][0];
                } else {
                    //m_qOut.data[i] = m_qIn.data[i];//// same value
                }
            }
        }
#endif
        ////
        std::cerr << "<finish>" << std::endl;
    }

}
#if 0
      {
        m_qOut.data.length(m_qIn.data.length());
        const auto & ref_joint_order = controller.robot().refJointOrder();
        const auto & qOut = controller.robot().mbc().q;
        for(size_t i = 0; i < ref_joint_order.size(); ++i) {
            auto jIdx = controller.robot().jointIndexInMBC(i);
            if(jIdx != -1 && qOut[static_cast<size_t>(jIdx)].size() == 1) {
                m_qOut.data[i] = qOut[ static_cast<size_t>(jIdx) ][0];
            } else {
                m_qOut.data[i] = m_qIn.data[i];//// same value
            }
        }
        /* FIXME Correction RPY convention here? */
        const auto & ff_state = qOut[0];
        if(ff_state.size())
        {
          Eigen::Vector3d rpyOut = Eigen::Quaterniond(ff_state[0], ff_state[1], ff_state[2], ff_state[3])
                                       .toRotationMatrix()
                                       .eulerAngles(2, 1, 0);
          m_rpyOut.data.r = rpyOut[2];
          m_rpyOut.data.p = rpyOut[1];
          m_rpyOut.data.y = rpyOut[0];

          m_pOut.data.x = ff_state[4];
          m_pOut.data.y = ff_state[5];
          m_pOut.data.z = ff_state[6];
        }
      }
#endif
