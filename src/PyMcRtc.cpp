#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h> //??

#include <vector>
#include <string>
#include <memory> // shared_ptr
#include <iostream>
#include <sstream>
#include <exception>
#include <stdexcept>

#include "mc_rtc_class.h"

namespace py = pybind11;

typedef Eigen::Ref<Eigen::Vector3d> refVec3;
typedef Eigen::Ref<Eigen::Vector4d> refVec4;

PYBIND11_MODULE(PyMcRtc, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: IRSL pybind_sample

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    py::class_< PyMC_RTC_Controller > mc_rtc_cls(m, "MCController");
    mc_rtc_cls.def(py::init<>())
    ////
    .def("checkControllerVersion", &PyMC_RTC_Controller::checkControllerVersion)
    .def("checkModel", [](PyMC_RTC_Controller &self) {
        std::string res; self.checkModel(res); return res;
    })
    .def_property_readonly("numJoints", &PyMC_RTC_Controller::numJoints)
    .def("setRunning", &PyMC_RTC_Controller::setRunning)
    ////
    .def("initController", [](PyMC_RTC_Controller &self, const py::array_t<double> &npa) {
        std::vector<double> invec;
        if (npa.ndim() != 1) {
            return false;
        }
        auto r = npa.unchecked<1>(); // for read
        invec.resize(npa.shape(0));
        for (py::ssize_t i = 0; i < npa.shape(0); i++) {
            invec[i] = r(i);
        }
        return self.initController(invec);
    })
    .def("setSensorPose", [](PyMC_RTC_Controller &self, const refVec3 pos, const refVec4 quat) {
        return self.setSensorPose(pos, quat);
    })
    .def("setSensorVelocity", [](PyMC_RTC_Controller &self, const refVec3 vel, const refVec3 rate) {
        return self.setSensorVelocity(vel, rate);
    })
    .def("setSensorAcceleration", [](PyMC_RTC_Controller &self, const refVec3 acc) {
        return self.setSensorAcceleration(acc);
    })
    .def("setEncoderValues", [](PyMC_RTC_Controller &self, const py::array_t<double> &npa) {
        std::vector<double> invec;
        if (npa.ndim() != 1) {
            return false;
        }
        auto r = npa.unchecked<1>();
        invec.resize(npa.shape(0));
        for (py::ssize_t i = 0; i < npa.shape(0); i++) {
            invec[i] = r(i);
        }
        return self.setEncoderValues(invec);
    })
    .def("setEncoderVelocities", [](PyMC_RTC_Controller &self, const py::array_t<double> &npa) {
        std::vector<double> invec;
        if (npa.ndim() != 1) {
            return false;
        }
        auto r = npa.unchecked<1>();
        invec.resize(npa.shape(0));
        for (py::ssize_t i = 0; i < npa.shape(0); i++) {
            invec[i] = r(i);
        }
        return self.setEncoderVelocities(invec);
    })
    .def("setWrenches", &PyMC_RTC_Controller::setWrenches)
    .def("setJointTorques", [](PyMC_RTC_Controller &self, const py::array_t<double> &npa) {
        std::vector<double> invec;
        if (npa.ndim() != 1) {
            return false;
        }
        auto r = npa.unchecked<1>();
        invec.resize(npa.shape(0));
        for (py::ssize_t i = 0; i < npa.shape(0); i++) {
            invec[i] = r(i);
        }
        return self.setJointTorques(invec);
    })
    ////
    .def("getTargetValues", [](PyMC_RTC_Controller &self) {
        std::vector<double> invec;
        self.getTargetValues(invec);
#if 1 // return numpy.array
        py::array_t<double> npa;
        npa.resize({invec.size()});
        auto r = npa.mutable_unchecked<1>();
        for (py::ssize_t i = 0; i < npa.shape(0); i++) {
            r(i) = invec[i];
        }
        return npa;
#else
        return invec;
#endif
    })
    ////
    .def("run", &PyMC_RTC_Controller::run)
    ;
}
