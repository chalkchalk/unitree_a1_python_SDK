/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <thread>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <algorithm>

using namespace UNITREE_LEGGED_SDK;

int power_level = 4;
double max_torque = 15.0;
class RobotInterface
{
public:
    RobotInterface() : safe(LeggedType::A1), udp(LOWLEVEL){
        udp.Recv();
        udp.GetRecv(state);
        state1 = state;
        std::thread initThread(&RobotInterface::InitEnvironment, this);
        initThread.detach();  // 让线程在后台运行，不阻塞主线程
    }
    LowState ReceiveObservation();
    void SendCommand(std::array<float, 60> motorcmd);
    void Brake();
    void Initialize();
    void InitEnvironment();

    std::array<float, 3> upper_bound{{0.802851455917, 4.18879020479, -0.916297857297}};
    std::array<float, 3> lower_bound{{-0.802851455917, -1.0471975512, -2.69653369433}};

    UDP udp;
    Safety safe;
    LowState state = {0};
    LowState state1 = {0};
    LowCmd cmd = {0};
};

LowState RobotInterface::ReceiveObservation() {
    int is_zero = 1;
    do{
        for (int motor_id = 0; motor_id < 12; motor_id++) {
            if (state.motorState[motor_id].q == 0){
                is_zero = 0;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }while(is_zero == 0);

    return state;
}

void RobotInterface::SendCommand(std::array<float, 60> motorcmd) {
//    ReceiveObservation();

    cmd.levelFlag = 0xff;
    for (int motor_id = 0; motor_id < 12; motor_id++) {
    cmd.motorCmd[motor_id].mode = 0x0A;

    cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
    cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
    cmd.motorCmd[motor_id].Kd = 1.0;
    cmd.motorCmd[motor_id].tau = 0.0;

    double clip_set_1 = std::max(
                    std::min(static_cast<double>(state.motorState[motor_id].q + max_torque / cmd.motorCmd[motor_id].Kp), static_cast<double>(cmd.motorCmd[motor_id].q)),
                    state.motorState[motor_id].q - max_torque / cmd.motorCmd[motor_id].Kp);
    cmd.motorCmd[motor_id].q = std::max(std::min(clip_set_1,static_cast<double>(upper_bound[motor_id % 3])),static_cast<double>(lower_bound[motor_id % 3]));
    }

//    std::cout << "send " << std::endl;
//    safe.PositionLimit(cmd);
//    safe.PowerProtect(cmd, state, power_level);
//    udp.SetSend(cmd);
//    udp.Send();
}

void RobotInterface::InitEnvironment() {
    std::string fileName = "/home/ubuntu/Desktop/a1_real_robot/0114_traj/saved/cmd.txt";
    std::ofstream outputFile(fileName, std::ios::app);
    auto currentTime = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime.time_since_epoch());
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file: " << fileName << std::endl;
        return;
    }
    while (true) {
        udp.Recv();
        udp.GetRecv(state1);
        int flag = 1;
        for (int motor_id = 0; motor_id < 12; motor_id++) {
//            std::cout << state1.motorState[motor_id].q << std::endl;
            if (state1.motorState[motor_id].q == 0){
                flag = 0;
                break;
            }
//        std::cout << flag << std::endl;
        if (flag){
            state = state1;
            }
        }

        cmd.levelFlag = 0xff;
            std::string idx = " ";
            for (int motor_id = 0; motor_id < 12; motor_id++) {

                if (cmd.motorCmd[motor_id].Kp == 0) {
                        cmd.motorCmd[motor_id].mode = 0x00;  // Electronic braking mode.
                }
                else
                {
                    cmd.motorCmd[motor_id].mode = 0x0A;
                    double clip_set_1 = std::max(
                    std::min(static_cast<double>(state.motorState[motor_id].q + max_torque / cmd.motorCmd[motor_id].Kp), static_cast<double>(cmd.motorCmd[motor_id].q)),
                    state.motorState[motor_id].q - max_torque / cmd.motorCmd[motor_id].Kp);
                    cmd.motorCmd[motor_id].q = std::max(std::min(clip_set_1,static_cast<double>(upper_bound[motor_id % 3])),static_cast<double>(lower_bound[motor_id % 3]));

                    cmd.motorCmd[motor_id].tau = 0.0;
                    idx += std::to_string(cmd.motorCmd[motor_id].mode) + "," + std::to_string(state.motorState[motor_id].q) + "," + std::to_string(cmd.motorCmd[motor_id].q) + "," + std::to_string(cmd.motorCmd[motor_id].Kp) + "," +  std::to_string(state.motorState[motor_id].temperature) + ",";
                }

//                cmd.motorCmd[motor_id].Kp = 1.0;

            }
            safe.PositionLimit(cmd);
            safe.PowerProtect(cmd, state, power_level);
            udp.SetSend(cmd);
            udp.Send();
//            std::cout << state.motorState[0].q << std::endl;
            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(endTime.time_since_epoch());
            outputFile << std::fixed << std::setprecision(6) << duration.count() << ',' << idx << "\n";
    }
    outputFile.close();
}

void RobotInterface::Brake() {
    for (int motor_id = 0; motor_id < 12; motor_id++) {
        cmd.motorCmd[motor_id].mode = 0x00;  // Electronic braking mode.
    }
//    udp.SetSend(cmd);
//    udp.Send();
//    Write2file("brake");
}

namespace py = pybind11;

// TODO: Expose all of comm.h and the RobotInterface Class.

PYBIND11_MODULE(robot_interface, m) {
    m.doc() = R"pbdoc(
          A1 Robot Interface Python Bindings
          -----------------------
          .. currentmodule:: a1_robot_interface
          .. autosummary::
             :toctree: _generate
      )pbdoc";

    py::class_<Cartesian>(m, "Cartesian")
        .def(py::init<>())
        .def_readwrite("x", &Cartesian::x)
        .def_readwrite("y", &Cartesian::y)
        .def_readwrite("z", &Cartesian::z);

    py::class_<IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("quaternion", &IMU::quaternion)
        .def_readwrite("gyroscope", &IMU::gyroscope)
        .def_readwrite("accelerometer", &IMU::accelerometer)
        .def_readwrite("rpy", &IMU::rpy)
        .def_readwrite("temperature", &IMU::temperature);

    py::class_<LED>(m, "LED")
        .def(py::init<>())
        .def_readwrite("r", &LED::r)
        .def_readwrite("g", &LED::g)
        .def_readwrite("b", &LED::b);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("mode", &MotorState::mode)
        .def_readwrite("q", &MotorState::q)
        .def_readwrite("dq", &MotorState::dq)
        .def_readwrite("ddq", &MotorState::ddq)
        .def_readwrite("tauEst", &MotorState::tauEst)
        .def_readwrite("q_raw", &MotorState::q_raw)
        .def_readwrite("dq_raw", &MotorState::dq_raw)
        .def_readwrite("ddq_raw", &MotorState::ddq_raw)
        .def_readwrite("temperature", &MotorState::temperature)
        .def_readwrite("reserve", &MotorState::reserve);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("q", &MotorCmd::q)
        .def_readwrite("dq", &MotorCmd::dq)
        .def_readwrite("tau", &MotorCmd::tau)
        .def_readwrite("Kp", &MotorCmd::Kp)
        .def_readwrite("Kd", &MotorCmd::Kd)
        .def_readwrite("reserve", &MotorCmd::reserve);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowState::levelFlag)
        .def_readwrite("commVersion", &LowState::commVersion)
        .def_readwrite("robotID", &LowState::robotID)
        .def_readwrite("SN", &LowState::SN)
        .def_readwrite("bandWidth", &LowState::bandWidth)
        .def_readwrite("imu", &LowState::imu)
        .def_readwrite("motorState", &LowState::motorState)
        .def_readwrite("footForce", &LowState::footForce)
        .def_readwrite("footForceEst", &LowState::footForceEst)
        .def_readwrite("tick", &LowState::tick)
        .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
        .def_readwrite("reserve", &LowState::reserve)
        .def_readwrite("crc", &LowState::crc);

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &LowCmd::levelFlag)
        .def_readwrite("commVersion", &LowCmd::commVersion)
        .def_readwrite("robotID", &LowCmd::robotID)
        .def_readwrite("SN", &LowCmd::SN)
        .def_readwrite("bandWidth", &LowCmd::bandWidth)
        .def_readwrite("motorCmd", &LowCmd::motorCmd)
        .def_readwrite("led", &LowCmd::led)
        .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
        .def_readwrite("reserve", &LowCmd::reserve)
        .def_readwrite("crc", &LowCmd::crc);

    py::class_<HighState>(m, "HighState")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighState::levelFlag)
        .def_readwrite("commVersion", &HighState::commVersion)
        .def_readwrite("robotID", &HighState::robotID)
        .def_readwrite("SN", &HighState::SN)
        .def_readwrite("bandWidth", &HighState::bandWidth)
        .def_readwrite("mode", &HighState::mode)
        .def_readwrite("imu", &HighState::imu)
        .def_readwrite("forwardSpeed", &HighState::forwardSpeed)
        .def_readwrite("sideSpeed", &HighState::sideSpeed)
        .def_readwrite("rotateSpeed", &HighState::rotateSpeed)
        .def_readwrite("bodyHeight", &HighState::bodyHeight)
        .def_readwrite("updownSpeed", &HighState::updownSpeed)
        .def_readwrite("forwardPosition", &HighState::forwardPosition)
        .def_readwrite("sidePosition", &HighState::sidePosition)
        .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
        .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
        .def_readwrite("footForce", &HighState::footForce)
        .def_readwrite("footForceEst", &HighState::footForceEst)
        .def_readwrite("tick", &HighState::tick)
        .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
        .def_readwrite("reserve", &HighState::reserve)
        .def_readwrite("crc", &HighState::crc);

    py::class_<HighCmd>(m, "HighCmd")
        .def(py::init<>())
        .def_readwrite("levelFlag", &HighCmd::levelFlag)
        .def_readwrite("commVersion", &HighCmd::commVersion)
        .def_readwrite("robotID", &HighCmd::robotID)
        .def_readwrite("SN", &HighCmd::SN)
        .def_readwrite("bandWidth", &HighCmd::bandWidth)
        .def_readwrite("mode", &HighCmd::mode)
        .def_readwrite("forwardSpeed", &HighCmd::forwardSpeed)
        .def_readwrite("sideSpeed", &HighCmd::sideSpeed)
        .def_readwrite("rotateSpeed", &HighCmd::rotateSpeed)
        .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
        .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
        .def_readwrite("yaw", &HighCmd::yaw)
        .def_readwrite("pitch", &HighCmd::pitch)
        .def_readwrite("roll", &HighCmd::roll)
        .def_readwrite("led", &HighCmd::led)
        .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
        .def_readwrite("AppRemote", &HighCmd::AppRemote)
        .def_readwrite("reserve", &HighCmd::reserve)
        .def_readwrite("crc", &HighCmd::crc);

    py::class_<UDPState>(m, "UDPState")
        .def(py::init<>())
        .def_readwrite("TotalCount", &UDPState::TotalCount)
        .def_readwrite("SendCount", &UDPState::SendCount)
        .def_readwrite("RecvCount", &UDPState::RecvCount)
        .def_readwrite("SendError", &UDPState::SendError)
        .def_readwrite("FlagError", &UDPState::FlagError)
        .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
        .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);

    py::class_<RobotInterface>(m, "RobotInterface")
        .def(py::init<>())
        .def("receive_observation", &RobotInterface::ReceiveObservation)
        .def("send_command", &RobotInterface::SendCommand)
        .def("brake", &RobotInterface::Brake);

    #ifdef VERSION_INFO
      m.attr("__version__") = VERSION_INFO;
    #else
      m.attr("__version__") = "dev";
    #endif

      m.attr("TEST") = py::int_(int(42));

}
