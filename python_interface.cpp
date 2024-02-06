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
#include "PDHybridMotorControl.h"

using namespace UNITREE_LEGGED_SDK;

const std::string MOTOR_NAMES[12] =
    {
        "FR_1",
        "FR_2",
        "FR_3",
        "FL_1",
        "FL_2",
        "FL_3",
        "RR_1",
        "RR_2",
        "RR_3",
        "RL_1",
        "RL_2",
        "RL_3",

};

const double UPPER_BOUND[3] = {0.802851455917, 4.18879020479, -0.916297857297};
const double LOWER_BOUND[3] = {-0.802851455917, -1.0471975512, -2.69653369433};
const double MAX_TORQUE = 15.0;
const double SLEEP_DURATION = 0.0004;
const int POWER_LEVEL = 6;
const double MAX_INCREMENT = 0.2;

bool is_valid_observation(const LowState &state_data)
{
    double sum = 0;
    for (int i = 0; i < 12; ++i)
    {
        sum += std::fabs(state_data.motorState[i].q);
    }
    return sum > 0.0;
}

class RobotInterface
{
public:
    RobotInterface();
    LowState ReceiveObservation();
    void UpdateCommand(std::array<float, 60> motorcmd);
    void Brake();
    bool UpdateObservation();

private:
    void setup_torque_motors();
    void send_init();
    void UpdateObservationThread();
    void SendCommandThread();
    bool breaking_;
    PDHybridMotorControl torque_motors[12];
    std::thread send_command_loop_thread_;
    std::thread update_observation_loop_thread_;
    UDP udp;
    Safety safe;
    LowState state = {0};
    LowCmd cmd = {0};
    LowCmd cmd_to_send = {0};
};

RobotInterface::RobotInterface() : safe(LeggedType::A1), udp(LOWLEVEL), breaking_(true)
{
    send_init();
    setup_torque_motors();
    UpdateObservation();
    send_command_loop_thread_ = std::thread(&RobotInterface::SendCommandThread, this);
    send_command_loop_thread_.detach();
    update_observation_loop_thread_ = std::thread(&RobotInterface::UpdateObservationThread, this);
    update_observation_loop_thread_.detach(); 
}

void RobotInterface::send_init()
{
    cmd.levelFlag = LOWLEVEL;
    for (int i = 0; i < 12; i++)
    {
        cmd.motorCmd[i].mode = 0x0A;                      // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = PosStopF; // 禁止位置环
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = VelStopF; // 禁止速度环
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }
    udp.SetSend(cmd);
    udp.Send();
    std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
}

void RobotInterface::setup_torque_motors()
{
    for (int i = 0; i < 12; ++i)
    {
        torque_motors[i] = PDHybridMotorControl(MOTOR_NAMES[i], LOWER_BOUND[i % 3], UPPER_BOUND[i % 3], 0.0, 0.0, MAX_TORQUE, MAX_TORQUE, MAX_TORQUE * 0.5, MAX_INCREMENT);
    }
}

LowState RobotInterface::ReceiveObservation()
{
    LowState state_data = {0};
    udp.Recv();
    udp.GetRecv(state_data);
    return state_data;
}

bool RobotInterface::UpdateObservation()
{
    LowState state_read = ReceiveObservation();
    if (is_valid_observation(state_read))
    {
        state = state_read;
        return true;
    }
    else
    {
        std::cout << "get an invalid state reading!" << std::endl;
        return false;
    }
}

void RobotInterface::UpdateCommand(std::array<float, 60> motorcmd)
{
    breaking_ = false;
    cmd.levelFlag = LOWLEVEL;
    for (int motor_id = 0; motor_id < 12; motor_id++)
    {
        cmd.motorCmd[motor_id].mode = 0x0A;
        cmd.motorCmd[motor_id].q = motorcmd[motor_id * 5];
        cmd.motorCmd[motor_id].Kp = motorcmd[motor_id * 5 + 1];
        cmd.motorCmd[motor_id].dq = motorcmd[motor_id * 5 + 2];
        cmd.motorCmd[motor_id].Kd = motorcmd[motor_id * 5 + 3];
        cmd.motorCmd[motor_id].tau = motorcmd[motor_id * 5 + 4];
    } // q kp dq kd tau
}
void RobotInterface::UpdateObservationThread()
{
     while (true)
    {
        UpdateObservation();
        std::this_thread::sleep_for(std::chrono::duration<double>(SLEEP_DURATION));
    }
}
void RobotInterface::SendCommandThread()
{
    while (true)
    {
        
        cmd_to_send.levelFlag = LOWLEVEL;
        for (int motor_id = 0; motor_id < 12; motor_id++)
        {
            if (breaking_)
            {
                cmd_to_send.motorCmd[motor_id].mode = 0x00; // Electronic braking mode.
            }
            else
            {
                torque_motors[motor_id].kp_ = cmd.motorCmd[motor_id].Kp;
                torque_motors[motor_id].kd_ = cmd.motorCmd[motor_id].Kd;
                double torque = torque_motors[motor_id].get_torque(state.motorState[motor_id].q, state.motorState[motor_id].dq,
                                                                   cmd.motorCmd[motor_id].q, cmd.motorCmd[motor_id].dq,
                                                                   cmd.motorCmd[motor_id].tau);
                cmd_to_send.motorCmd[motor_id].mode = 0x0A;
                cmd_to_send.motorCmd[motor_id].q = PosStopF;
                cmd_to_send.motorCmd[motor_id].Kp = 0.0;
                cmd_to_send.motorCmd[motor_id].dq = VelStopF;
                cmd_to_send.motorCmd[motor_id].Kd = 0.0;
                cmd_to_send.motorCmd[motor_id].tau = torque;

            }
            
        }
        safe.PositionLimit(cmd_to_send);
        udp.SetSend(cmd_to_send);
        safe.PowerProtect(cmd_to_send, state, POWER_LEVEL);
        udp.Send();
        std::this_thread::sleep_for(std::chrono::duration<double>(SLEEP_DURATION));
    }
}

void RobotInterface::Brake()
{
    breaking_ = true;
}

namespace py = pybind11;

// TODO: Expose all of comm.h and the RobotInterface Class.

PYBIND11_MODULE(robot_interface, m)
{
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
        .def("update_command", &RobotInterface::UpdateCommand)
        .def("brake", &RobotInterface::Brake);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif

    m.attr("TEST") = py::int_(int(42));
}
