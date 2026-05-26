/*
 * Copyright (c) 2021, Kim Lindberg Schwaner <kils@mmmi.sdu.dk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "hd++.h"

#include <iomanip>
#include <sstream>

namespace hd {

void error_check()
{
    auto e = hdGetError();

    if (e.errorCode != HD_SUCCESS) {
        std::stringstream ss;
        ss << hdGetErrorString(e.errorCode)
           << " ("
           << "0x" << std::hex << std::setfill('0') << std::setw(4) << e.errorCode
           << ")";
        throw std::runtime_error(ss.str());
    }
}

Device::Device(const std::string& name)
    : handle_(HD_INVALID_HANDLE)
{
    if (name.empty())
        handle_ = hdInitDevice(HD_DEFAULT_DEVICE);
    else
        handle_ = hdInitDevice(name.c_str());

    error_check();
}

Device::~Device()
{
    if (handle_ != HD_INVALID_HANDLE) {
        hdDisableDevice(handle_);
        error_check();
    }
}

Device::Device(Device&& other)
    : handle_(std::move(other.handle_))
{
    other.handle_ = HD_INVALID_HANDLE;
}

Device& Device::operator=(Device&& rhs)
{
    handle_ = std::move(rhs.handle_);
    rhs.handle_ = HD_INVALID_HANDLE;
    return *this;
}

Device::handle_t Device::handle() const
{
    return handle_;
}

std::string Device::model_type() const
{
    make_current();
    return hdGetString(HD_DEVICE_MODEL_TYPE);
}

std::string Device::driver_version() const
{
    make_current();
    return hdGetString(HD_DEVICE_DRIVER_VERSION);
}

std::string Device::vendor() const
{
    make_current();
    return hdGetString(HD_DEVICE_VENDOR);
}

std::string Device::serial_number() const
{
    make_current();
    return hdGetString(HD_DEVICE_SERIAL_NUMBER);
}

void Device::make_current() const
{
    hdMakeCurrentDevice(handle_);
    error_check();
}

void Device::set_force_enabled(bool enable)
{
    make_current();

    if (enable)
        hdEnable(HD_FORCE_OUTPUT);
    else
        hdDisable(HD_FORCE_OUTPUT);
}

std::array<bool, 4> Device::buttons_current() const
{
    HDint buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    return {(buttons & HD_DEVICE_BUTTON_1) != 0,
            (buttons & HD_DEVICE_BUTTON_2) != 0,
            (buttons & HD_DEVICE_BUTTON_3) != 0,
            (buttons & HD_DEVICE_BUTTON_4) != 0};
}
std::array<bool, 4> Device::buttons_last() const
{
    HDint buttons = 0;
    hdGetIntegerv(HD_LAST_BUTTONS, &buttons);
    return {(buttons & HD_DEVICE_BUTTON_1) != 0,
            (buttons & HD_DEVICE_BUTTON_2) != 0,
            (buttons & HD_DEVICE_BUTTON_3) != 0,
            (buttons & HD_DEVICE_BUTTON_4) != 0};
}

bool Device::inkwell_switch_current() const
{
    HDboolean inkwell_switch = 0;
    hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &inkwell_switch);
    return (inkwell_switch == 0); // is 0 when placed in inkwell, 1 otherwise
}

bool Device::inkwell_switch_last() const
{
    HDboolean inkwell_switch = 0;
    hdGetBooleanv(HD_LAST_INKWELL_SWITCH, &inkwell_switch);
    return (inkwell_switch == 0); // is 0 when placed in inkwell, 1 otherwise
}

Vector3d Device::position_current() const
{
    Vector3d position;
    hdGetDoublev(HD_CURRENT_POSITION, position.data());
    return position;
}

Vector3d Device::position_last() const
{
    Vector3d position;
    hdGetDoublev(HD_LAST_POSITION, position.data());
    return position;
}

Transform Device::transform_current() const
{
    // Transform is in column-major format (same as Eigen default)
    Transform transform;
    hdGetDoublev(HD_CURRENT_TRANSFORM, transform.data());
    return transform;
}

Transform Device::transform_last() const
{
    Transform transform;
    hdGetDoublev(HD_LAST_TRANSFORM, transform.data());
    return transform;
}

Vector6d Device::twist_current() const
{
    Vector6d twist;
    hdGetDoublev(HD_CURRENT_VELOCITY, twist.data());
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, std::next(twist.data(), 3));
    return twist;
}

Vector6d Device::twist_last() const
{
    Vector6d twist;
    hdGetDoublev(HD_LAST_VELOCITY, twist.data());
    hdGetDoublev(HD_LAST_ANGULAR_VELOCITY, std::next(twist.data(), 3));
    return twist;
}

Vector6d Device::wrench_current() const
{
    Vector6d wrench;
    hdGetDoublev(HD_CURRENT_FORCE, wrench.data());
    hdGetDoublev(HD_CURRENT_TORQUE, std::next(wrench.data(), 3));
    return wrench;
}

Vector6d Device::wrench_last() const
{
    Vector6d wrench;
    hdGetDoublev(HD_LAST_FORCE, wrench.data());
    hdGetDoublev(HD_LAST_TORQUE, std::next(wrench.data(), 3));
    return wrench;
}

Matrix6d Device::jacobian_current() const
{
    // Jacobian is in row-major format, or it operates on row-vectors. In any
    // case, we convert it to column-major format as preferred by Eigen.
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian;
    hdGetDoublev(HD_CURRENT_JACOBIAN, jacobian.data());
    return jacobian;
}

Matrix6d Device::jacobian_last() const
{
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> jacobian;
    hdGetDoublev(HD_LAST_JACOBIAN, jacobian.data());
    return jacobian;
}

Vector6d Device::joint_angles_current() const
{
    Vector6d angles;
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, angles.data());
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, std::next(angles.data(), 3));

    // For some reason the HD API has added the the angle of the 2nd joint
    // to that of the 3rd joint, so correct that here
    angles[2] -= angles[1];

    return angles;
}

Vector6d Device::joint_angles_last() const
{
    Vector6d angles;
    hdGetDoublev(HD_LAST_JOINT_ANGLES, angles.data());
    hdGetDoublev(HD_LAST_GIMBAL_ANGLES, std::next(angles.data(), 3));
    angles[2] -= angles[1];
    return angles;
}

Vector6d Device::joint_torque_current() const
{
    Vector6d torque;
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, torque.data());
    hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, std::next(torque.data(), 3));
    return torque;
}

Vector6d Device::joint_torque_last() const
{
    Vector6d torque;
    hdGetDoublev(HD_LAST_JOINT_TORQUE, torque.data());
    hdGetDoublev(HD_LAST_GIMBAL_TORQUE, std::next(torque.data(), 3));
    return torque;
}

void Device::set_current_wrench(const Vector6d& wrench)
{
    hdSetDoublev(HD_CURRENT_FORCE, wrench.head<3>().data());
}

bool Device::calibration_ok() const
{
    auto status = hdCheckCalibration();
    error_check();
    return (status == HD_CALIBRATION_OK);
}

Scheduler::Scheduler()
    : is_running_(false)
{
}

Scheduler::~Scheduler()
{
    stop();
    unschedule_all();
}

void Scheduler::start(unsigned desired_rate_hz)
{
    if (is_running_)
        return;

    hdSetSchedulerRate(desired_rate_hz);
    error_check();
    hdStartScheduler();
    error_check();
    is_running_ = true;
}

void Scheduler::stop()
{
    if (!is_running_)
        return;

    hdStopScheduler();
    error_check();
    is_running_ = false;
}

bool Scheduler::is_running() const
{
    return is_running_;
}

void Scheduler::unschedule(handle_t handle)
{
    hdUnschedule(handle);
    error_check();
    operations_.remove_if([handle](const auto& op) { return op.handle == handle; });
}

void Scheduler::unschedule_all()
{
    for (const auto& op : operations_) {
        hdUnschedule(op.handle);
        error_check();
    }

    operations_.clear();
}

void Scheduler::schedule_synchronous(HDSchedulerCallback cb, void* data, HDushort prio)
{
    hdScheduleSynchronous(cb, data, prio);
    error_check();
}

Scheduler::handle_t Scheduler::schedule_asynchronous(HDSchedulerCallback cb, void* data, HDushort prio)
{
    auto handle = hdScheduleAsynchronous(cb, data, prio);
    error_check();
    return handle;
}

ScopedFrame::ScopedFrame(const Device& device)
    : handle_(HD_INVALID_HANDLE)
{
    hdBeginFrame(device.handle());
    error_check();
    handle_ = device.handle();
}

ScopedFrame::~ScopedFrame()
{
    if (handle_ != HD_INVALID_HANDLE) {
        hdEndFrame(handle_);
        error_check();
    }
}

ScopedFrame::ScopedFrame(ScopedFrame&& other)
    : handle_(other.handle_)
{
    other.handle_ = HD_INVALID_HANDLE;
}

ScopedFrame& ScopedFrame::operator=(ScopedFrame&& rhs)
{
    handle_ = rhs.handle_;
    rhs.handle_ = HD_INVALID_HANDLE;
    return *this;
}

} // namespace hd
