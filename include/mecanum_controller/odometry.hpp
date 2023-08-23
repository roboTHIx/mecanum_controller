#ifndef MECANUM_CONTROLLER__ODOMETRY_HPP_
#define MECANUM_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "rclcpp/time.hpp"
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace mecanum_controller
{

class Odometry
{

public:
    explicit Odometry(size_t velocity_rolling_window_size = 10);

    void init(const rclcpp::Time & time);
    bool update(double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos, const rclcpp::Time & time);
    bool updateFromVelocity(double front_left_vel, double front_right_vel, double rear_left_vel, double rear_right_vel, const rclcpp::Time & time);
    void updateOpenLoop(double linear, double angular, const rclcpp::Time & time);
    void resetOdometry();

    double getX() const { return _x; }
    double getY() const { return _y; }
    double getHeading() const { return _heading; }
    double getLinear() const { return _linear; }
    double getAngular() const { return _angular; }

    void setWheelParams(double wheel_separation, double front_left_wheel_radius, double front_right_wheel_radius, double rear_left_wheel_radius, double rear_right_wheel_radius);
    void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
    using RollingMeanAccumulator = rcppmath::RollingMeanAccumulator<double>;

    void integrateRungeKutta2(double linear, double angular);
    void integrateExact(double linear, double angular);
    void resetAccumulators();

    // Current timestamp:
    rclcpp::Time _timestamp;

    // Current pose:
    double _x;        //   [m]
    double _y;        //   [m]
    double _heading;  // [rad]

    // Current velocity:
    double _linear;   //   [m/s]
    double _angular;  // [rad/s]

    // Wheel kinematic parameters [m]:
    double _wheel_separation;
    double _front_left_wheel_radius;
    double _front_right_wheel_radius;
    double _rear_left_wheel_radius;
    double _rear_right_wheel_radius;

    // Previous wheel position/state [rad]:
    double _front_left_wheel_old_pos;
    double _front_right_wheel_old_pos;
    double _rear_left_wheel_old_pos;
    double _rear_right_wheel_old_pos;

    // Rolling mean accumulators for the linear and angular velocities:
    size_t _velocity_rolling_window_size;
    RollingMeanAccumulator _linear_accumulator;
    RollingMeanAccumulator _angular_accumulator;
};

}  // namespace mecanum_controller

#endif  // MECANUM_CONTROLLER__ODOMETRY_HPP_