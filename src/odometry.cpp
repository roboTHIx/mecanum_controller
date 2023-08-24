#include "mecanum_controller/odometry.hpp"


namespace mecanum_controller
{

Odometry::Odometry(size_t velocity_rolling_window_size)
:   _timestamp(0.0),
    _x(0.0),
    _y(0.0),
    _heading(0.0),
    _linear_x(0.0),
    _linear_y(0.0),
    _angular(0.0),
    _wheel_separation_x(0.0),
    _wheel_separation_y(0.0),
    _front_left_wheel_radius(0.0),
    _front_right_wheel_radius(0.0),
    _rear_left_wheel_radius(0.0),
    _rear_right_wheel_radius(0.0),
    _front_left_wheel_old_pos(0.0),
    _front_right_wheel_old_pos(0.0),
    _rear_left_wheel_old_pos(0.0),
    _rear_right_wheel_old_pos(0.0),
    _velocity_rolling_window_size(velocity_rolling_window_size),
    _linear_x_accumulator(velocity_rolling_window_size),
    _linear_y_accumulator(velocity_rolling_window_size),
    _angular_accumulator(velocity_rolling_window_size)
{

}

void Odometry::init(const rclcpp::Time & time)
{
    // Reset accumulators and timestamp:
    resetAccumulators();
    _timestamp = time;
}

bool Odometry::update(double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos, const rclcpp::Time & time)
{
    // We cannot estimate the speed with very small time intervals:
    const double dt = time.seconds() - _timestamp.seconds();
    if (dt < 0.0001)
    {
        return false;  // Interval too small to integrate with
    }

    // Get current wheel joint positions:
    const double front_left_wheel_cur_pos  = front_left_pos  * _front_left_wheel_radius;
    const double front_right_wheel_cur_pos = front_right_pos * _front_right_wheel_radius;
    const double rear_left_wheel_cur_pos   = rear_left_pos   * _rear_left_wheel_radius;
    const double rear_right_wheel_cur_pos  = rear_right_pos  * _rear_right_wheel_radius;

    // Estimate velocity of wheels using old and current position:
    const double front_left_wheel_est_vel  = front_left_wheel_cur_pos  - _front_left_wheel_old_pos;
    const double front_right_wheel_est_vel = front_right_wheel_cur_pos - _front_right_wheel_old_pos;
    const double rear_left_wheel_est_vel   = rear_left_wheel_cur_pos   - _rear_left_wheel_old_pos;
    const double rear_right_wheel_est_vel  = rear_right_wheel_cur_pos  - _rear_right_wheel_old_pos;

    // Update old position with current:
    _front_left_wheel_old_pos  = front_left_wheel_cur_pos;
    _front_right_wheel_old_pos = front_right_wheel_cur_pos;
    _rear_left_wheel_old_pos   = rear_left_wheel_cur_pos;
    _rear_right_wheel_old_pos  = rear_right_wheel_cur_pos;

    updateFromVelocity(front_left_wheel_est_vel, front_right_wheel_est_vel, rear_left_wheel_est_vel, rear_right_wheel_est_vel, time);

    return true;
}

bool Odometry::updateFromVelocity(double front_left_vel, double front_right_vel, double rear_left_vel, double rear_right_vel, const rclcpp::Time & time)
{
    const double dt = time.seconds() - _timestamp.seconds();

    // Compute linear and angular diff:
    // TODO: Mecanum math for linear and angular diff
    const double linear_x = (0);
    const double linear_y = (0);
    const double angular  = (0);

    // Integrate odometry:
    integrateExact(linear_x, linear_y, angular);

    _timestamp = time;

    // Estimate speeds using a rolling mean to filter them out:
    _linear_x_accumulator.accumulate(linear_x / dt);
    _linear_y_accumulator.accumulate(linear_y / dt);
    _angular_accumulator.accumulate(angular / dt);

    _linear_x = _linear_x_accumulator.getRollingMean();
    _linear_y = _linear_y_accumulator.getRollingMean();
    _angular  = _angular_accumulator.getRollingMean();

    return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
    /// Save last linear and angular velocity:
    _linear_x  = linear_x;
    _linear_y  = linear_y;
    _angular   = angular;

    /// Integrate odometry:
    const double dt = time.seconds() - _timestamp.seconds();
    _timestamp = time;
    integrateExact(linear_x * dt, linear_y * dt, angular * dt);
}

void Odometry::resetOdometry()
{
    _x       = 0.0;
    _y       = 0.0;
    _heading = 0.0;
}

void Odometry::setWheelParams(double wheel_separation_x, double wheel_separation_y, double front_left_wheel_radius, double front_right_wheel_radius, double rear_left_wheel_radius, double rear_right_wheel_radius)
{
    _wheel_separation_x       = wheel_separation_x;
    _wheel_separation_y       = wheel_separation_y;
    _front_left_wheel_radius  = front_left_wheel_radius;
    _front_right_wheel_radius = front_right_wheel_radius;
    _rear_left_wheel_radius   = rear_left_wheel_radius;
    _rear_right_wheel_radius  = rear_right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
    _velocity_rolling_window_size = velocity_rolling_window_size;
    resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double linear_y, double angular)
{
    // TODO: check if direction calc is correct
    const double direction = _heading + angular * 0.5;

    /// Runge-Kutta 2nd order integration:
    _x       += (linear_x * cos(direction)) - (linear_y * sin(direction));
    _y       += (linear_x * sin(direction)) + (linear_y * cos(direction));
    _heading += angular;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
    if (fabs(angular) < 1e-6)
    {
        integrateRungeKutta2(linear_x, linear_y, angular);
    }
    else
    {
        /// Exact integration (should solve problems when angular is zero):
        const double heading_old = _heading;
        const double r = sqrt(linear_x + linear_x + linear_y * linear_y) / angular;
        _heading += angular;
        _x +=  r * (sin(_heading) - sin(heading_old));
        _y += -r * (cos(_heading) - cos(heading_old));
    }
}

void Odometry::resetAccumulators()
{
    _linear_x_accumulator = RollingMeanAccumulator(_velocity_rolling_window_size);
    _linear_y_accumulator = RollingMeanAccumulator(_velocity_rolling_window_size);
    _angular_accumulator  = RollingMeanAccumulator(_velocity_rolling_window_size);
}

}  // namespace mecanum_controller
