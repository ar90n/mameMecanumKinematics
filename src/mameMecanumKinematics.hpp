#ifndef MAMEMECANUMKINEMATICS_HPP_
#define MAMEMECANUMKINEMATICS_HPP_

namespace mameMecanumKinematics {
struct WheelAngularVelocity
{
  double front_left;
  double front_right;
  double rear_left;
  double rear_right;
};

struct Twist
{
  double linear_x;
  double linear_y;
  double angular_z;
};

class MameMecanumKinematics final
{
public:
  /**
   * @brief Constructs a MecanumKinematics object with specified wheel parameters.
   *
   * @param wheel_radius Radius of the wheels (meters).
   * @param wheel_base Distance between front and rear wheels (meters).
   * @param track_width Distance between left and right wheels (meters).
   */
  explicit MameMecanumKinematics(double wheel_radius, double wheel_base, double track_width)
    : wheel_radius_(wheel_radius)
    , wheel_base_(wheel_base)
    , track_width_(track_width)
  {
  }

  /**
   * @brief Default destructor.
   */
  ~MameMecanumKinematics() = default;

  /**
   * @brief Computes the wheel angular velocities given a desired robot twist.
   *
   * Inverse Kinematics: Twist -> Wheel Angular Velocities
   *
   * @param twist Desired robot twist (linear and angular velocities).
   * @return WheelAngularVelocity Struct containing angular velocities for each wheel (rad/s).
   */
  WheelAngularVelocity inverse(Twist const& twist) const
  {
    double const r           = (wheel_base_ + track_width_) / 2;
    double const front_left  = (twist.linear_x - twist.linear_y - r * twist.angular_z) / wheel_radius_;
    double const front_right = (twist.linear_x + twist.linear_y + r * twist.angular_z) / wheel_radius_;
    double const rear_left   = (twist.linear_x + twist.linear_y - r * twist.angular_z) / wheel_radius_;
    double const rear_right  = (twist.linear_x - twist.linear_y + r * twist.angular_z) / wheel_radius_;
    return WheelAngularVelocity{ front_left, front_right, rear_left, rear_right };
  }

  /**
   * @brief Computes the robot's twist given the current wheel angular velocities.
   *
   * Forward Kinematics: Wheel Angular Velocities -> Twist
   *
   * @param wheel_velocities Current wheel angular velocities (rad/s).
   * @return Twist Struct containing the robot's linear and angular velocities (m/s, rad/s).
   */
  Twist forward(WheelAngularVelocity const& wheel_velocities) const
  {
    double const r        = (wheel_base_ + track_width_) / 2;
    double const linear_x = 0.25 * wheel_radius_ *
                            (wheel_velocities.front_left + wheel_velocities.front_right +
                             wheel_velocities.rear_left + wheel_velocities.rear_right);
    double const linear_y = 0.25 * wheel_radius_ *
                            (-wheel_velocities.front_left + wheel_velocities.front_right +
                             wheel_velocities.rear_left - wheel_velocities.rear_right);
    double const angular_z = 0.25 * wheel_radius_ *
                             (-wheel_velocities.front_left + wheel_velocities.front_right -
                              wheel_velocities.rear_left + wheel_velocities.rear_right) /
                             r;
    return Twist{ linear_x, linear_y, angular_z };
  }

private:
  double wheel_radius_; ///< Radius of the wheels (meters).
  double wheel_base_;   ///< Distance between front and rear wheels (meters).
  double track_width_;  ///< Distance between left and right wheels (meters).
};
} // mameMecanumKinematics

#endif // MAMEMECANUMKINEMATICS_HPP_
