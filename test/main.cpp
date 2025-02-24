#include "../src/mameMecanumKinematics.hpp"
#include "utest.h"

using namespace mameMecanumKinematics;

UTEST(MameMecanumKinematics, InverseKinematics)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  Twist                 twist{ 1.0, 0.0, 0.0 };
  WheelAngularVelocity  wheelVelocities = kinematics.inverse(twist);

  ASSERT_NEAR(10.0, wheelVelocities.front_left, 1e-5);
  ASSERT_NEAR(10.0, wheelVelocities.front_right, 1e-5);
  ASSERT_NEAR(10.0, wheelVelocities.rear_left, 1e-5);
  ASSERT_NEAR(10.0, wheelVelocities.rear_right, 1e-5);
}

UTEST(MameMecanumKinematics, ForwardKinematics)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  WheelAngularVelocity  wheelVelocities{ 10.0, 10.0, 10.0, 10.0 };
  Twist                 twist = kinematics.forward(wheelVelocities);

  ASSERT_NEAR(1.0, twist.linear_x, 1e-5);
  ASSERT_NEAR(0.0, twist.linear_y, 1e-5);
  ASSERT_NEAR(0.0, twist.angular_z, 1e-5);
}

UTEST(MameMecanumKinematics, ZeroVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  Twist                 twist{ 0.0, 0.0, 0.0 };
  WheelAngularVelocity  wheelVelocities = kinematics.inverse(twist);

  ASSERT_NEAR(0.0, wheelVelocities.front_left, 1e-5);
  ASSERT_NEAR(0.0, wheelVelocities.front_right, 1e-5);
  ASSERT_NEAR(0.0, wheelVelocities.rear_left, 1e-5);
  ASSERT_NEAR(0.0, wheelVelocities.rear_right, 1e-5);
}

UTEST(MameMecanumKinematics, NegativeVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  Twist                 twist{ -1.0, 0.0, 0.0 };
  WheelAngularVelocity  wheelVelocities = kinematics.inverse(twist);

  ASSERT_NEAR(-10.0, wheelVelocities.front_left, 1e-5);
  ASSERT_NEAR(-10.0, wheelVelocities.front_right, 1e-5);
  ASSERT_NEAR(-10.0, wheelVelocities.rear_left, 1e-5);
  ASSERT_NEAR(-10.0, wheelVelocities.rear_right, 1e-5);
}

UTEST(MameMecanumKinematics, MaxVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  Twist                 twist{ 10.0, 10.0, 0.0 };
  WheelAngularVelocity  wheelVelocities = kinematics.inverse(twist);

  ASSERT_NEAR(0.0, wheelVelocities.front_left, 1e-5);
  ASSERT_NEAR(200.0, wheelVelocities.front_right, 1e-5);
  ASSERT_NEAR(200.0, wheelVelocities.rear_left, 1e-5);
  ASSERT_NEAR(0.0, wheelVelocities.rear_right, 1e-5);
}

UTEST(MameMecanumKinematics, InverseKinematicsCombinedVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  Twist                 twist{ 1.0, 1.0, 1.0 };
  WheelAngularVelocity  wheelVelocities = kinematics.inverse(twist);

  ASSERT_NEAR(-5.0, wheelVelocities.front_left, 1e-5);
  ASSERT_NEAR(25.0, wheelVelocities.front_right, 1e-5);
  ASSERT_NEAR(15.0, wheelVelocities.rear_left, 1e-5);
  ASSERT_NEAR(5.0, wheelVelocities.rear_right, 1e-5);
}

UTEST(MameMecanumKinematics, ForwardZeroVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  WheelAngularVelocity  wheelVelocities{ 0.0, 0.0, 0.0, 0.0 };
  Twist                 twist = kinematics.forward(wheelVelocities);

  ASSERT_NEAR(0.0, twist.linear_x, 1e-5);
  ASSERT_NEAR(0.0, twist.linear_y, 1e-5);
  ASSERT_NEAR(0.0, twist.angular_z, 1e-5);
}

UTEST(MameMecanumKinematics, ForwardNegativeVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  WheelAngularVelocity  wheelVelocities{ -10.0, -10.0, -10.0, -10.0 };
  Twist                 twist = kinematics.forward(wheelVelocities);

  ASSERT_NEAR(-1.0, twist.linear_x, 1e-5);
  ASSERT_NEAR(0.0, twist.linear_y, 1e-5);
  ASSERT_NEAR(0.0, twist.angular_z, 1e-5);
}

UTEST(MameMecanumKinematics, ForwardMaxVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  WheelAngularVelocity  wheelVelocities{ 100.0, 100.0, 100.0, 100.0 };
  Twist                 twist = kinematics.forward(wheelVelocities);

  ASSERT_NEAR(10.0, twist.linear_x, 1e-5);
  ASSERT_NEAR(0.0, twist.linear_y, 1e-5);
  ASSERT_NEAR(0.0, twist.angular_z, 1e-5);
}

UTEST(MameMecanumKinematics, ForwardKinematicsCombinedVelocities)
{
  MameMecanumKinematics kinematics(0.1, 0.5, 0.5);
  WheelAngularVelocity  wheelVelocities{ 5.0, 15.0, 15.0, 5.0 };
  Twist                 twist = kinematics.forward(wheelVelocities);

  ASSERT_NEAR(1.0, twist.linear_x, 1e-5);
  ASSERT_NEAR(0.5, twist.linear_y, 1e-5);
  ASSERT_NEAR(0.0, twist.angular_z, 1e-5);
}

UTEST_MAIN();