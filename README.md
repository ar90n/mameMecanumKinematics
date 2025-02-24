# mameMecanumKinematics

A comprehensive implementation of mecanum wheel kinematics, designed for robotics applications requiring omnidirectional movement.

## Features

- **Omnidirectional Movement**: Supports forward, backward, lateral, and diagonal movement.
- **Easy Integration**: Simple API for integrating with existing robotics systems.
- **High Performance**: Optimized for real-time applications.

## Installation

To use mameMecanumKinematics, download the [`mameMecanumKinematics.hpp`](https://github.com/ar90n/mamePID/blob/main/src/mamePID.hpp) file and include it in your project.

```bash
wget https://raw.githubusercontent.com/ar90n/mameMecanumKinematics/refs/heads/main/src/mameMecanumKinematics.hpp
```

## Usage

Include the `mameMecanumKinematics.hpp` header in your project and use the provided functions to calculate wheel velocities based on desired movement vectors.

### Specifications

- **wheel_radius**: Radius of the wheels (meters).
- **wheel_base**: Distance between front and rear wheels (meters).
- **track_width**: Distance between left and right wheels (meters).

### Example

#### Inverse Kinematics

```cpp
#include "mameMecanumKinematics.hpp"

int main() {
    // Define wheel parameters
    double wheel_radius = 0.1; // meters
    double wheel_base = 0.5;   // meters
    double track_width = 0.4;  // meters

    // Create a MameMecanumKinematics object
    mameMecanumKinematics::MameMecanumKinematics kinematics(wheel_radius, wheel_base, track_width);

    // Define a desired twist (linear and angular velocities)
    mameMecanumKinematics::Twist twist = {1.0, 0.0, 0.5}; // m/s, m/s, rad/s

    // Calculate wheel angular velocities
    mameMecanumKinematics::WheelAngularVelocity velocities = kinematics.inverse(twist);

    // Output the results
    std::cout << "Front Left Wheel Velocity: " << velocities.front_left << " rad/s" << std::endl;
    std::cout << "Front Right Wheel Velocity: " << velocities.front_right << " rad/s" << std::endl;
    std::cout << "Rear Left Wheel Velocity: " << velocities.rear_left << " rad/s" << std::endl;
    std::cout << "Rear Right Wheel Velocity: " << velocities.rear_right << " rad/s" << std::endl;

    return 0;
}
```

#### Forward Kinematics

```cpp
#include "mameMecanumKinematics.hpp"

int main() {
    // Define wheel parameters
    double wheel_radius = 0.1; // meters
    double wheel_base = 0.5;   // meters
    double track_width = 0.4;  // meters

    // Create a MameMecanumKinematics object
    mameMecanumKinematics::MameMecanumKinematics kinematics(wheel_radius, wheel_base, track_width);

    // Define current wheel angular velocities
    mameMecanumKinematics::WheelAngularVelocity velocities = {1.0, 1.0, 1.0, 1.0}; // rad/s

    // Calculate the robot's twist
    mameMecanumKinematics::Twist twist = kinematics.forward(velocities);

    // Output the results
    std::cout << "Linear X Velocity: " << twist.linear_x << " m/s" << std::endl;
    std::cout << "Linear Y Velocity: " << twist.linear_y << " m/s" << std::endl;
    std::cout << "Angular Z Velocity: " << twist.angular_z << " rad/s" << std::endl;

    return 0;
}
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request for any enhancements or bug fixes.
