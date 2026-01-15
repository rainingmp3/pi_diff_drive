# ROS2 Differential Drive Robot
A working ROS2 Jazzy implementation for controlling a differential drive robot with encoder feedback.

## Hardware
- Raspberry Pi 4B
- Arduino Nano (clone)
- L298N motor driver
- IR encoders
- 4× AA batteries
 
## Challenges
Getting ros2_control working on real hardware is notoriously difficult:
- **Fragmented documentation** - Information is scattered across multiple ROS2 distributions with breaking changes between versions
- **Complex debugging** - Issues can originate from hardware, firmware, drivers, or any layer of the ros2_control stack
- **Large codebase** - Tracing problems through controllers, hardware interfaces, and lifecycle management is time-consuming
- **Simultaneous failures** - Hardware and software issues often occur together, making root cause analysis challenging

This repository provides a **complete, tested implementation** for the hardware mentioned above with **ROS2 Jazzy**.

## Credits
This project combines work from:
- [Josh Newans](https://github.com/joshnewans) - hardware package (hardware interface and Arduino code)
- [Edouard Renard](https://github.com/niryo) - all other packages (basic structure of the project, main configuration and bringup files)
- [Aleksandar Haber](https://www.youtube.com/@AleksandarHaber) - hardware assembly

## Resources
- [Josh Newans YouTube Series](https://www.youtube.com/watch?v=OWeLUSzxMsw&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT)
- [Edouard Renard's YouTube Channel](https://www.youtube.com/@RoboticsBackEnd)
- [Aleksandar Haber PhD YouTube](https://www.youtube.com/watch?v=RsZBxfpcLu8)
- [ROS2 Control Documentation](https://control.ros.org/jazzy/index.html)
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
- [Buy list, electronics, etc.](https://ros-mobile-robots.com)

## Status
- [x] Differential drive control working  
- [x] Encoder feedback integrated  
- [x] Tested on real hardware
- [ ] Polishing, tuning 
- [ ] Combining low level control with higher level navigation
- [ ] Behavior trees, obstacle avoidance, and general "higher level" tasks
