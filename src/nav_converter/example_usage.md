# Navigation Converter Usage Example

This document provides a step-by-step guide on how to use the nav_converter package.

## Prerequisites

1. ROS Melodic/Noetic installed
2. Catkin workspace set up
3. The nav_converter package built successfully

## Quick Start

### 1. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Test the Converter

First, launch the converter:
```bash
roslaunch nav_converter nav_converter.launch
```

In another terminal, run the test script:
```bash
rosrun nav_converter test_converter.py
```

### 3. Monitor Topics

Monitor the topics to see the conversion in action:
```bash
# Monitor input velocity commands
rostopic echo /cmd_vel

# Monitor output position commands
rostopic echo /cmd

# Monitor odometry
rostopic echo /odom
```

## Integration with Navigation Stack

### 1. Add to Navigation Launch File

Add this line to your navigation launch file (e.g., `sentry_movebase.launch`):

```xml
<include file="$(find nav_converter)/launch/nav_converter.launch"/>
```

### 2. Verify Topic Connections

Ensure the following topics are properly connected:

- **Input**: `/cmd_vel` (from move_base)
- **Input**: `/odom` (from localization)
- **Output**: `/cmd` (to px4ctrl)

### 3. Configure Parameters

Edit `param/converter_params.yaml` to match your drone's capabilities:

```yaml
# Adjust these values for your drone
max_velocity: 2.0          # Maximum velocity in m/s
max_acceleration: 1.0      # Maximum acceleration in m/s²
max_yaw_rate: 1.0          # Maximum yaw rate in rad/s
control_frequency: 50.0    # Control loop frequency in Hz
```

## Troubleshooting

### Common Issues

1. **No position commands published**
   ```bash
   # Check if topics exist
   rostopic list | grep cmd
   
   # Check topic types
   rostopic type /cmd_vel
   rostopic type /cmd
   ```

2. **Converter not receiving odometry**
   ```bash
   # Check odometry topic
   rostopic echo /odom
   
   # Verify topic name in converter
   # Default is /odom, change if needed
   ```

3. **Commands too aggressive**
   ```bash
   # Reduce velocity limits in parameters
   # Edit param/converter_params.yaml
   max_velocity: 1.0  # Reduce from 2.0
   ```

### Debug Mode

Enable debug output by setting ROS log level:
```bash
rosrun nav_converter cmd_vel_to_position --ros-args --log-level debug
```

## Advanced Configuration

### Custom Topic Names

You can change topic names by modifying the source code or using ROS remapping:

```bash
rosrun nav_converter cmd_vel_to_position cmd_vel:=/move_base/cmd_vel cmd:=/px4ctrl/cmd
```

### Custom Parameters

Override parameters at runtime:
```bash
rosrun nav_converter cmd_vel_to_position _max_velocity:=1.5 _control_frequency:=100.0
```

## Performance Monitoring

### Monitor Conversion Latency

```bash
# Check message timestamps
rostopic echo /cmd_vel -n 1 | grep stamp
rostopic echo /cmd -n 1 | grep stamp
```

### Monitor Message Frequency

```bash
# Check publishing rates
rostopic hz /cmd_vel
rostopic hz /cmd
```

## Safety Considerations

1. **Always test in simulation first**
2. **Start with conservative velocity limits**
3. **Monitor the drone's behavior closely**
4. **Have an emergency stop mechanism ready**
5. **Verify odometry accuracy before flight**

## Integration Checklist

- [ ] Converter builds successfully
- [ ] Test script runs without errors
- [ ] Topics are properly connected
- [ ] Parameters are configured for your drone
- [ ] Safety limits are appropriate
- [ ] Emergency stop is available
- [ ] Tested in simulation
- [ ] Ready for real flight 