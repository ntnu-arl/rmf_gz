# RMF Controller Options

This package provides two controller options for the RMF quadrotor model to prevent conflicts from simultaneous controller usage.

## Controller Types

### Attitude Controller
- **Model file**: `model_attitude.sdf`
- **Command topic**: `/rmf/cmd/att` (geometry_msgs/Quaternion)
- **Description**: Controls the quadrotor using attitude (orientation) commands
- **Use case**: When you want direct control over the vehicle's orientation

### Acceleration Controller
- **Model file**: `model_acceleration.sdf`
- **Command topic**: `/rmf/cmd/acc` (geometry_msgs/Twist)
- **Description**: Controls the quadrotor using acceleration commands
- **Use case**: When you want to specify desired accelerations (commonly used with higher-level planners)

## Usage

### Selecting a Controller

Use the `controller` launch argument to specify which controller to use:

```bash
# Use attitude controller
ros2 launch rmf_gz start_sim.launch.xml controller:=attitude

# Use acceleration controller
ros2 launch rmf_gz start_sim.launch.xml controller:=acceleration
```

**Default**: If no controller is specified, the attitude controller is used by default.

### Important Notes

⚠️ **WARNING**: Only ONE controller should be active at a time. The model variants (`model_attitude.sdf` and `model_acceleration.sdf`) are designed to prevent conflicts by including only one controller plugin each.

- Do NOT send commands to both `/rmf/cmd/att` and `/rmf/cmd/acc` simultaneously
- Choose the appropriate controller based on your application needs
- The default `model.sdf` file includes only the attitude controller for backward compatibility

## Example Launch Commands

```bash
# Launch with acceleration controller for use with acceleration-based planners
ros2 launch rmf_gz start_sim.launch.xml controller:=acceleration world:=empty

# Launch with attitude controller
ros2 launch rmf_gz start_sim.launch.xml controller:=attitude world:=empty

# Launch with custom position
ros2 launch rmf_gz start_sim.launch.xml controller:=acceleration x:=1.0 y:=2.0 z:=0.5
```

## Controller Parameters

Both controllers share similar internal parameters (configured in the respective model files):
- **attitudeGain**: 15 15 2
- **angularRateGain**: 2 2 0.2
- **maximumLinearVelocity**: 5 5 5 m/s
- **maximumLinearAcceleration**: 5 5 5 m/s²
- **maximumAngularVelocity**: 5 5 3 rad/s

Refer to the individual model files for detailed configuration.
