# Pedro Pathing Tuning Guide

## Overview
The `Tuning.java` OpMode provides a menu-based system to tune your robot's Pedro Pathing parameters for optimal autonomous performance.

## How to Use

1. **Select "Tuning" from Driver Station** OpMode list
2. **Choose a tuning mode** from the menu using gamepad
3. **Follow on-screen instructions** for each tuner
4. **Record values** and update `Constants.java`

## Tuning Order (Recommended)

### 1. Localization Tuning
Run these first to ensure accurate position tracking:

- **Localization Test** - Verify robot position tracking
  - Drive robot with gamepad 1
  - Check if telemetry position matches actual position
  - Verify heading changes correctly

- **Forward Tuner** - Calibrate forward movement
  - Push/pull robot exactly 48 inches forward
  - Record the "ticks to inches" multiplier
  - Update forward encoder constant

- **Lateral Tuner** - Calibrate sideways movement  
  - Push/pull robot exactly 48 inches sideways
  - Record the "ticks to inches" multiplier
  - Update lateral encoder constant

- **Turn Tuner** - Calibrate rotation
  - Rotate robot exactly 360 degrees
  - Record the heading multiplier
  - Update turn constant

### 2. Automatic Tuning
Let the robot automatically calculate parameters:

- **Forward Velocity Tuner** - Measures max forward speed
- **Lateral Velocity Tuner** - Measures max lateral speed
- **Forward Zero Power Acceleration Tuner** - Measures deceleration
- **Lateral Zero Power Acceleration Tuner** - Measures lateral deceleration

### 3. Manual Tuning
Fine-tune PID and motion parameters:

- **Translational Tuner** - Tune forward/backward PID
- **Heading Tuner** - Tune rotation PID
- **Drive Tuner** - Tune overall drive performance
- **Centripetal Tuner** - Tune curve following

### 4. Tests
Verify tuning with test paths:

- **Line** - Simple straight line
- **Triangle** - Three-sided path
- **Circle** - Circular path

## After Tuning

Update values in `Constants.java`:
```java
// Example values to update
public static double forwardTicksToInches = 0.001989436789;
public static double lateralTicksToInches = 0.001989436789;
public static double turnTicksToInches = 0.001989436789;
```

## Tips

- Tune in order listed above
- Run multiple trials and average results
- Use FTC Dashboard for real-time visualization
- Test with actual autonomous paths after tuning
- Re-tune if you change wheels, motors, or weight distribution

## Troubleshooting

- **Robot drifts during straight paths** → Re-run Forward/Lateral Tuner
- **Robot overshoots turns** → Adjust Heading Tuner PID values
- **Jerky motion** → Lower max velocity or increase acceleration time
- **Position tracking incorrect** → Verify encoder directions in Constants.java
