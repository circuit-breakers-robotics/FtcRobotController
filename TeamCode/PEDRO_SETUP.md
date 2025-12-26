# Pedro Pathing Setup Instructions

## 1. Add Pedro Pathing Dependency

Add to `TeamCode/build.gradle` in the `dependencies` section:

```gradle
dependencies {
    implementation project(':FtcRobotController')
    implementation 'com.github.pedropathing:pedropathing:1.0.0'
}
```

## 2. Add JitPack Repository

Add to root `build.gradle` in the `repositories` section:

```gradle
allprojects {
    repositories {
        google()
        mavenCentral()
        maven { url 'https://jitpack.io' }
    }
}
```

## 3. Sync Gradle

Click "Sync Now" in Android Studio after making these changes.

## 4. Configure Your Robot

Update the path coordinates in `DecodeAutonomousPedro.java` to match your field positions (in inches).

## 5. Tune Follower Parameters

Create a tuning OpMode or adjust follower parameters in the constructor if needed.

## Path Explanation

The example path:
- Drives forward 24 inches
- Turns and drives to (24, 24) at 90°
- Drives forward to (36, 24)
- Runs intake at 30% of path completion
- Runs launch at 70% of path completion

Customize the path using BezierLine, BezierCurve, or other path types.
