package org.firstinspires.ftc.teamcode.samples;

import static java.lang.Math.floor;
import static java.lang.Math.round;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "IntrgratedRobotTeleOpSample", group = "TeleOp")
public class IntegratedRobotTeleOpSample extends LinearOpMode {

    public static final double defaultLaunchPower = 0.678;
    private Servo flicker;
    private Servo spindexer;
    private DcMotorSimple intake; // for spark mini servo controller
    private DcMotorSimple launch;

    private DcMotor leftDriveFront;
    private DcMotor leftDriveRear;
    private DcMotor rightDriveFront;
    private DcMotor rightDriveRear;
    private IMU imu;

    private boolean spindexerRunning = false;
    private static final int ROTATION_TIME_MS = 1100; // Time for 120° rotation

    private boolean autoLaunchActive = false;
    private int launchStep = 0;
    private long stepStartTime = 0;
    private static final int FLICK_TIME_MS = 1000;
    private static final int FLICKER_RETURN_MS = 500;
    private static final int LAUNCHER_THROTTLE_MS = 2800;

    //[125-130, 0.88]
    //[80-83, 0.768]
    //[75-78, 0.70]
    //[66-70, 0.78]
    //[74-65, 0.65]
    //[45-65, 0.652]
    //[30-45, 0.62]
    //[65-67, 0.678]
    private static final double[][] LAUNCH_VELOCITY_MAP = {
        {30, 45, 0.62},
        {45, 65, 0.652},
        {65, 67, 0.678},
        {67, 75, 0.688},
        {75, 78, 0.70},
        {78, 85, 0.768},
        {124, 130, 0.88}
    };

    private static final double FAR_ZONE_MAX = 126.0; // inches (65 + 15)

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int counter = 0;
    // for rotation in one direction saves cycle time
    private double spindexer_2 = 0.965;
    private double spindexer_1 = 0.51;
    private double spindexer_0 = 0.035;
    private double flickerBase = 0.0;
    private double flickerUp = 0.52;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initRobotHardware();
        // Wait for the start button
        telemetry.addData(">", "Press Start to operate TeleOp" );
        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()){
            runDrive();
            runIntake();

            if(gamepad2.right_bumper && !autoLaunchActive) {
                autoLaunchActive = true;
                launchStep = 0;
                stepStartTime = System.currentTimeMillis();
            }

            if(autoLaunchActive) {
                runAutoLaunch();
            } else {
                runFlicker();
                runSpindexer();
                runLaunch();
            }

            telemetry.update();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    private void runFlicker() {
        if(gamepad2.right_trigger > 0.5) {
            flicker.setPosition(flickerUp);
        } else {
            flicker.setPosition(flickerBase);
        }
        telemetry.addData("Flicker position: ", flicker.getPosition());
    }

    private void runSpindexer() {
        if((gamepad2.left_trigger > 0.5 || autoLaunchActive) && !spindexerRunning) {
            if (counter == 0) {
                spindexer.setPosition(spindexer_0);
            } else if(counter == 1) {
                spindexer.setPosition(spindexer_1);
            } else if(counter == 2) {
                spindexer.setPosition(spindexer_2);
                counter = -1;
            }
            spindexerRunning = true;
            counter++;
            telemetry.addData("Spindexer position: ", spindexer.getPosition());
        }

        if(gamepad2.left_trigger == 0 || autoLaunchActive) {
            spindexerRunning = false;
        }
    }

    private double getAprilTagDistance() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);
            return detection.ftcPose.range;
        }

        return FAR_ZONE_MAX; // Default to far zone if no tag detected
    }

    private void runIntake() {
        double intakePower = gamepad2.left_stick_y;
        intake.setPower(intakePower);
        telemetry.addData("Intake Power: ", intakePower);
    }

    private void runLaunch() {
        double launchPower = gamepad2.right_stick_y;
        launch.setPower(launchPower);
        telemetry.addData("Launch Power: ", launchPower);
    }

    private void runDrive() {
//        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y);
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;
        driveLinear(axial, lateral, yaw);
    }

    private void initRobotHardware() {
        flicker = hardwareMap.get(Servo.class, "flicker");
        flicker.setPosition(flickerBase);

        spindexer = hardwareMap.get(Servo.class, "spindexer");
        spindexer.setPosition(spindexer_0);


        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launch = hardwareMap.get(DcMotorSimple.class, "launch");


        leftDriveFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveRear = hardwareMap.get(DcMotor.class, "left_drive_rear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveRear = hardwareMap.get(DcMotor.class, "right_drive_rear");

        imu = hardwareMap.get(IMU.class, "imu"); // Integral Measurement Unit
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "webcam"), aprilTag);

    }

    private void runAutoLaunch() {
        long elapsed = System.currentTimeMillis() - stepStartTime;
        double distance = floor(getAprilTagDistance());
        double launchPower = defaultLaunchPower;

        for(double[] range : LAUNCH_VELOCITY_MAP) {
            if(distance >= range[0] && distance <= range[1]) {
                launchPower = range[2];
                break;
            }
        }

        visionPortal.resumeStreaming();
        launch.setPower(launchPower);
        telemetry.addData("Distance from goal inches", distance);

        // Step 0,2,4: Flick (500ms each)
        // Step 1,3,5: Spin to position (667ms each)
        if(launchStep % 2 != 0) {
            // Spin step
            flicker.setPosition(flickerBase);
            if(elapsed >= ROTATION_TIME_MS) {
                launchStep++;
                stepStartTime = System.currentTimeMillis();
            }
        } else {
            // Flick step
            if(launchStep == 0 && elapsed < LAUNCHER_THROTTLE_MS) {
                flicker.setPosition(flickerBase);
            } else if(elapsed < FLICK_TIME_MS + (launchStep == 0 ? LAUNCHER_THROTTLE_MS : 0)) {
                flicker.setPosition(flickerUp);
            } else if(elapsed < FLICK_TIME_MS + FLICKER_RETURN_MS + (launchStep == 0 ? LAUNCHER_THROTTLE_MS : 0)) {
                flicker.setPosition(flickerBase);
            } else {
                flicker.setPosition(flickerBase);
                if (counter == 0) {
                    spindexer.setPosition(spindexer_0);
                } else if(counter == 1) {
                    spindexer.setPosition(spindexer_1);
                } else if(counter == 2) {
                    spindexer.setPosition(spindexer_2);
                    counter = -1;
                }
                counter++;
                launchStep++;
                stepStartTime = System.currentTimeMillis();
            }
        }

        if(launchStep >= 6) {
            autoLaunchActive = false;
            launch.setPower(0);
            visionPortal.stopStreaming();
        }
        telemetry.addData("Auto Launch Step: ", launchStep);
        telemetry.addData("Launch Power: ", launchPower);
    }

    // Thanks to FTC16072 for sharing this code!!
    private void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        leftDriveFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        rightDriveFront.setPower(maxSpeed * (frontRightPower / maxPower));
        leftDriveRear.setPower(maxSpeed * (backLeftPower / maxPower));
        rightDriveRear.setPower(maxSpeed * (backRightPower / maxPower));

        telemetry.addData("left front Power: ", leftDriveFront.getPower());
        telemetry.addData("left back Power: ", leftDriveRear.getPower());
        telemetry.addData("right front Power: ", rightDriveFront.getPower());
        telemetry.addData("right back Power: ", rightDriveRear.getPower());
    }

    private void driveLinear(double axial, double lateral, double yaw) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        leftDriveFront.setPower(frontLeftPower);
        rightDriveFront.setPower(frontRightPower);
        leftDriveRear.setPower(backLeftPower);
        rightDriveRear.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }
}
