package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "IntrgratedRobotTeleOpSample", group = "TeleOp")
public class IntrgratedRobotTeleOpSample extends LinearOpMode {

    CRServo flicker;
    CRServo spindexer;
    DcMotorSimple intake; // for spark mini servo controller
    DcMotor launch;

    private DcMotor leftDriveFront;
    private DcMotor leftDriveRear;
    private DcMotor rightDriveFront;
    private DcMotor rightDriveRear;
    
    private long spindexerStartTime = 0;
    private boolean spindexerRunning = false;
    private static final int ROTATION_TIME_MS = 667; // Time for 120° rotation
    
    private boolean autoLaunchActive = false;
    private int launchStep = 0;
    private long stepStartTime = 0;
    private static final int FLICK_TIME_MS = 500;
    
    private static final double NEAR_ZONE_MIN = 38.0; // inches (48 - 10)
    private static final double NEAR_ZONE_MAX = 63.0; // inches (48 + 15)
    private static final double NEAR_ZONE_VELOCITY = 0.6;
    private static final double FAR_ZONE_MIN = 55.0; // inches (65 - 10)
    private static final double FAR_ZONE_MAX = 80.0; // inches (65 + 15)
    private static final double FAR_ZONE_VELOCITY = 1.0;
    
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initRobotHardware();
        // Wait for the start button
        telemetry.addData(">", "Press Start to operate TeleOp" );
        telemetry.update();
        waitForStart();

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
            flicker.setPower(1.0);
        } else {
            flicker.setPower(0);
        }
        telemetry.addData("Flicker Power: ", flicker.getPower());
    }

    private void runSpindexer() {
        if(gamepad2.left_trigger > 0 && !spindexerRunning) {
            spindexerStartTime = System.currentTimeMillis();
            spindexerRunning = true;
        }
        
        if(spindexerRunning) {
            long elapsed = System.currentTimeMillis() - spindexerStartTime;
            if(elapsed < ROTATION_TIME_MS) {
                spindexer.setPower(1.0);
            } else {
                spindexer.setPower(0);
                spindexerRunning = false;
            }
        }
        telemetry.addData("Spindexer Running: ", spindexerRunning);
    }

    private void runLaunch() {
        double distance = getAprilTagDistance();
        double launchPower;
        
        if(distance >= NEAR_ZONE_MIN && distance <= NEAR_ZONE_MAX) {
            launchPower = NEAR_ZONE_VELOCITY;
        } else if(distance >= FAR_ZONE_MIN && distance <= FAR_ZONE_MAX) {
            launchPower = FAR_ZONE_VELOCITY;
        } else {
            launchPower = 0.8; // Mid-range default
        }
        
        if(gamepad1.a) {
            visionPortal.resumeStreaming();
            launch.setPower(launchPower);
        } else {
            visionPortal.stopStreaming();
            launch.setPower(0);
        }
        
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Launch Power: ", launchPower);
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
        double intakePower = -gamepad1.right_stick_y;
        intake.setPower(intakePower);
        telemetry.addData("Intake Power: ", intakePower);
    }

    private void runDrive() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        leftDriveFront.setPower(leftPower);
        leftDriveRear.setPower(leftPower);
        rightDriveFront.setPower(rightPower);
        rightDriveRear.setPower(rightPower);
        
        telemetry.addData("Left Drive: ", leftPower);
        telemetry.addData("Right Drive: ", rightPower);
    }

    private void initRobotHardware() {
        flicker = hardwareMap.get(CRServo.class, "flicker");
        flicker.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        spindexer.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorSimple.class, "intake");
        launch = hardwareMap.get(DcMotor.class, "launch");
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveRear = hardwareMap.get(DcMotor.class, "left_drive_rear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveRear = hardwareMap.get(DcMotor.class, "right_drive_rear");

        // make sure the pair has same direction
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD);

//        imu = hardwareMap.get(IMU.class, "imu"); // Integral Measurement Unit
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        imu.initialize(parameters);

        // for consistent runs
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

    }
    
    private void runAutoLaunch() {
        long elapsed = System.currentTimeMillis() - stepStartTime;
        double distance = getAprilTagDistance();
        double launchPower;
        
        if(distance >= NEAR_ZONE_MIN && distance <= NEAR_ZONE_MAX) {
            launchPower = NEAR_ZONE_VELOCITY;
        } else if(distance >= FAR_ZONE_MIN && distance <= FAR_ZONE_MAX) {
            launchPower = FAR_ZONE_VELOCITY;
        } else {
            launchPower = 0.8;
        }
        
        visionPortal.resumeStreaming();
        launch.setPower(launchPower);
        
        // Step 0,2,4: Spin to position (667ms each)
        // Step 1,3,5: Flick (500ms each)
        if(launchStep % 2 == 0) {
            // Spin step
            if(elapsed < ROTATION_TIME_MS) {
                spindexer.setPower(1.0);
                flicker.setPower(0);
            } else {
                spindexer.setPower(0);
                launchStep++;
                stepStartTime = System.currentTimeMillis();
            }
        } else {
            // Flick step
            if(elapsed < FLICK_TIME_MS) {
                spindexer.setPower(0);
                flicker.setPower(1.0);
            } else {
                flicker.setPower(0);
                launchStep++;
                stepStartTime = System.currentTimeMillis();
                
                if(launchStep >= 6) {
                    autoLaunchActive = false;
                    launch.setPower(0);
                    visionPortal.stopStreaming();
                }
            }
        }
        
        telemetry.addData("Auto Launch Step: ", launchStep);
        telemetry.addData("Launch Power: ", launchPower);
    }
}
