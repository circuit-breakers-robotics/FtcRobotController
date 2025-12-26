package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="DECODE Autonomous PID", group="TeamCode")
public class DecodeAutonomousPID extends LinearOpMode {

    private DcMotor leftDriveFront = null;
    private DcMotor leftDriveRear = null;
    private DcMotor rightDriveFront = null;
    private DcMotor rightDriveRear = null;
    private DcMotor intakeMotor = null;
    private DcMotor launchMotor = null;
    private IMU imu = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;
    static final double HEADING_THRESHOLD = 1.0;
    static final double P_TURN_GAIN = 0.02;
    static final double P_DRIVE_GAIN = 0.03;

    @Override
    public void runOpMode() {
        leftDriveFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveRear = hardwareMap.get(DcMotor.class, "left_drive_rear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveRear = hardwareMap.get(DcMotor.class, "right_drive_rear");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        launchMotor = hardwareMap.get(DcMotor.class, "launch_motor");

        // make sure the pair has same direction
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu"); // Integral Measurement Unit
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // for consistent runs
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        // Autonomous sequence
        driveStraight(DRIVE_SPEED, 24, 0, 5.0);
        intake(1.0, 2.0);  // Run intake for 2 seconds
        turnToHeading(TURN_SPEED, 90, 3.0);
        driveStraight(DRIVE_SPEED, 12, 90, 3.0);
        launch(1.0, 1.5);  // Launch for 1.5 seconds
        turnToHeading(TURN_SPEED, 0, 3.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void driveStraight(double maxSpeed, double distance, double heading, double timeout) {
        if (!opModeIsActive()) return;

        int target = (int)(distance * COUNTS_PER_INCH);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout) {
            int leftPos = Math.abs(leftDriveFront.getCurrentPosition());
            int rightPos = Math.abs(rightDriveFront.getCurrentPosition());
            int avgPos = (leftPos + rightPos) / 2;

            if (avgPos >= Math.abs(target)) break;

            double headingError = heading - getHeading();
            double turnSpeed = Range.clip(headingError * P_DRIVE_GAIN, -1, 1);
            double driveSpeed = maxSpeed * (1 - Math.abs(avgPos - target) / (double)target * 0.3);

            setDrivePower(driveSpeed - turnSpeed, driveSpeed + turnSpeed);

            telemetry.addData("Target", target);
            telemetry.addData("Position", "%d:%d", leftPos, rightPos);
            telemetry.addData("Heading", "%.1f", getHeading());
            telemetry.update();
        }
        setDrivePower(0, 0);
        sleep(100);
    }

    public void turnToHeading(double maxSpeed, double heading, double timeout) {
        if (!opModeIsActive()) return;

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < timeout) {
            double headingError = heading - getHeading();
            
            while (headingError > 180) headingError -= 360;
            while (headingError <= -180) headingError += 360;

            if (Math.abs(headingError) < HEADING_THRESHOLD) break;

            double turnSpeed = Range.clip(headingError * P_TURN_GAIN, -maxSpeed, maxSpeed);
            setDrivePower(-turnSpeed, turnSpeed);

            telemetry.addData("Target Heading", heading);
            telemetry.addData("Current Heading", "%.1f", getHeading());
            telemetry.addData("Error", "%.1f", headingError);
            telemetry.update();
        }
        setDrivePower(0, 0);
        sleep(100);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivePower(double leftPower, double rightPower) {
        leftDriveFront.setPower(leftPower);
        leftDriveRear.setPower(leftPower);
        rightDriveFront.setPower(rightPower);
        rightDriveRear.setPower(rightPower);
    }

    public void intake(double power, double duration) {
        if (!opModeIsActive()) return;
        intakeMotor.setPower(power);
        sleep((long)(duration * 1000));
        intakeMotor.setPower(0);
    }

    public void launch(double power, double duration) {
        if (!opModeIsActive()) return;
        launchMotor.setPower(power);
        sleep((long)(duration * 1000));
        launchMotor.setPower(0);
    }
}
