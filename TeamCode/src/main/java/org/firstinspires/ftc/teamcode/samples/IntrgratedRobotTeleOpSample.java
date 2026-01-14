package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "IntrgratedRobotTeleOpSample", group = "TeleOp")
public class IntrgratedRobotTeleOpSample extends LinearOpMode {

    CRServo flicker;
    CRServo spindexer;
    DcMotorSimple intake; // for spark mini servo controller
    DcMotorSimple launch;

    @Override
    public void runOpMode() {
        flicker = hardwareMap.get(CRServo.class, "flicker");
        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        spindexer.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hardwareMap.get(DcMotorSimple.class, "intake");

        launch = hardwareMap.get(DcMotor.class, "launch");


        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        // Scan servo till stop pressed.
        while(opModeIsActive()){
            runIntake();
            runFlicker();
            runSpindexer();
            runLaunch();
            telemetry.update();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    private void runFlicker() {
        double flickerPower = gamepad2.right_trigger;
        flickerPower *= 20;

        if(flickerPower > 0) {
            flicker.setPower(flickerPower);  // Use trigger value as power (0-1)
        } else {
            flicker.setPower(0);  // Stop servo
        }
        telemetry.addData("Flicker Power: ", flickerPower);
    }

    private void runSpindexer() {
        double spindexerPower = gamepad2.left_trigger;
        spindexerPower *= 20;
        if(spindexerPower > 0) {
            spindexer.setPower(spindexerPower);  // Use trigger value as power (0-1)
        } else {
            spindexer.setPower(0);
        }
        telemetry.addData("Spindexer Power: ", spindexerPower);
    }

    private void runLaunch() {
        double launchPower = -gamepad1.left_stick_y;
        launch.setPower(launchPower);
        telemetry.addData("Launch Power: ", launchPower);
    }

    private void runIntake() {
        double intakePower = -gamepad1.right_stick_y;
        intake.setPower(intakePower);
        telemetry.addData("Intake Power: ", intakePower);
    }
}
