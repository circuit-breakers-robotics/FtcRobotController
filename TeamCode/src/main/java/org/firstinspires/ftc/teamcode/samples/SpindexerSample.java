package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SpindexerSample", group = "TeleOp")
public class SpindexerSample extends LinearOpMode {

    CRServo crSpindexer;
    Servo spindexer;
    private boolean useCRServo = false; // true=CRServo, false=Servo
    private long spindexerStartTime = 0;
    private boolean spindexerRunning = false;
    private static final int ROTATION_TIME_MS = 650;
    private double currentPosition = 0.0;
    private int counter = 0;
    private double base = 0.51;
    private double left = 0.965;
    private double right = 0.035;
    //1.0;//0.56;//0.09;
    //0.0;0.47;0.93

    @Override
    public void runOpMode() throws InterruptedException {

        if(useCRServo) {
            crSpindexer = hardwareMap.get(CRServo.class, "spindexer");
            crSpindexer.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            spindexer = hardwareMap.get(Servo.class, "spindexer");
            spindexer.setPosition(base); // base for spindexer ball holder

        }

        telemetry.addData(">", "Press Start to operate TeleOp" );
        telemetry.addData("Mode", useCRServo ? "CRServo" : "Servo");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            runSpindexer();

            telemetry.addData("Spindexer position: ", spindexer.getPosition());
            telemetry.update();
        }
    }

    private void runSpindexer() {
        if(useCRServo) {
            // CRServo implementation (time-based)
            if(gamepad2.left_trigger > 0.5 && !spindexerRunning) {
                spindexerStartTime = System.currentTimeMillis();
                spindexerRunning = true;
            }

            if(spindexerRunning) {
                long elapsed = System.currentTimeMillis() - spindexerStartTime;
                if(elapsed < ROTATION_TIME_MS) {
                    crSpindexer.setPower(1.0);
                } else {
                    crSpindexer.setPower(0);
                    spindexerRunning = false;
                }
            }
            telemetry.addData("Spindexer Running: ", spindexerRunning);
        } else {
            // Servo implementation (position-based)
            if(gamepad2.left_trigger > 0.5 && !spindexerRunning) {
//                currentPosition += 0.333; // 120° out of 270° range
//                if(currentPosition > 1.0) currentPosition -= 1.0;
                if (counter == 0) {
                    spindexer.setPosition(left);
                } else if(counter == 1) {
                    spindexer.setPosition(right);
                } else if(counter == 2) {
                    spindexer.setPosition(base);
                    counter = -1;
                }
//                spindexer.setPosition(currentPosition);
                spindexerRunning = true;
                counter++;
                telemetry.addData("Spindexer position: ", spindexer.getPosition());
            }

            if(gamepad2.left_trigger == 0) {
                spindexerRunning = false;
            }
            //telemetry.addData("Servo Position: ", currentPosition);
        }
    }
}
