package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="31250_Teleop_Near_Launch", group = "Teleop")
public class CircuitBreakersTeleopRunNearLaunch extends LinearOpMode {
    private static final double SPIN_STEP = 0.333;
    private static final double SPIN_DELAY_MS = 300;
    private static final double FLICK_POS = 0.75;
    private static final double FLICK_REST_POS = 0.3;
    private static final double FLICK_TIME_MS = 300;
    private Servo spindexerServo;
    private Servo flickerServo;
    private boolean spindexerRunning = false;
    private Servo flicker;
    private ElapsedTime timer = new ElapsedTime();
    // Define class members
    private boolean flicking = false;
    private Object runFlicker;

    @Override
    public void runOpMode() {
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
           double spinPower = gamepad2.left_trigger;
           double flickPower = gamepad2.right_trigger;
            if (spinPower > 0) {
                int detectedTagId = 21;
                runSpindexer(0, spindexerServo);
            }
            telemetry.addData("Spindexer Pos", spindexerServo.getPosition());
            telemetry.update();
            if (flickPower > 0) {
               runFlicker(flickerServo);

            }
            telemetry.addData("Flicker Pos", flickerServo.getPosition());
            telemetry.update();
        }
    }


    private int getRotationCount(int tagId) {
        //not yet implemented sorting
        switch (tagId) {
            case 21:
            case 22:
            case 23:
                return 2;
            default:
                return 1;
        }
    }

    private void rotateSpindexerStep() {
        double nextPosition = spindexerServo.getPosition() + SPIN_STEP;
        if (nextPosition > 1.0) {
            nextPosition -= 1.0;
        }
        spindexerServo.setPosition(nextPosition);
        sleep((long) SPIN_DELAY_MS);
    }

    private void runSpindexer(int detectedTagId, Servo spindexerServo) {
        int rotations = getRotationCount(detectedTagId);
        rotateSpindexerStep();
    }
    private void runFlicker (Servo flickerServo){
        flickerServo.setPosition(FLICK_POS);
    }

}