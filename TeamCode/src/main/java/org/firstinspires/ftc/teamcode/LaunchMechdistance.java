package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
@Autonomous(name="Basic: Linear OpMode", group="Linear OpMode")
public class LaunchMechdistance {
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1.0;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    private DcMotor leftDrive;  //  Used to control the left drive wheel
    private DcMotor rightDrive;  //  Used to control the right drive wheel
    private DcMotor leftbackDrive; // Used to control the back left wheel
    private DcMotor rightbackDrive; // Used to control the back right wheel
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag; // Used to hold the data for a detected AprilTag
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();   // how long a thing runs for.
    private DcMotor launchPower;
    private DcMotor intake;
    private double y;
    //The variable to store our instance of the AprilTag processor.
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.

    }
    double power;
if (opModeIsActive()) {
    while (opModeisActive()); {
        telemetryAprilTag();
        telemetry.update();// Push telemetry to the Driver Station.
        }
        if (y < 10){
            launchPower = Range.clip(gpower, -1.0, 25.0);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "albert (%.2f)", launchPower);
            telemetry.update();
    }
        else (y < 20){
            launchPower = Range.clip(power, -1.0, 30.0);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "albert (%.2f)", launchPower);
            telemetry.update();}
        if(y < 30){
            launchPower = Range.clip(power, -1.0, 35.0);}
        else(y < 40){
            launchPower = Range.clip(power, -1.0, 40.0);
        }

}
}