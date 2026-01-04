package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@Autonomous(name="Basic: Linear OpMode", group="Linear OpMode")
public class LaunchMechdistance extends LinearOpMode {
    final double SPEED_GAIN = 0.02;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 1.0;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
    private DcMotor leftDrive;  //  Used to control the left drive wheel
    private DcMotor rightDrive;  //  Used to control the right drive wheel
    private DcMotor leftbackDrive; // Used to control the back left
    private DcMotor rightbackDrive; // Used to control the back right wheel
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagDetection desiredTag; // Used to hold the data for a detected AprilTag
    private IMU imu;   // Built in coordinates. Stands for intergral mesaurement unit
    private ElapsedTime runtime = new ElapsedTime();   // how long it runs for.
    private DcMotor launch;   // Motor for launch Mechanisim
    private Double launchSpeed;
    private double range;
    double Power;
    private int bluebaseID20 = 20;
    private int redbaseID24 = 24;
    private AprilTagProcessor aprilTag;        // Used for managing the AprilTag detection process.

    //The variable to store our instance of the vision portal.
    @Override
    public void runOpMode() {
        initAprilTag();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        // waitForStart();    In OpMode, but using linear OpMode.

        if (this.opModeIsActive()) {
            telemetry.update();
            visionPortal.resumeStreaming();
            sleep(20);
        } else {
            visionPortal.stopStreaming();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        // get the launch velocity
        double launchVelocity = getLaunchVelocity();
        // activate the launcher
        launch.setPower(launchVelocity);
        // activate the spindexer

        //activate the flicker

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private <telemetryAprilTag> void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        //The variable to store our instance of the AprilTag processor.
  /*     if (linearOpMode.opModeIsActive()) {
            while(linearOpMode.opModeIsActive()) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    telemetry.addData("# AprilTags Detected", currentDetections.size());
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));    // Only need the y (distance)
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                        } else {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                        }
                    }   // end for() loop

                    // Add "key" information to telemetry
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
                    telemetry.addLine("RBE = Range, Bearing & Elevation");
                    telemetry.update();// Push telemetry to the Driver Station.
                }
                }
                telemetry.addData("Status", "Run Time: " + runtime.toString());    // How long the motor is running for
                telemetry.addData("Motors", "launch (%.2f)", Power);   //Updates the motor
                telemetry.update();
                // Spin_dexer() for later when spin-dexer class
                // flick()       for later, when flicker class is finished.
                launch.setPower(0);     // Stops the motor
            }
        }   */
    }

    private double getLaunchVelocity() {
        double verticalD = 0.0;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection at : currentDetections) {
            if (at.metadata.id == bluebaseID20 || at.metadata.id == redbaseID24) {
                verticalD = at.ftcPose.y;
            }
        }
        double speedFactor = -1.0;
        if (verticalD < 10.0) {
            speedFactor = 0.125;    // Setting the power, based on the distance (in if statements above)j
        } else if (verticalD < 20.0 && verticalD > 10.0) {
            speedFactor = 0.25;   // Setting the power, based on the distance (in if statements above)
        } else if (verticalD > 20.0 && verticalD < 30.0) {
            speedFactor = 0.5;   // Setting the power, based on the distance (in if statements above)
        }
        return speedFactor;
    }
}
