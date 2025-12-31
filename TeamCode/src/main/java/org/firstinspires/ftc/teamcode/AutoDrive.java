package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
     * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
     * The code assumes a basic two-wheel (Tank) Robot Drivetrain
     *
     * For an introduction to AprilTags, see the ftc-docs link below:
     * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
     *
     * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
     * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
     * https://ftc-docs.firstinspires.org/apriltag-detection-values
     */
@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class ConceptAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.+
     *
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
    @TeleOp(name="Tank Drive To AprilTag", group = "Concept")
    public class AutoDrive extends LinearOpMode
    {
        // Adjust these numbers to suit your robot.
        final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)
        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN =   0.02 ;   //  Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double TURN_GAIN  =   0.01 ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 1.0;     //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.25;  //  Clip the turn speed to this max value (adjust for your robot)
        private DcMotor leftDrive;  //  Used to control the left drive wheel
        private DcMotor rightDrive;  //  Used to control the right drive wheel
        private DcMotor leftbackDrive; // back left wheel
        private DcMotor rightbackDrive; // back right wheel
        private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
        private static final int DESIRED_TAG_ID = -1;    // Choose the tag you want to approach or set to -1 for ANY tag.
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag; // Used to hold the data for a detected AprilTag
        private IMU imu;
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor launch;
        private DcMotor intake;
        double launchPower;
        double intakePower;

        @Override public void runOpMode()
        {
            boolean FlickerReady  = false;    // Set to true when flicker system is finished
            double  drive = 0;        // Desired forward power/speed (-1 to +1) +ve is forward
            double  turn = 0;        // Desired turning power/speed (-1 to +1) +ve is CounterClockwise
            boolean turn67 = false;  // Know when to trn, or not to turn

            // Initialize the Apriltag Detection process
            //initAprilTag();
            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must match the names assigned during the robot configuration.
            // step (using the FTC Robot Controller app on the phone).
            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
            //  rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            leftDrive.setPower(1);
            rightDrive.setPower(1);
            // To drive forward, most robots need the motor on one side to be reversed because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Single Gear Reduction or 90 Deg drives may require direction flips

            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftbackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightbackDrive.setDirection(DcMotor.Direction.REVERSE);
            launch.setDirection(DcMotor.Direction.FORWARD);
            intake.setDirection(DcMotor.Direction.FORWARD);

            // Wait for the driver to press Start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch START to start OpMode");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                FlickerReady = false; // It is true just to check if it the code works.
                desiredTag = null;
                /* Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }*/

                // Activates the launch system after the balls are sorted
                if (FlickerReady) {
  //                  telemetry.addData("\n>","left Drive (%.2f)",MAX_AUTO_SPEED);        (These commands should happen before the flicker is ready)
//                    telemetry.addData("\n>", "rightDrive (%.2f)", MAX_AUTO_SPEED);      (These commands should happen before the flicker is ready)
                    launch = hardwareMap.get(DcMotor.class, "albert");
                    intake = hardwareMap.get(DcMotor.class, "jimbo");
                    // telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    // telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    // telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    sleep(10);
                    //moveRobot(drive, turn);
                } else {
                    telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                }

            }
        }

        /**
         * Move robot according to desired axes motions
         * <p>
         * Positive X is forward
         * <p>
         * Positive Yaw is counter-clockwise

        public void moveRobot(double x, double yaw) {
            // Calculate left and right wheel powers.
            double leftPower    = x - yaw;
            double rightPower   = x + yaw;

            // Normalize wheel powers to be less than 1.0
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max >1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            // Send powers to the wheels.
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }

        /**
         * Initialize the AprilTag processor.

        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;

        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }
        }*/
    }
 }
}
}
}
}