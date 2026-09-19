/*
This sample FTC OpMode uses methods of the Datalogger class to specify and
collect robot data to be logged in a CSV file, ready for download and charting.

For instructions, see the tutorial at the FTC Wiki:
https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Datalogging


The Datalogger class is suitable for FTC OnBot Java (OBJ) programmers.
Its methods can be made available for FTC Blocks, by creating myBlocks in OBJ.

Android Studio programmers can see instructions in the Datalogger class notes.

Credit to @Windwoes (https://github.com/Windwoes).

*/


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Meet 2 Code 2 Rev 2", group = "Dev 2")
public class meetTwoCodeTwo extends LinearOpMode
{
    Datalog datalog;
    BNO055IMU imu;
    
    VoltageSensor battery;
    DcMotorEx flywheelMotor;
    double motorRpmIdeal = 0.0;
    double powerIncrement = 0.01;
    double flywheelStopperStopPos = 0.4;
    double flywheelStopperShootPos = 0.7;
    // Servo flywheelStopperServo;
    int longZoneRPM = 3500;
    // Servo shooterRampServo;
    CRServo transferServo;
    ElapsedTime startTime;
    double transferMoveSpeed = 0.5;
    double currentTransferSpeed = 0.0; 
//    DcMotor intakeMotor;
    boolean spinIntake = false;
//  webcam
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    boolean targetFound;
    double rxInput;
    void updateMotorPower()
    {
        
        if(gamepad1.b)
        {
            motorRpmIdeal = 0.0;
            // flywheelStopperServo.setPosition(0.0);
            convertToCounts();
        }
        if(gamepad1.a)
        {
            motorRpmIdeal = 2500;
            // flywheelStopperServo.setPosition(0.2);
            convertToCounts();
        }
        if(gamepad1.y)
        {
            //  motorRpmIdeal = 3300;
            motorRpmIdeal = 3000;
            // flywheelStopperServo.setPosition(0.6);
            convertToCounts();
        }
        if(gamepad1.dpad_up)
        {
            // flywheelStopperServo.setPosition(0.8);
            motorRpmIdeal = 4400;
            convertToCounts();
        }
        if(gamepad1.dpad_down)
        {
            // flywheelStopperServo.setPosition(1.0);
            motorRpmIdeal = 4200;
            convertToCounts();
        }
        if(gamepad1.dpad_right)
        {
        }
        if(gamepad1.dpad_left)
        {

        }

//        Intake
        if(gamepad1.right_bumper)
        {
//            spinIntake = true;
            // one is bottom
            // shooterRampServo.setPosition(0.25);
            // flywheelStopperServo.setPosition(flywheelStopperStopPos);
        }
        if(gamepad1.left_bumper)
        {
//            spinIntake = false;
            // 0 is top
            // shooterRampServo.setPosition(0.11);
            // flywheelStopperServo.setPosition(flywheelStopperStopPos);
        }

//        transfer
        if(gamepad1.right_trigger >= 0.5)
        {

            transferServo.setPower(1.0);
            // flywheelStopperServo.setPosition(flywheelStopperShootPos);
        }
        else if(gamepad1.left_trigger >= 0.5)
        {
            transferServo.setPower(-1.0);
        }
        else
        {
           transferServo.setPower(0.0);
        }
    }
    
    void convertToCounts()
    {
        motorRpmIdeal /= 60;
        motorRpmIdeal *= 28;
        
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        targetFound = false;
        initAprilTag();

        // Drive train
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        // Intake
        
        
        
        // Hood
        // flywheelStopperServo = hardwareMap.get(Servo.class, "flywheelStopperServo");
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "FlywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // shooterRampServo = hardwareMap.get(Servo.class, "shooterRampServo");



//        Webcam
//        if (USE_WEBCAM)
//            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur



        startTime = new ElapsedTime();
        // Get devices from the hardwareMap.
        // If needed, change "Control Hub" to (e.g.) "Expansion Hub 1".
        battery = hardwareMap.voltageSensor.get("Control Hub");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Initialize the datalog
        datalog = new Datalog("datalog_01");
        
        // You do not need to fill every field of the datalog
        // every time you call writeLine(); those fields will simply
        // contain the last value.
        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        datalog.opModeStatus.set("RUNNING");

        for (int i = 0; opModeIsActive(); i++)
        {

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            rxInput = rx;
            doOpModeVision();
            if(gamepad1.x && targetFound)
            {
                TurnToAprilTag();
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rxInput), 1);
            double frontLeftPower = (y + x + rxInput) / denominator;
            double backLeftPower = (y - x + rxInput) / denominator;
            double frontRightPower = (y - x - rxInput) / denominator;
            double backRightPower = (y + x - rxInput) / denominator;
            
            
            
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(-backLeftPower);
            frontRightMotor.setPower(-frontRightPower);
            backRightMotor.setPower(backRightPower);
            //  handle shooter code
            updateMotorPower();




/*
            if(spinIntake)
            {
                intakeMotor.setPower(-1.0);
            }
            else
            {
                intakeMotor.setPower(0.0);
            }
*/


            flywheelMotor.setVelocity(motorRpmIdeal);
            // Note that the order in which we set datalog fields
            // does *not* matter! The order is configured inside
            // the Datalog class constructor.

            // calculate rotations per second/minute
            double countsPerSecond = flywheelMotor.getVelocity();
            double rpm = (countsPerSecond/28) * 60;
            double dgps = (flywheelMotor.getVelocity(AngleUnit.DEGREES));
            
            datalog.encoderCounts.set(countsPerSecond);
            datalog.calculatedRpm.set(rpm);
            datalog.motorDgsps.set(dgps);
            datalog.time.set(startTime.seconds());

            
            datalog.loopCounter.set(i);
            datalog.battery.set(battery.getVoltage());

            Orientation orientation = imu.getAngularOrientation();
            datalog.time.set(startTime.seconds());
            // The logged timestamp is taken when writeLine() is called.
            datalog.writeLine();

            // Datalog fields are stored as text only; do not format here.
            telemetry.addData("Time (ms)", startTime.seconds());
            
            telemetry.addData("RPM: ", datalog.calculatedRpm);

            telemetry.addLine();
            telemetry.addData("OpMode Status", datalog.opModeStatus);
            telemetry.addData("Loop Counter", datalog.loopCounter);
            telemetry.addData("Battery", datalog.battery);
            telemetry.addData("Motor Power", motorRpmIdeal);


            telemetry.update();

            sleep(20);
        }

        
        /*
         * The datalog is automatically closed and flushed to disk after 
         * the OpMode ends - no need to do that manually :')
         */

    }

    private void TurnToAprilTag()
    {
        if(!targetFound)
        {
            return;
        }

        double tagBearing = desiredTag.ftcPose.bearing;
        double margin = 1;
        double divisor = 20;
        double turnAmount = rxInput;
            telemetry.addData("TagBearing", tagBearing);
        if(tagBearing > margin)
        {
            turnAmount =  Math.min(-tagBearing/divisor, 1.0);
            rxInput = turnAmount;
//            Turn left
        }
        else if(tagBearing < -margin)
        {
            turnAmount = Math.min(-tagBearing/divisor, 1.0);
            rxInput = turnAmount;
//            turn right
        }
        else
        {

            telemetry.addLine("Within margin!");
        }
    }

    private void doOpModeVision()
    {
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
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
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);

        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.left_bumper && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - 0);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
//                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

//            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        } else {

            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//            drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//            strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//            turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//            telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        //telemetry.addData("DISTANCE", desiredTag.ftcPose.range);
//        telemetry.update();
//
        // Apply desired axes motions to the drivetrain.
//        moveRobot(drive, strafe, turn);
        sleep(10);
    }

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
//            telemetry.update();
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
        }
    }

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
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField time = new Datalogger.GenericField("Time");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        
        public Datalogger.GenericField encoderCounts = new Datalogger.GenericField("Encoder counts/s");
        public Datalogger.GenericField calculatedRpm = new Datalogger.GenericField("Calculated Rpm");
        public Datalogger.GenericField motorDgsps = new Datalogger.GenericField("Encoder Degrees per second");
    
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            time,
                            loopCounter,
                            opModeStatus,
                            encoderCounts,
                            calculatedRpm,
                            motorDgsps,
                            battery
                            //Dgsps = Degrees per second
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
