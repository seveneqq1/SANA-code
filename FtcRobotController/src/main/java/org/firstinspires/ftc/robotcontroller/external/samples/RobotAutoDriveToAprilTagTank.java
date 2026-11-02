 




























package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

 






































@TeleOp(name="Tank Drive To AprilTag", group = "Concept")
@Disabled
public class RobotAutoDriveToAprilTagTank extends LinearOpMode
{
     
    final double DESIRED_DISTANCE = 12.0;  

     
     
     
    final double SPEED_GAIN =   0.02 ;    
    final double TURN_GAIN  =   0.01 ;    

    final double MAX_AUTO_SPEED = 0.5;    
    final double MAX_AUTO_TURN  = 0.25;   

    private DcMotor leftDrive   = null;   
    private DcMotor rightDrive  = null;   

    private static final boolean USE_WEBCAM = true;   
    private static final int DESIRED_TAG_ID = -1;     
    private VisionPortal visionPortal;                
    private AprilTagProcessor aprilTag;               
    private AprilTagDetection desiredTag = null;      

    @Override public void runOpMode()
    {
        boolean targetFound     = false;     
        double  drive           = 0;         
        double  turn            = 0;         

         
        initAprilTag();

         
         
         
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

         
         
         
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(6, 250);   

         
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;

             
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                 
                if (detection.metadata != null) {
                     
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                         
                        targetFound = true;
                        desiredTag = detection;
                        break;   
                    } else {
                         
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                     
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

             
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

             
            if (gamepad1.left_bumper && targetFound) {

                 
                double  rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError = desiredTag.ftcPose.bearing;

                 
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn  = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;

                telemetry.addData("Auto","Drive %5.2f, Turn %5.2f", drive, turn);
            } else {

                 
                drive = -gamepad1.left_stick_y  / 2.0;   
                turn  = -gamepad1.right_stick_x / 4.0;   
                telemetry.addData("Manual","Drive %5.2f, Turn %5.2f", drive, turn);
            }
            telemetry.update();

             
            moveRobot(drive, turn);
            sleep(10);
        }
    }

     






    public void moveRobot(double x, double yaw) {
         
        double leftPower    = x - yaw;
        double rightPower   = x + yaw;

         
        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max >1.0) {
            leftPower /= max;
            rightPower /= max;
        }

         
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

     


    private void initAprilTag() {
         
        aprilTag = new AprilTagProcessor.Builder().build();

         
         
         
         
         
         
         
        aprilTag.setDecimation(2);

         
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

     



    private void    setManualExposure(int exposureMS, int gain) {
         

        if (visionPortal == null) {
            return;
        }

         
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

         
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
    }
}
