package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import RoboRaiders.Pipelines.AprilTagDetectionPipeline;
import RoboRaiders.Robot.RR_ChuckBot_LIT;

import RoboRaiders.Teleop.TestBotTeleop;
import RoboRaiders.Utilities.Logger.Logger;

@Autonomous
public class ParkOnlyAuto extends LinearOpMode {

    OpenCvCamera camera;
    int cameraMonitorViewId;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double numofticks;

    Logger myLogger;
    // UNITS ARE METERS
    double tagsize = 0.166;

    int numFramesWithoutDetection = 0;
    int aprilTagId;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    final int[] OUR_APRIL_TAGS = {0,1,2};


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RR_ChuckBot_LIT Chuckbot = new RR_ChuckBot_LIT();
        Chuckbot.initialize(hardwareMap);
        myLogger = new Logger("DectectATAndParkLO");
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Robot is stopped and Initialized...");
            aprilTagId = getAprilTag();
            telemetry.addData("AprilTagId: ", aprilTagId);
            telemetry.update();
            myLogger.Debug("init_loop() - aprilTagId: ", aprilTagId);
        }

        while (opModeIsActive()) {
            telemetry.addData("Servo Position: ", Chuckbot.getInTakePosition());

//            Chuckbot.setinTakeServoPosition(1.0);
            telemetry.addData("Servo Position: ", Chuckbot.getInTakePosition());
            telemetry.update();
        }

    }
    public int getAprilTag() {
        boolean april_tag_found = false;
        int atId = 0;
        // Calling getDetectionsUpdate() will only return an object if there was a new frame
        // processed since the last time we called it. Otherwise, it will return null. This
        // enables us to only run logic when there has been a new frame, as opposed to the
        // getLatestDetections() method which will always return an object.

        //ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();  // Changed to this line to always return an object

        // If there's been a new frame...
        if (detections != null) {


            // If we don't see any tags
            if (detections.size() == 0) {
                numFramesWithoutDetection++;
                telemetry.addData("numFramesWithoutDetection: ",numFramesWithoutDetection);

                // If we haven't seen a tag for a few frames, lower the decimation
                // so we can hopefully pick one up if we're e.g. far back
                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    telemetry.addData("setting decimation to: ", "LOW");
                }
            }
            // We do see tags!
            else {
                numFramesWithoutDetection = 0;

                // If the target is within 1 meter, turn on high decimation to
                // increase the frame rate
                if (detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    telemetry.addData("setting decimation to: ", "HIGH");
                }

                // get the first detection rather than all the detections, we will need to test this
                // out on the field since there could be other teams using apriltags for their sleeve
                // that are across the field, so we just need to print another sleeve and put in the
                // position across field from where the robot is stating from.
                AprilTagDetection detection = detections.get(0);

                // for (AprilTagDetection detection : detections) {
                //     telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

                // get the detection id information and stash it into a variable for now
                atId = detection.id;

                for (int i = 0; i<=2; i++){
                    if(atId == OUR_APRIL_TAGS[i]){
                        april_tag_found = true;
                    }

                }

                // }
            }

            // telemetry.update();
        }
        if(april_tag_found){
            return atId;
        }
        else{
            return 99999999;
        }

    }
}
