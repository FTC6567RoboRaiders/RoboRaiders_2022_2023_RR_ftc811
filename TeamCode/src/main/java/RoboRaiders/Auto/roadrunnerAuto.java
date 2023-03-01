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
public class roadrunnerAuto extends LinearOpMode {

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


        Pose2d startPose = new Pose2d(-33, 39, Math.toRadians(-90));
        Pose2d parkPose = new Pose2d(-35.8,14,Math.toRadians(0));

        Pose2d approachPose = new Pose2d(-33, 63, Math.toRadians(-90));
        Pose2d pose2 = new Pose2d(-39, 54,Math.toRadians(-135));

        drive.setPoseEstimate(approachPose);
        Trajectory step1A = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0.5, () -> {
                    Chuckbot.setLiftPositionMid();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
                .strafeRight(3)
                .build();
        Trajectory step2A = drive.trajectoryBuilder(step1A.end())
                .lineTo(new Vector2d(-36, 36), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .splineTo(new Vector2d(-35,44), Math.toRadians(-135))
//                        .splineTo(new Vector2d(-36.5, 47), Math.toRadians(-90))
                .build();

        Trajectory step3A = drive.trajectoryBuilder(pose2)
                .lineTo(new Vector2d(-48, 39), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory step4A = drive.trajectoryBuilder(step3A.end())
                .strafeRight(5)
                .build();
        Trajectory step5A = drive.trajectoryBuilder(step4A.end())
                .back(15)
                .build();
        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(.75, () -> {
                    //After 3/4s of a second, have the lift go up
                    Chuckbot.setLiftPositionMid();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
                .splineToConstantHeading(new Vector2d(-36.5,52),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36.5, 47), Math.toRadians(-90))
                .forward(21.75)
                .splineTo(new Vector2d(-40,10), Math.toRadians(-136))
                .addTemporalMarker(1.25, () -> {
                    //After 1.25 seconds rotate the turret to the back position
                    Chuckbot.setTurretPositionBack();
                    Chuckbot.turretRunWithEncodersSTP();
                    Chuckbot.setTurretMotorVelocity(110);
                })
                .build();
        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .lineTo(new Vector2d(-29.0,23.25))   //align to medium junction
                .addTemporalMarker(3.25, () -> {
                    //code in here, this is where we put the code to lift the lift.
                    while (Math.abs(Chuckbot.getTurretEncoderCounts() - (-182.0)) > 5.0) {  //wait for the turret to get to the right position
                        telemetry.addData("turret encoder count: ", String.valueOf(Math.abs(Chuckbot.getTurretEncoderCounts() - (-182.0))));
                        telemetry.update();
                    }
                    Chuckbot.setLiftPositionMidDeposit();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(1500.0);
                })
                .addTemporalMarker(4.0, () -> {
                    Chuckbot.setinTakeServoPosition(1.0);  //Deposit cone
                })
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .forward(.00001)
                .addTemporalMarker(.05, () -> {
                    Chuckbot.setTurretPositionConeStack();
                    Chuckbot.turretRunWithEncodersSTP();
                    Chuckbot.setTurretMotorVelocity(200);
                })
//                .addTemporalMarker(.6, () -> {
//                    Chuckbot.setinTakeServoPosition(0.0);
//                })
                .build();

        Trajectory step35 = drive.trajectoryBuilder(step3.end())
                .addTemporalMarker(.01, () -> {
                    Chuckbot.setLiftPositionStack5Collect(1500);
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
                .splineTo(new Vector2d(-58, 15.25), Math.toRadians(-180)) // Wade changed the cords, the were (-58, 13)
                .forward(4.5) // this was in step 4 originally - drive to the cone stack
                .build();



        Trajectory step4 = drive.trajectoryBuilder(step35.end())  // get to the high junction
                .back(2.5)
                .splineTo(new Vector2d(-31, 7.5), Math.toRadians(-40))
                .build();
//                .addTemporalMarker(.05, () -> {
//                    while( Math.abs(Chuckbot.getTurretEncoderCounts() - Chuckbot.lift_stack5_collect) > 5.0 ) {
//                            // Do nothing but let the turret get to where it needs to be before we pick up another cone
//                    }
//                    Chuckbot.setLiftPositionHigh();
//                    Chuckbot.setLiftRunWithEncodersSTP();
//                    Chuckbot.setLiftMotorVelocity(4000.0);
//                })
//                .addTemporalMarker(.1, () -> {

   //             })
// code gods, make the robot do 6 cone pickup auto :)

        Trajectory step5 = drive.trajectoryBuilder(step4.end())   //Deposit cone, robot is at the high junction
                .forward(0.5) // was 1.5
                .addTemporalMarker(1.0, () -> {
                    Chuckbot.setTurretPositionHighJunc();
                    Chuckbot.turretRunWithEncodersSTP();
                    Chuckbot.setTurretMotorVelocity(200);
                })
                .addTemporalMarker(3.25, () -> {
                    //code in here, this is where we put the code to lift the lift.
                    while (Math.abs(Chuckbot.getTurretEncoderCounts() - (Chuckbot.turret_highBack)) > 5.0) {
                        telemetry.addData("turret encoder count: ", String.valueOf(Math.abs(Chuckbot.getTurretEncoderCounts() - (-185))));
                        telemetry.update();
                    }
                })
                .addTemporalMarker(4.25, () -> {
                    Chuckbot.setinTakeServoPosition(1.0);  //Deposit cone
                })
                .build();

        Trajectory step6 = drive.trajectoryBuilder(step5.end())  // Start parking trajectory
                .forward(12)
                .addTemporalMarker(1.0, () -> {
                    Chuckbot.setTurretPositionHome();
                    Chuckbot.turretRunWithEncodersSTP();
                    Chuckbot.setTurretMotorVelocity(200);
                    Chuckbot.setLiftPositionGround();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
                .build();
        Trajectory step7 = drive.trajectoryBuilder(parkPose)
                .back(24)
                .build();
        Trajectory step8 = drive.trajectoryBuilder(parkPose)
                .forward(20)
                .build();

        waitForStart();
        drive.followTrajectory(step1A);
        drive.followTrajectory(step2A);
        drive.turn(Math.toRadians(-90));

        drive.setPoseEstimate(pose2);
        drive.followTrajectory(step3A);
        drive.followTrajectory(step4A);
        drive.followTrajectory(step5A);
        drive.turn(Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        drive.followTrajectory(step1);
        drive.followTrajectory(step2);
        sleep(1000);
        drive.followTrajectory(step3);
        sleep(250);
        drive.followTrajectory(step35);
        sleep(250);
        Chuckbot.setLiftPositionStack5Collect(1300.0);// added by wade
        sleep(1000);
        Chuckbot.setinTakeServoPosition(0.0); // in take a cone
        sleep(1000);
        Chuckbot.setLiftPositionHigh();
        Chuckbot.setLiftRunWithEncodersSTP();
        Chuckbot.setLiftMotorVelocity(4000.0);
        sleep(250);
        drive.followTrajectory(step4);
        drive.followTrajectory(step5);
        sleep(1000);
        drive.followTrajectory(step6);
        drive.turn(Math.toRadians(40));
        sleep(500);

        drive.setPoseEstimate(parkPose);

        if(aprilTagId == 0){
            drive.followTrajectory(step7);
        }
        else if(aprilTagId == 2){
            drive.followTrajectory(step8);
        }

        // add code to determine where to part based on april tags
//       Chuckbot.setinTakeServoPosition(1.0); // deposit cone
//        drive.followTrajectory(step4); // added by wade



//        Chuckbot.setLiftPositionMidDeposit();
//        Chuckbot.setLiftRunWithEncodersSTP();
//        Chuckbot.setLiftMotorVelocity(3000.0);

        while (opModeIsActive()) {
            telemetry.addData("Servo Position: ", Chuckbot.getInTakePosition());

//            Chuckbot.setinTakeServoPosition(1.0);
            telemetry.addData("Servo Position: ", Chuckbot.getInTakePosition());
            telemetry.update();
        }

//        drive.followTrajectory(step3);
//        drive.turn(Math.toRadians(-39));
//        drive.followTrajectory(step3);
//        drive.followTrajectory(step5);






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
