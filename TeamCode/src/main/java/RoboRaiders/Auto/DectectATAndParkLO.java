/**
 * This is DetectATAndParkLO, LO for LinearOpMode.  This performs the same function as
 * DetectATandPark except this "extends" LinearOpMode, whereas, DetectAtandPark "extends"
 * OpMode.
 */

package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.config.TrajectoryConfig;
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
import RoboRaiders.Robot.TestRobot;
import RoboRaiders.Utilities.Logger.Logger;

@Autonomous

public class DectectATAndParkLO extends LinearOpMode {

    Logger myLogger;
    OpenCvCamera camera;
    int cameraMonitorViewId;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TestRobot stevesRobot;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double numofticks;


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

        /**
         * Initialize some stuff
         */
        myLogger = new Logger("DectectATAndParkLO");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RR_ChuckBot_LIT Chuckbot = new RR_ChuckBot_LIT();
        Chuckbot.initialize(hardwareMap);
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


        // waitForStart();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Robot is stopped and Initialized...");
            aprilTagId = getAprilTag();
            telemetry.addData("AprilTagId: ", aprilTagId);
            telemetry.update();
            myLogger.Debug("init_loop() - aprilTagId: ", aprilTagId);
        }

        /*
        Values For Different Directions:
        (pos, neg, neg, pos) = strafe left
        (neg, pos, pos, neg) = strafe right
        (neg, neg ,neg, neg) = move forward
        (pos, pos, pos, pos) = move backward
         */
        //This Part Will Deposit Cone, Same Code Regardless of AprilTag

        // move forward off wall
//        numofticks = stevesRobot.driveTrainCalculateCounts(1.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//        }
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        // move left
//        numofticks = stevesRobot.driveTrainCalculateCounts(20.5);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//        }
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        // move backwards into wall
//        numofticks = stevesRobot.driveTrainCalculateCounts(5.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//        }
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        //move forward
//        numofticks = stevesRobot.driveTrainCalculateCounts(19.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//
//        }
//        if(stevesRobot.getHeading() > 0.0) {
//            stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//            while (stevesRobot.getHeading() > 0.02) {
//                stevesRobot.setDriveMotorPower(0.1, -0.1, 0.1, -0.1);
//            }
//        }
//        else if(stevesRobot.getHeading() < 0.0) {
//            stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//            while (stevesRobot.getHeading() < -0.02) {
//                stevesRobot.setDriveMotorPower(-0.1, 0.1, -0.1, 0.1);
//            }
//        }
//        numofticks = stevesRobot.driveTrainCalculateCounts(19.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//
//        }
//        if(stevesRobot.getHeading() > 0.0) {
//            stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//            while (stevesRobot.getHeading() > 0.02) {
//                stevesRobot.setDriveMotorPower(0.2, -0.2, 0.2, -0.2);
//            }
//        }
//        else if(stevesRobot.getHeading() < 0.0) {
//            stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//            while (stevesRobot.getHeading() < 0.02) {
//                stevesRobot.setDriveMotorPower(-0.2, 0.2, -0.2, 0.2);
//            }
//        }
//
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        myLogger.Debug("ROBOT HEADING ROBOT HEADING ROBOT HEADING", stevesRobot.getHeading());
//        sleep(1000);
//
//        //move lift to high junction height
//        stevesRobot.setLiftMotorTargetPosition(-7750);
//        stevesRobot.liftRunWithEncodersSTP();
//        stevesRobot.setLiftMotorVelocity(1500.0);
//        while (Math.abs(stevesRobot.getLiftEncoderCounts() + 7750) < 5.0){
//            stevesRobot.liftRunWithoutEncoders();
//            telemetry.addData("getLiftEncoderCounts()", stevesRobot.getLiftEncoderCounts());
//        }
//
//        sleep(5000);
//
//        // move turret to pole
//        stevesRobot.setTurretMotorTargetPosition(-89.0);
//        stevesRobot.turretRunWithEncodersSTP();
//        stevesRobot.setTurretMotorVelocity(600.0);
//        while (Math.abs(stevesRobot.getTurretEncoderCounts() + 89.0) < 89.0) {
//            telemetry.addData("getTurretEncoderCounts()", stevesRobot.getTurretEncoderCounts());
//        }
//
//        sleep(1500);
//
//        // move closer to pole
//        numofticks = stevesRobot.driveTrainCalculateCounts(4.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//        }
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        sleep(2000);
//
//        // deposit
//        stevesRobot.setinTakeServoPosition(1.0);
//
//        sleep(2000);
//
//        // move away from pole
//        numofticks = stevesRobot.driveTrainCalculateCounts(4.0);
//        telemetry.addData("numofticks: ", numofticks);
//        stevesRobot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
//        while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//            telemetry.addData("getSortEncoderCount: ", stevesRobot.getSortedEncoderCount());
//        }
//        stevesRobot.resetEncoders();
//        stevesRobot.runWithEncoders();
//        stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//        //move lift to starting position
//        stevesRobot.setLiftMotorTargetPosition(0);
//        stevesRobot.liftRunWithEncodersSTP();
//        stevesRobot.setLiftMotorVelocity(1500.0);
//        while (Math.abs(stevesRobot.getLiftEncoderCounts() - 7750) < 5.0){
//            stevesRobot.liftRunWithoutEncoders();
//            telemetry.addData("getLiftEncoderCounts()", stevesRobot.getLiftEncoderCounts());
//        }

        switch (aprilTagId) {
            case 0:
                Pose2d startPose = new Pose2d(-33, 63, Math.toRadians(-90));
                Pose2d pose2 = new Pose2d(-39, 54,Math.toRadians(-135));


                drive.setPoseEstimate(startPose);

                Trajectory step1 = drive.trajectoryBuilder(startPose)
                        .addTemporalMarker(0.5, () -> {
                            Chuckbot.setLiftPositionMid();
                            Chuckbot.setLiftRunWithEncodersSTP();
                            Chuckbot.setLiftMotorVelocity(3000.0);
                        })
                        .strafeRight(3)
                        .build();
                Trajectory step2 = drive.trajectoryBuilder(step1.end())
                        .lineTo(new Vector2d(-36, 36), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .splineTo(new Vector2d(-35,44), Math.toRadians(-135))
//                        .splineTo(new Vector2d(-36.5, 47), Math.toRadians(-90))
                        .build();

                Trajectory step3 = drive.trajectoryBuilder(pose2)
                        .lineTo(new Vector2d(-48, 36), SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                Trajectory step54 = drive.trajectoryBuilder(step3.end())

                        .back(12)
                        .build();
                Trajectory step5 = drive.trajectoryBuilder(step3.end())

                        .back(12)
                        .build();

                telemetry.addData("Status: ", "case 1");
                drive.followTrajectory(step1);
                drive.followTrajectory(step2);
                drive.turn(Math.toRadians(-90));

                drive.setPoseEstimate(pose2);
                drive.followTrajectory(step3);

                drive.followTrajectory(step5);
                drive.turn(Math.toRadians(90));

                break;

           // case 1:
//                telemetry.addData("Status: ", "case 2");
//
//                // move forward off wall
//                numofticks = stevesRobot.driveTrainCalculateCounts(1.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                telemetry.update();
//
//                // move backwards into wall
//                numofticks = stevesRobot.driveTrainCalculateCounts(42.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move forward off wall
//                numofticks = stevesRobot.driveTrainCalculateCounts(1.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move right
//                numofticks = stevesRobot.driveTrainCalculateCounts(24.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move backwards
//                numofticks = stevesRobot.driveTrainCalculateCounts(5.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move forwards to parking zone
//                numofticks = stevesRobot.driveTrainCalculateCounts(38.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                break;
//
//            case 2:
//                telemetry.addData("Status: ", "case 2");
//
//                // move forward off wall
//                numofticks = stevesRobot.driveTrainCalculateCounts(1.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                telemetry.update();
//
//                // move right
//                numofticks = stevesRobot.driveTrainCalculateCounts(52.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move backwards
//                numofticks = stevesRobot.driveTrainCalculateCounts(5.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                // move forwards to parking zone
//                numofticks = stevesRobot.driveTrainCalculateCounts(38.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortedEncoderCount: ", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                break;

        }

        //old cases below

//        switch (aprilTagId) {
//            case 0:
//                //move left, then forward
//                telemetry.addData("Status", "Case 1");
//
//                numofticks = stevesRobot.driveTrainCalculateCounts(1);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//                numofticks =  stevesRobot.driveTrainCalculateCounts(20.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, -0.25, -0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//                numofticks = stevesRobot.driveTrainCalculateCounts(1);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//                numofticks = stevesRobot.driveTrainCalculateCounts(4);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//
//                sleep(100);
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//                numofticks = stevesRobot.driveTrainCalculateCounts(30);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//                break;
//
//            case 1:
//                myLogger.Debug("loop() - Case 2");
//                telemetry.addData("Status", "Case 2");
//                telemetry.addData("aprilTagId: ", aprilTagId);
//
//                //Move right
//                numofticks = stevesRobot.driveTrainCalculateCounts(3);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                //Move forward
//                numofticks = stevesRobot.driveTrainCalculateCounts(30);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                break;
//
//            case 2:
//                //move right then forward
//                telemetry.addData("Status", "Case 3");
//
//                //Move forward
//                numofticks = stevesRobot.driveTrainCalculateCounts(0.7);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                //Move right
//                numofticks =  stevesRobot.driveTrainCalculateCounts(28.0);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, 0.25, 0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                //Move backwards
//                numofticks = stevesRobot.driveTrainCalculateCounts(2);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks) {
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                //Move backwards
//                numofticks = stevesRobot.driveTrainCalculateCounts(4);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(0.25, 0.25, 0.25, 0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                sleep(100);
//
//                stevesRobot.resetEncoders();
//                stevesRobot.runWithEncoders();
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//
//                //Move forward
//                numofticks = stevesRobot.driveTrainCalculateCounts(30);
//                telemetry.addData("numofticks: ", numofticks);
//                stevesRobot.setDriveMotorPower(-0.25, -0.25, -0.25, -0.25);
//                while (opModeIsActive() && stevesRobot.getSortedEncoderCount() <= numofticks){
//                    telemetry.addData("getSortEncoderCount()", stevesRobot.getSortedEncoderCount());
//                }
//
//                telemetry.update();
//                stevesRobot.setDriveMotorPower(0.0, 0.0, 0.0, 0.0);
//                break;
//
//            default:
//                myLogger.Debug("loop() - default");
//                telemetry.addData("No April Tag Found Parking In Default Location", aprilTagId);
//                break;
//        }
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
