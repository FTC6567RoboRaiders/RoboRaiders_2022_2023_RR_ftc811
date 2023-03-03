package RoboRaiders.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

import RoboRaiders.Pipelines.AprilTagDetectionPipeline;
import RoboRaiders.Robot.TestRobot;
import RoboRaiders.Utilities.Logger.Logger;

@Autonomous


public class DetectATandPark extends OpMode
{

    enum State {
        NOT_INITIALIZED,            // Robot has not been initialized, this is the starting state
        INITIALIZED,                // Robot has been initialized properly, occurs after pushing the INIT button
        STARTED,                    // Robot has been started, occurs after the START button has been pushed
        ON_THE_MOVE,                // Robot is on the move, that is a power other than 0.0 has been applied to the drive train motors.
        PARKED,                     // Robot is parked in one of 4 locations on the field
        STOP,                       // Robot is stopped, occurs at the end of autonomous after 30 seconds
        DONE                        // Robot is done, nothing more for it to do
    }

    State state = State.NOT_INITIALIZED;

    Logger myLogger;
    OpenCvCamera camera;
    int cameraMonitorViewId;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    TestRobot bill;

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

    /**
     * init() is run once when the INIT button is pushed on the driver station
     */
    @Override
    public void init() {
        bill = new TestRobot();
        bill.initialize(hardwareMap);
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        myLogger = new Logger("DetectATandPark");
        state = State.INITIALIZED;
        myLogger.Debug("init() - aprilTagId: ",aprilTagId);
        myLogger.Debug("init() - state: ", state.toString());
    }

    /**
     * init_loop() is run repeatedly after the INIT button has been pressed but before the PLAY or
     * START button has been pushed.
     */
    @Override
    public void init_loop() {

        telemetry.addData("Status", "Robot is stopped and Initialized...");
        aprilTagId = getAprilTag();
        telemetry.addData("AprilTagId: ", aprilTagId);
        telemetry.update();
        myLogger.Debug("init_loop() - aprilTagId: ", aprilTagId);
        myLogger.Debug("init_loop() - state: ", state.toString());
    }

    /**
     * start() is run once when the START or PLAY button on the driver station is pushed
     */
    @Override
    public void start() {

        state = State.STARTED;
        myLogger.Debug("start() - aprilTagId: ",aprilTagId);
        myLogger.Debug("start() - state: ", state.toString());
    }

    /**
     * loop() is run repeatedly after the START or PLAY button is pushed on the driver station but
     * before the STOP button is pushed on the driver station.
     */
    @Override
    public void loop() {

        telemetry.addData("State: ", state);
        myLogger.Debug("loop() - aprilTagId: ",aprilTagId);
        myLogger.Debug("loop() - state: ", state.toString());

        switch(state)  {
            case STARTED:
                bill.resetEncoders();
                bill.runWithEncoders();

                switch (aprilTagId) {
                    case 0:
                        //move left, then forward

                        break;


                    case 1:
                        //move forward
                        myLogger.Debug("loop() - Case 2");
                        telemetry.addData("Status", "Case 2");
                        telemetry.addData("aprilTagId: ", aprilTagId);

                        numofticks = bill.driveTrainCalculateCounts(12.5);
                        telemetry.addData("numofticks: ", numofticks);
                        bill.setDriveMotorPower(-0.5, -0.5, -0.5, -0.5);

                        state = State.ON_THE_MOVE;
                        break;

                    case 2:


                        break;
                    default:
                        myLogger.Debug("loop() - default");
                        telemetry.addData("No April Tag Found Parking In Default Location", aprilTagId);
                        state = State.PARKED;    // Once code is added to power the motors, then state should be set to ON_THE_MOVE
                        break;
                }
                //telemetry.addData("Robot State: ",state);
                break;

            case ON_THE_MOVE:
                if (bill.getSortedEncoderCount() >= numofticks) {
                    bill.setDriveMotorPower(0.0,0.0,0.0,0.0);
                    state = state.PARKED;
                }
                break;

            case PARKED:
                state = State.STOP;
                //telemetry.addData("Robot State: ",state);
                break;

            case STOP:
                state = State.DONE;
                telemetry.addData("Robot State: ",state);
                break;

            default:
                break;


        }
        //telemetry.update();

    }

    public int getAprilTag() {

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

               // }
            }

           // telemetry.update();
        }
        return atId;
    }

}

