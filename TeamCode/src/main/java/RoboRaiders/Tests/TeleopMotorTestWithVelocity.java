package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robot.MotorBot;
import RoboRaiders.Utilities.Logger.Logger;
import RoboRaiders.Utilities.RRStopWatch.RRStopWatch;

//----------------------------------------------------------------------------------------------
// Static Test Teleop - tests the usage of Static variable in the RoboRaidersProperties class.
// This opmode will get the static variable.
//----------------------------------------------------------------------------------------------
@TeleOp(name="Teleop Motor Test w/Velocity", group="Test Teleops")

public class TeleopMotorTestWithVelocity extends OpMode {

    enum States {
        START,
        GOING_UP,
        DEPOSITING_CONE,
        GOING_DOWN,
    }

    Logger myLogger = new Logger("TMTWV");

    static double LEFT_TURN_MAX = 257.0;
    static double RETURN_HOME = 0.0;
    static double FIVE_SECONDS = 5.0;

    public MotorBot mBot = new MotorBot();
    boolean dPadUp, dPadDown, dPadLeft, dPadRight;
    double lastEncoderPosition;
    double encoderCounts;
    double distanceToTravel;
    boolean runSomeMore = true;
    boolean firstTime = true;
    boolean busy = true;
    States myState = States.START;  // initial state

    RRStopWatch liftTimer = new RRStopWatch();



    @Override
    public void init() {

        // Initialize mBot and tell user that the robot is initialized
        mBot.initialize(hardwareMap);
        mBot.resetIMU();
     //   mBot.setMotorSpinDirectionForward();
    }

    @Override
    public void init_loop() {
        telemetry.addData("- init_loop - mBot.getMotorEncoderCounts: ",mBot.getMotorEncoderCounts());
        telemetry.addData("- init_loop -  mBot.getMotorPosition: ",mBot.getMotorPosition());
        myLogger.Debug("init_loop");
    }

    @Override
    public void loop() {

        dPadUp    = gamepad1.dpad_up;
        dPadDown  = gamepad1.dpad_down;
        dPadLeft  = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;
        myLogger.Debug("myState: ",String.valueOf(myState));

        switch(myState) {
            case START:

                // When dpadLeft is pushed
                if (dPadLeft) {
                    distanceToTravel = 2.51 / 4.0;
                    encoderCounts = mBot.calculateMotorEncodeCounts(distanceToTravel);
                    mBot.setMotorTargetPosition((int)encoderCounts);
                    mBot.runMotorWithEncodersSTP();
                    mBot.setMotorVelocity(200.0);
                    myState = States.GOING_UP;
                    myLogger.Debug("myState, dPadLeft",String.valueOf(myState+", "+dPadLeft));
                }
                break;

            case GOING_UP:
              //  myLogger.Debug("myState, encoder",String.valueOf(Math.abs(mBot.getMotorPosition() - LEFT_TURN_MAX)));
                myLogger.Debug("myState, encoder",String.valueOf(myState) + ", " + String.valueOf(mBot.getMotorPosition()) + ", " + String.valueOf(Math.abs(mBot.getMotorPosition() - LEFT_TURN_MAX)));
                if (Math.abs(mBot.getMotorPosition() - LEFT_TURN_MAX) < 10) {
                    mBot.setMotorVelocity(0.0);
                    myState = States.DEPOSITING_CONE;
                    liftTimer.startTime();  // restart the timer
                }
                break;

            case DEPOSITING_CONE:
                telemetry.addData("myState: ", myState);
                telemetry.addData("Depositing cone ","for 5 seconds - just for show");
                telemetry.addData("Press A Button to Deposit", " and move to next state");
//                telemetry.addData("elapsed time: ", liftTimer.getElaspedTime());
//                telemetry.addData("5 Seconds: ",FIVE_SECONDS);
//                if (liftTimer.getElaspedTime() >= FIVE_SECONDS) {
                if (gamepad1.a) {
                        mBot.setMotorTargetPosition(0);
                        //       mBot.runMotorWithEncodersSTP();
                        //       mBot.setMotorSpinDirectionForward();
                        mBot.setMotorVelocity(200.0);
                        myState = States.GOING_DOWN;
                }
                //myLogger.Debug("myState, encoder ",String.valueOf(myState) + ", " + String.valueOf(mBot.getMotorPosition()));
                break;

            case GOING_DOWN:
                myLogger.Debug("myState, encoder ",String.valueOf(myState) + ", " + String.valueOf(mBot.getMotorPosition()) + ", " + String.valueOf(Math.abs(mBot.getMotorPosition() - RETURN_HOME)));
                if (Math.abs(mBot.getMotorPosition() - RETURN_HOME) < 10) {
                    mBot.setMotorVelocity(0.0);
                    myState = States.START;
                }
                break;
            default:
                myState = States.START;

        }
//        telemetry.addData("dPadUp:    ",dPadUp);
//        telemetry.addData("dPadDown:  ",dPadDown);
//        telemetry.addData("dPadLeft:  ",dPadLeft);
//        telemetry.addData("dPadRight: ",dPadRight);

 //       if (dPadLeft) {
//        telemetry.addData("runSomeMore: ",runSomeMore);
//        telemetry.addData("firstTime: ",firstTime);
//        telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
//        telemetry.addData("busy: ",busy);
//
//        telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
//        telemetry.addData("velocity: ",mBot.aMotor.getVelocity());
//        telemetry.addData("encoderCounts: ",encoderCounts);
//
//        if (firstTime) {
//            distanceToTravel = 2.51 / 8.0;
//            encoderCounts = -mBot.turretCalculateCounts(distanceToTravel);
//            telemetry.addData("encoderCounts: ",encoderCounts);
//            telemetry.addData("mBot.aMotor.encoderCount: ",mBot.getEncoderCounts());
//            telemetry.addData("velocity: ",mBot.aMotor.getVelocity());
//
//            mBot.setMotorTargetPosition((int)encoderCounts);
//            mBot.runWithEncodersSTP();
//            mBot.setDriveMotorVelocity(200.0);
//            firstTime = false;
//        }
//        if (runSomeMore) {
//
//            telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
//            telemetry.addData("busy: ",busy);
//
//            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
//            telemetry.addData("velocity: ",mBot.aMotor.getVelocity());
//            telemetry.addData("encoderCounts: ",encoderCounts);
//
//            if (!mBot.aMotor.isBusy()) {
//
//                busy = false;
//                mBot.setMotorVelocity(0.0);   // we are done!!!!
//             //   mBot.runWithoutEncoders();
//                mBot.resetMotorEncoders();
//                runSomeMore = false;
//            }
//        }
//        else {
//            telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
//            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
//            telemetry.addData("velocity: ",mBot.aMotor.getVelocity());
//            telemetry.addData("encoderCounts: ",encoderCounts);
//            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getTargetPositionTolerance());
//            telemetry.addData("mBot.aMotor.encoderCount: ",mBot.getMotorEncoderCounts());
//        }
    }

}
