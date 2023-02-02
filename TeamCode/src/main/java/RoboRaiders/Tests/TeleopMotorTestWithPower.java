package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import RoboRaiders.Robot.MotorBotWithDCMOTOR;

//----------------------------------------------------------------------------------------------
// Static Test Teleop - tests the usage of Static variable in the RoboRaidersProperties class.
// This opmode will get the static variable.
//----------------------------------------------------------------------------------------------
@Disabled
@TeleOp(name="Teleop Motor Test w/Power", group="Test Teleops")


public class TeleopMotorTestWithPower extends OpMode {

    public MotorBotWithDCMOTOR mBot = new MotorBotWithDCMOTOR();
    boolean dPadUp, dPadDown, dPadLeft, dPadRight;
    double lastEncoderPosition;
    double encoderCounts;
    double distanceToTravel;
    boolean runSomeMore = true;
    boolean firstTime = true;
    boolean busy = true;



    @Override
    public void init() {

        // Initialize mBot and tell user that the robot is initialized
        mBot.initialize(hardwareMap);
        mBot.resetIMU();
    }

    @Override
    public void init_loop() {
        mBot.resetEncoders();
        telemetry.addData("- init_loop - mBot.aMotor.encoderCount: ",mBot.getEncoderCounts());
    }

    @Override
    public void loop() {

        dPadUp    = gamepad1.dpad_up;
        dPadDown  = gamepad1.dpad_down;
        dPadLeft  = gamepad1.dpad_left;
        dPadRight = gamepad1.dpad_right;

//        telemetry.addData("dPadUp:    ",dPadUp);
//        telemetry.addData("dPadDown:  ",dPadDown);
//        telemetry.addData("dPadLeft:  ",dPadLeft);
//        telemetry.addData("dPadRight: ",dPadRight);

        //       if (dPadLeft) {
        telemetry.addData("------------loop","()------------");
        telemetry.addData("runSomeMore: ",runSomeMore);
        telemetry.addData("firstTime: ",firstTime);
        telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
        telemetry.addData("busy: ",busy);

        telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
        telemetry.addData("power: ",mBot.aMotor.getPower());
        telemetry.addData("encoderCounts: ",encoderCounts);

        if (firstTime) {
            distanceToTravel = 2.74 / 4.0;
            encoderCounts = mBot.turretCalculateCounts(distanceToTravel);
            mBot.aMotor.setDirection(DcMotor.Direction.REVERSE);
            telemetry.addData("-------firstTime loop","()-------");
            telemetry.addData("encoderCounts: ",encoderCounts);
            telemetry.addData("mBot.aMotor.encoderCount: ",mBot.getEncoderCounts());
            telemetry.addData("power: ",mBot.aMotor.getPower());

            mBot.setTargetPosition((int)encoderCounts);
            mBot.runWithEncodersSTP();
            mBot.setDriveMotorPower(0.25);
            firstTime = false;
        }
        if (runSomeMore) {

            telemetry.addData("-------runSomeMore(TRUE) loop","()-------");
            telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
            telemetry.addData("busy: ",busy);

            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
            telemetry.addData("power: ",mBot.aMotor.getPower());
            telemetry.addData("encoderCounts: ",encoderCounts);

            if (!mBot.aMotor.isBusy()) {

                busy = false;
                mBot.setDriveMotorPower(0.0);   // we are done!!!!
                //   mBot.runWithoutEncoders();
                mBot.resetEncoders();
                runSomeMore = false;
            }
        }
        else {
            telemetry.addData("-------runSomeMore(FALSE) loop","()-------");
            telemetry.addData("isBusy: ",mBot.aMotor.isBusy());
            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getCurrentPosition());
            telemetry.addData("power: ",mBot.aMotor.getPower());
            telemetry.addData("encoderCounts: ",encoderCounts);
            telemetry.addData("mBot.aMotor.target position: ",mBot.aMotor.getTargetPosition());
            telemetry.addData("mBot.aMotor.encoderCount: ",mBot.getEncoderCounts());
        }
    }

}
