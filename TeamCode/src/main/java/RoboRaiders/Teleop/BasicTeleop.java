package RoboRaiders.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Robot.TestRobot;
import RoboRaiders.Utilities.Logger.Logger;

// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name="Basic Teleop - No Automation", group="Test Teleops")

public class BasicTeleop extends OpMode {


    // Create an instance of the TestRobot and store it into StevesRobot
    public TestRobot stevesRobot = new TestRobot();
    public Logger myLogger =  new Logger("TestBotTeleop");
    public Logger dtLogger = new Logger("DT");   // Drive train logger


    @Override
    public void init() {

        // Initialize stevesRobot and tell user that the robot is initialized
        stevesRobot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }


    @Override
    public void loop() {

        /**
         * very basic teleop to run all the movements manually
         */

        telemetry.addData("        Gamepad2 Controls ", "as follows:");
        telemetry.addData("+-------------------------", "--------------------------------+");
        telemetry.addData("| Gamepad2 Right Stick X: ", "Rotates turret at 75% max power |");
        telemetry.addData("| Gamepad2 Left Stick Y:  ", "Lift goes up and lift goes down |");
        telemetry.addData("| Gamepad2 Right Bumper:  ", "Grabs cone                      |");
        telemetry.addData("| Gamepad2 Left Bumper:   ", "Release cone                    |");
        telemetry.addData("+-------------------------", "--------------------------------+");

        stevesRobot.setTurretMotorPower(0.55*gamepad2.right_stick_x);  //* moves the turret at 75% maximum power
        stevesRobot.setLiftMotorPower(-gamepad2.left_stick_y * 2.0);        //* moves the lift up and down

        if (gamepad2.left_bumper) {
            stevesRobot.setinTakeServoPosition(1.0);                                        // Have the intake mechanism deposit the cone
            telemetry.addData("Cone being deposited ", "press right bumper to intake next cone");
        }
        else if (gamepad2.right_bumper) {
            stevesRobot.setinTakeServoPosition(0.0);                                        // Have the intake grab a cone
            telemetry.addData("Cone grabbed ", "press left bumper to deposit cone");
        }
        telemetry.addData("BOT HEADING BOT HEADING",stevesRobot.getHeading());
        doDrive();

        telemetry.update();


    }


    public void doDrive() {
        //double autoHeading = RoboRaidersProperties.getHeading();
        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = stevesRobot.getHeading();

        double y = gamepad1.left_stick_y; // Remember, this is reversed!`
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double lTrigger = gamepad1.left_trigger;
        double rTrigger = gamepad1.right_trigger;

//        telemetry.addLine("MAKE SURE THE ARROWS ON MOTORS 1 AND 3 FACE THE DRIVER");
//        telemetry.addLine("Variables");
//        telemetry.addData("botHeading", String.valueOf(botHeading));
//        telemetry.addData("y", String.valueOf(y));
//        telemetry.addData("x", String.valueOf(x));
//        telemetry.addData("rx", String.valueOf(rx));
//        telemetry.addData("rotX", String.valueOf(rotX));
//        telemetry.addData("rotY", String.valueOf(rotY));
//        telemetry.addData("denominator", String.valueOf(denominator));
//        telemetry.addData("frontLeftPower", String.valueOf(frontLeftPower));
//        telemetry.addData("backLeftPower", String.valueOf(backLeftPower));
//        telemetry.addData("frontRightPower", String.valueOf(frontRightPower));
//        telemetry.addData("backRightPower", String.valueOf(backRightPower));
//        telemetry.addData("auto heading: ", RoboRaidersProperties.getHeading());

        if(lTrigger > 0.0){
            frontLeftPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            frontRightPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            backLeftPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
            backRightPower = (frontLeftPower*0.65) - (0.2 * lTrigger);
        }
        else if(rTrigger > 0.0){
            frontLeftPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            frontRightPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            backLeftPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
            backRightPower = (frontLeftPower*0.65) + (0.2 * lTrigger);
        }

//        if(gamepad2.right_bumper) {
//            stevesRobot.setinTakeServoPosition(0.0);
//        }
//        else if(gamepad2.left_bumper) {
//            stevesRobot.setinTakeServoPosition(1.0);
//        }


        stevesRobot.setDriveMotorPower(
                frontLeftPower*0.45,
                frontRightPower*0.45,
                backLeftPower*0.45,
                backRightPower*0.45);
        //               dtLogger);
    }
}
