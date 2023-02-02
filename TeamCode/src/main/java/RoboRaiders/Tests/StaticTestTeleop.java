package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import RoboRaiders.Properties.RoboRaidersProperties;
import RoboRaiders.Robot.Hubbot;

//----------------------------------------------------------------------------------------------
// Static Test Teleop - tests the usage of Static variable in the RoboRaidersProperties class.
// This opmode will get the static variable.
//----------------------------------------------------------------------------------------------
@TeleOp(name="Static Test Teleop", group="Test Teleops")

public class StaticTestTeleop extends OpMode {

    public Hubbot dogMan = new Hubbot();


    @Override
    public void init() {

        // Initialize stevesRobot and tell user that the robot is initialized
        dogMan.initialize(hardwareMap);
        dogMan.resetIMU();

        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.addData("Last Heading: ", RoboRaidersProperties.lastHeading);

        /*
        telemetry.addData("Auto Heading: ", Hubbot.autoheading);
        Hubbot.autoheading = 1.0;
        telemetry.addData("Auto Heading: ", Hubbot.autoheading);
        */

        telemetry.update();
    }

    @Override
    public void loop() {

        telemetry.addData("Last Heading: ", RoboRaidersProperties.lastHeading);
        telemetry.addData("Integrated Z Axis: ", dogMan.getIntegratedZAxis());
        telemetry.update();


    }

}
