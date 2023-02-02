package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.Properties.RoboRaidersProperties;
import RoboRaiders.Robot.Hubbot;

@Autonomous(name="Static Test Auto", group="Test Autos")

public class StaticTestAuto extends LinearOpMode {


    //----------------------------------------------------------------------------------------------
    // Static Test Auto - tests the usage of Static variable in the RoboRaidersProperties class.
    // This opmode will set the static variable.
    //----------------------------------------------------------------------------------------------

    @Override

    public void runOpMode() throws InterruptedException {

        Hubbot dogMan = new Hubbot();
        dogMan.initialize(hardwareMap);
        dogMan.resetIMU();

        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.addData("Initial Heading", dogMan.getIntegratedZAxis());
        telemetry.update();

        // Wait for start to be pushed
        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("lastHeading: ", RoboRaidersProperties.lastHeading);
            //        if (Hubbot.autoheading == 0.0) {
            RoboRaidersProperties.lastHeading = dogMan.getIntegratedZAxis();
            //          RoboRaidersProperties.setHeading(dogMan.getIntegratedZAxis());  // Not sure why the setter and getters no workie
            //        }
            telemetry.addData("Integrated Z Axis: ", dogMan.getIntegratedZAxis());
            telemetry.addData("lastHeading: ", RoboRaidersProperties.lastHeading);

    /*
             if (isStopRequested()) {
                telemetry.addData("Stop Requested...", true);
               telemetry.addData("autoheading: ",Hubbot.autoheading);
               Hubbot.autoheading = dogMan.getIntegratedZAxis();telemetry.update();
               telemetry.addData("autoheading: ",Hubbot.autoheading);

            }
            telemetry.update();

     */
            telemetry.update();

        }

        /*

        RoboRaidersProperties.setHeading(dogMan.getIntegratedZAxis());
        Hubbot.autoheading = dogMan.getIntegratedZAxis();

         */

    }

}
