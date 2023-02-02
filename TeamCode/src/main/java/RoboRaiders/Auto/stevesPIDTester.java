package RoboRaiders.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import RoboRaiders.PID.PidUdpReceiver;

@Autonomous
//@Disabled

public class stevesPIDTester extends LinearOpMode {


        private PidUdpReceiver pidUdpReceiver;
        private double kP, kI, kD, degrees, direction;


        //----------------------------------------------------------------------------------------------
        // Main logic
        //----------------------------------------------------------------------------------------------

        @Override
        public void runOpMode() throws InterruptedException {

            // Create new instance of receiver
            pidUdpReceiver = new PidUdpReceiver();

            // Start listening
            pidUdpReceiver.beginListening();

            // initialize the robot
            //   robot.initialize(hardwareMap);

            // set the transmission interval to 50 milliseconds
            telemetry.setMsTransmissionInterval(50);

            // Wait for start to be pushed
            waitForStart();

            while (opModeIsActive()) {

                updatePIDCoefficients();

                telemetry.addLine("Steve's PID Tuner");
                telemetry.addData("kP", String.valueOf(kP));
                telemetry.addData("kI", String.valueOf(kI));
                telemetry.addData("kD", String.valueOf(kD));
                telemetry.addData("Degrees", String.valueOf(degrees));

                if (direction == 0.0) {telemetry.addLine("Turn Direction - Right");}
                else {telemetry.addLine("Turn Direction - Left");}
                telemetry.update();
            }

            pidUdpReceiver.shutdown();
        }

        public void updatePIDCoefficients() {

            kP = pidUdpReceiver.getP();
            kI = pidUdpReceiver.getI();
            kD = pidUdpReceiver.getD();
            degrees = pidUdpReceiver.getDegrees();
            direction = pidUdpReceiver.getDirection();
        }
}
