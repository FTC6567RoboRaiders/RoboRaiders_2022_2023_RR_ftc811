package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import RoboRaiders.Robot.RR_ChuckBot_LIT;
import RoboRaiders.Teleop.TestBotTeleop;

@Autonomous
public class roadrunnerAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RR_ChuckBot_LIT Chuckbot = new RR_ChuckBot_LIT();
        Chuckbot.initialize(hardwareMap);

        Pose2d startPose = new Pose2d(-33, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

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
                .lineTo(new Vector2d(-29.0,23.25))
                .addTemporalMarker(3.25, () -> {
                    //code in here, this is where we put the code to lift the lift.
                    while (Math.abs(Chuckbot.getTurretEncoderCounts() - (-192.0)) > 5.0) {
                        telemetry.addData("turret encoder count: ", String.valueOf(Math.abs(Chuckbot.getTurretEncoderCounts() - (-192.0))));
                        telemetry.update();
                    }
                    Chuckbot.setLiftPositionMidDeposit();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(1500.0);
                })
                .addTemporalMarker(4.0, () -> {
                    Chuckbot.setinTakeServoPosition(1.0);
                })
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .splineTo(new Vector2d(-58, 15.25), Math.toRadians(-180)) // Wade changed the cords, the were (-58, 13)
                .forward(4.7) // this was in step 4 originally
                .addTemporalMarker(.05, () -> {
                    Chuckbot.setTurretPositionHome();
                    Chuckbot.turretRunWithEncodersSTP();
                    Chuckbot.setTurretMotorVelocity(200);
                })
                .addTemporalMarker(.5, () -> {
                    Chuckbot.setLiftPositionStack5Collect();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
//                .addTemporalMarker(.6, () -> {
//                    Chuckbot.setinTakeServoPosition(0.0);
//                })
                .build();
        Trajectory step4 = drive.trajectoryBuilder(step3.end())
                .addTemporalMarker(.05, () -> {
                    Chuckbot.setLiftPositionHigh();
                    Chuckbot.setLiftRunWithEncodersSTP();
                    Chuckbot.setLiftMotorVelocity(3000.0);
                })
                .back(3.0)
                .splineTo(new Vector2d(-29, 5), -222)

                .build();

//        Trajectory step5 = drive.trajectoryBuilder(step4.end())
//                .addTemporalMarker(.05, () -> {
//                    Chuckbot.setLiftPositionStack5Collect();
//                    Chuckbot.setLiftRunWithEncodersSTP();
//                    Chuckbot.setLiftMotorVelocity(3000.)
//                    Chuckbot.setinTakeServoPosition(0.0);
//                    Chuckbot.setLiftPositionHigh();
//                })
//                .build();

        waitForStart();
        drive.followTrajectory(step1);
        drive.followTrajectory(step2);
        sleep(1000);
        drive.followTrajectory(step3); // added by wade
        sleep(500);
        Chuckbot.setinTakeServoPosition(0.0);
        sleep(1000);
        drive.followTrajectory(step4);
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
}
