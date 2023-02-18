package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class roadrunnerAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-33, 63, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        Trajectory step1 = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0.5, () -> {
                    //code in here, this is where we put the code to lift the lift.

                })
                .splineToConstantHeading(new Vector2d(-36.5,52),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-36.5, 47), Math.toRadians(-90))
                .forward(32)
                .splineTo(new Vector2d(-40,10), Math.toRadians(-131))
                .build();
        Trajectory step2 = drive.trajectoryBuilder(step1.end())
                .lineTo(new Vector2d(-29.25,20))
                .build();
        Trajectory step3 = drive.trajectoryBuilder(step2.end())
                .splineTo(new Vector2d(-58, 13), Math.toRadians(-180))

                .build();
        Trajectory step4 = drive.trajectoryBuilder(step3.end())
                .forward(22)
                .build();
        Trajectory step5 = drive.trajectoryBuilder(step4.end())
                .back(10)
                .build();
//        Trajectory step6 = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(-28,14), Math.toRadians(60))
//                .build();
//        Trajectory step7 = drive.trajectoryBuilder(startPose)
//                .strafeTo(new Vector2d(-33,12))
//                .build();
        waitForStart();
        drive.followTrajectory(step1);
        drive.followTrajectory(step2);
        drive.followTrajectory(step3);
//        drive.turn(Math.toRadians(-39));
//        drive.followTrajectory(step3);
//        drive.followTrajectory(step5);






    }
}
