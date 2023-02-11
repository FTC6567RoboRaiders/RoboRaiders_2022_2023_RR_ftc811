package RoboRaiders.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class roadrunnerAuto extends LinearOpMode {

    Pose2d startPose = new Pose2d();
    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Trajectory strafeLeftTraj = drive.trajectoryBuilder(startPose)
//                .strafeRight(3)
//                .build();
//        Trajectory goForwardTraj= drive.trajectoryBuilder(startPose)
//                .forward(39)
//                .build();
        waitForStart();
//        drive.followTrajectory(strafeLeftTraj);
//        drive.followTrajectory(goForwardTraj);



    }
}
