
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static java.lang.Thread.sleep;

public class MoreMeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.26, 59.57, Math.toRadians(315.6882312138729), Math.toRadians(315.6882312138729), 11.46)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(33, -65, Math.toRadians(90)))
//                                .forward(48)
//                                .turn(Math.toRadians(-45))
//                                .back(5)
                                        .splineToConstantHeading(new Vector2d(36.5,-52),Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(36.5, -47), Math.toRadians(90))
                                        .forward(22)
                                        .splineTo(new Vector2d(40,-10), Math.toRadians(35))
                                        .lineTo(new Vector2d(29.0,-23.25))
//                                        .lineTo(new Vector2d(40,-10))
//                                        .turn(Math.toRadians(-35))
                                        .splineTo(new Vector2d(54, -12), Math.toRadians(-165))//Wade changed the cords, the were (-58, 13)
                                        .forward(4.7) //wade changed was 22
//                                        .back(3)
//                                        .splineTo(new Vector2d(29, -5), 45)
//                                .back(10)
//                                .splineTo(new Vector2d(-28,14), Math.toRadians(60))
//                                .strafeTo(new Vector2d(-33,12))
//                                .turn(Math.toRadians(-60))
//                                .forward(26.5)
//                                .back(10)
//                                .splineTo(new Vector2d(-28,14), Math.toRadians(60))
//                                .strafeTo(new Vector2d(-33,12))
//                                .turn(Math.toRadians(-60))
//                                .forward(26.5)
//                                .back(10)
//                                .splineTo(new Vector2d(-28,14), Math.toRadians(60))

//                                .splineToConstantHeading(new Vector2d(-34, 33), -90)
//                                .forward(5)
//                                .splineTo(new Vector2d(-33, 13.2), Math.toRadians(0))
//                                .turn(Math.toRadians(-135))
//                                .back(5)
//                                .splineTo(new Vector2d(-41, 10), Math.toRadians(180))
//                                .splineToConstantHeading(new Vector2d(-58, 13), Math.toRadians(0))
                                        .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setTheme(new ColorSchemeRedLight())
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
