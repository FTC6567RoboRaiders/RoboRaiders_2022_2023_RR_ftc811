
package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import static java.lang.Thread.sleep;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);



        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.26, 59.57, Math.toRadians(315.6882312138729), Math.toRadians(315.6882312138729), 11.46)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-33, 63, Math.toRadians(-90)))
                                .forward(48)
                                .turn(Math.toRadians(-45))
                                .back(5)
//                                .splineTo(new Vector2d(-35, 35), 0)
//                                .splineTo(new Vector2d(-33, 13.2), Math.toRadians(0))
//                                .turn(Math.toRadians(-135))
//                                .back(5)
//                                .forward(5)
//                                .turn(Math.toRadians(-45))
//                                .splineTo(new Vector2d(-58, 13), Math.toRadians(0))
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
