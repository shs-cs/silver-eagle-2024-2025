package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 18)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 72, -Math.PI / 2))
                .strafeTo(new Vector2d(0, 35))
                .strafeTo(new Vector2d(0, 55))
                .waitSeconds(1)
                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(-44, 0, Math.PI / 2), -Math.PI / 2)
                .strafeTo(new Vector2d(-44, 65))
                                //.strafeTo()
//                .lineToY(35.0)
//                .lineToYLinearHeading(55.0, Math.PI/2)
//                .waitSeconds(0.5)
//                .splineToLinearHeading(new Pose2d(-44, 0, -Math.PI/2), Math.PI/2)
                  .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}