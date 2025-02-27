package com.example.meepmeep;

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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(24, -59.5, Math.toRadians(90)))
                .strafeTo(new Vector2d(10,-27))
                .waitSeconds(.5)
                .waitSeconds(.5)
                .setTangent(-Math.PI/2)
                .strafeTo(new Vector2d(24,-59.5))
                .setTangent(-Math.PI/2)
                .lineToY(-58)
                .splineTo(new Vector2d(52, 0),Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(62, -5),Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(71,-5),Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                //grab spec
//                .strafeTo(new Vector2d(65,-54))
                .waitSeconds(1)
                .waitSeconds(.5)
                .strafeTo(new Vector2d(65,-54))
                .waitSeconds(1)
                .strafeTo(new Vector2d(2,-27))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}