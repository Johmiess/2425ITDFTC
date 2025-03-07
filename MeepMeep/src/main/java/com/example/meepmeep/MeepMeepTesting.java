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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -62.5, Math.toRadians(90)))
//                .strafeTo(new Vector2d(10,-24))
//                .setTangent(0)
//                .lineToX(60)
//                        .setTangent(Math.PI/2)
//                .lineToY(-56)
//                .strafeTo(new Vector2d(60,-56))
//                .waitSeconds(2)
//                .waitSeconds(.5)
//                .strafeTo(new Vector2d(5,-24))
//                .waitSeconds(.5)
                        .setTangent(Math.PI)
                        .lineToX(0)
                        .setTangent(Math.PI/2)
                        .lineToY(-30)
                        .lineToY(-50)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}