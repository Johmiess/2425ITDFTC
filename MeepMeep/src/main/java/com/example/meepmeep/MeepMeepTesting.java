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
                .splineTo(new Vector2d(0, -28.5),Math.PI/2)
                .lineToY(-50)
                .setTangent(Math.PI)
                .lineToX(30)
                .setTangent(Math.PI/2)
                .lineToY(-45)
                .splineTo(new Vector2d(52, -2),Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(60, -5),Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(66,-5),Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .strafeTo(new Vector2d(47,-62))


//                .stopAndAdd(arm.armPickUp())
                .waitSeconds(1)
//                .stopAndAdd(claw.closeClaw())
                .waitSeconds(1)
                //untested cause slides died
//                .stopAndAdd(arm.armPreScoring())
                .strafeTo(new Vector2d(0,-30))
//                .stopAndAdd(slide.slideUp())
//                .stopAndAdd(arm.armPostScoring())
                .waitSeconds(.5)
//                .stopAndAdd(claw.openClaw())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}