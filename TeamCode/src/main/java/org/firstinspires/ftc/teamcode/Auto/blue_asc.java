//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//@Autonomous(name = "blue_asc", group = "Autonomous")
//public class blue_asc extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Pose2d initialPose = new Pose2d(24, 60, Math.toRadians(-90));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
//
//        TrajectoryActionBuilder p1 = drive.actionBuilder(initialPose)
//                .setTangent(Math.PI)
//                .lineToX(40)
//                .setTangent(-Math.PI/2)
//                .lineToY(0)
//                .turn(Math.toRadians(-90))
//                .setTangent(Math.PI)
//                .lineToX(20);
//
//        telemetry.update();
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        Action a1 = p1.build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        a1
//                )
//        );
//    }
//}