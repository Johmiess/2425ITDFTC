package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot;

@Config
@TeleOp(name="teleport operations")
public class TeleopPIDMain extends LinearOpMode {
    robot robo;

    public static double intakeSpool = 0.4;
    public static double scoringSpool = 1;

    @Override
    public void runOpMode() {
        robo = new robot(this);
        robo.init();

        while (opModeInInit()) {
            telemetry.addLine("starting");

            /*1000*/
            telemetry.update();

        }

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lf = (y - x + rx) / denominator;
            double lb = (y + x + rx) / denominator;
            double rf = (y - x - rx) / denominator;
            double rb = (y + x - rx) / denominator;

            if (gamepad2.right_trigger > 0.2) {
                robo.setMotorPowers(lf * 0.2, lb * 0.2, rf * 0.2, rb * 0.2);
            } else {
                robo.setMotorPowers(lf, lb, rf, rb);

                if (gamepad1.y) {
                    robo.Hori(-.75);
                    robo.Vert(.75);

                } else if (gamepad1.a) {
                    robo.Hori(.75);
                    robo.Vert(-.75);

                } else {
                    robo.Hori(0);
                    robo.Vert(0);

                }

                if (gamepad1.right_trigger > 0.1) {
                    robo.intake(-0.7);
                } else if (gamepad1.right_trigger == 0) {
                    robo.intake(0);
                }

                if (gamepad1.x) {
                    robo.intake(0.7);
                }
//                if (gamepad2.dpad_up) {
//                    robo.arm(0.85);
//                } else if (gamepad2.dpad_down) {
//                    robo.arm(0);
//                }
//                else if (gamepad2.dpad_right) {
//                    robo.arm(0.6);
//                }


                if (gamepad2.y) {
                    robo.Hori(.75);
                    robo.Vert(-.75);
                } else if (gamepad2.a) {
                    robo.Hori(-.75);
                    robo.Vert(.75);
                } else {
                    robo.Hori(0);
                    robo.Vert(0);
                }

                if (gamepad2.dpad_up) {
                    robo.armFoward(0.15);
                }
                else if (gamepad2.dpad_down) {
                    robo.armBack(0.15);
                } else if(gamepad2.dpad_right){
                    robo.clawSpinClockWise(0.5);
                } else if(gamepad2.dpad_left){
                    robo.clawSpinCounterClockWise(0.5);
                } else {
                    // set power to zero to keep still
                    robo.armFoward(0);
                }

                if (gamepad2.right_bumper) {
                    robo.claw(0.3);
                } else {
                    robo.claw(0.08);
                }


                telemetry.addData("LF", robo.leftFront.getPower());
                telemetry.addData("RF", robo.rightFront.getPower());
                telemetry.addData("RB", robo.rightBack.getPower());
                telemetry.addData("RF", robo.rightFront.getPower());
                telemetry.update();

            }
        }
    }
}