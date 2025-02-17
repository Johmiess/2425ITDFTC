package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name="full teleop")
public class TeleopPIDMain extends LinearOpMode {
    robot robo;

    double x, y, rx, lf, lb, rf, rb, denominator = 0;

    public static double power = 0.75;

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
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            lf = (y + x - rx) / denominator;
            lb = (y - x - rx) / denominator;
            rf = (y - x + rx) / denominator;
            rb = (y + x + rx) / denominator;

            if (gamepad1.right_trigger > 0.2) {
                robo.setMotorPowers(lf * 0.2, lb * 0.2, rf * 0.2, rb * 0.2);
            } else {
                robo.setMotorPowers(lf, lb, rf, rb);

            if (gamepad1.x) {
                robo.intake(0.75);
            }
            else if (gamepad1.y){
                robo.intake(-0.75);
            } else {
                robo.intake(0);
            }

            if (gamepad1.a) {
                robo.verticalSlides(0.75);
            } else if (gamepad1.b) {
                robo.verticalSlides(-0.75);
            } else {
                robo.verticalSlides(0);
            }

            if (gamepad2.dpad_up) {
                robo.clawSpinCounterClockWise(0.15);
//                    robo.armFoward(0.15);
            }
            else if (gamepad2.dpad_down) {
//                    robo.armBack(0.15);
                robo.clawSpinClockWise(0.15);
            } else if(gamepad2.dpad_right){
                robo.armBack(0.15);
//                    robo.clawSpinClockWise(0.15);
            } else if(gamepad2.dpad_left){
                robo.armFoward(0.15);
                //                    robo.clawSpinCounterClockWise(0.15);
            } else {
                robo.armBack(0.01);
            }

            if (gamepad2.right_bumper) {
                robo.claw(1);
            } else {
                robo.claw(0);
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