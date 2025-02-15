package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.robot;

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
            double y = gamepad2.left_stick_y * -1;
            double x = gamepad2.left_stick_x * 1.1;
            double rx = gamepad2.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double lf = (y - x + rx) / denominator;
            double lb = (y + x + rx) / denominator;
            double rf = (y + x - rx) / denominator;
            double rb = (y - x - rx) / denominator;

            if (gamepad2.right_trigger > 0.2) {
                robo.setMotorPowers(lf * 0.2, lb * 0.2, rf * 0.2, rb * 0.2);
            } else {
                robo.setMotorPowers(lf, lb, rf, rb);

            if (gamepad2.right_trigger > 0.1) {
                robo.intake(-0.7);
            }
            else if (gamepad2.right_trigger == 0) {
                robo.intake(0);
            }

            if (gamepad2.x) {
                robo.intake(0.7);
            }

            if (gamepad2.y) {
                robo.verticalSlidesUP(.35);
            } else if (gamepad2.a) {
                robo.verticalSlidesDown(.35);
            } else {
                robo.verticalSlidesDown(0);
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