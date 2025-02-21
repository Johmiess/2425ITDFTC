package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name="TELEOP TEST 2")
public class TeleopTest2 extends LinearOpMode {
    robot robo;

    double x, y, rx, lf, lb, rf, rb, denominator = 0;

    public static double power = 0.75;
    public static double target = 0;

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

            if (gamepad1.x) {
                robo.vertSlidesPIDup(target);
            }


            telemetry.addData("LF", robo.leftFront.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("RB", robo.rightBack.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("JST LEFT", robo.getRightArmEncoderPosition());
            telemetry.addData("JST RIGHT", robo.getLeftArmEncoderPosition());

            telemetry.update();

        }
    }
}