package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name="TELEOP TEST 2")
public class TeleopTest2 extends LinearOpMode {
    public ServoImplEx leftIntake, rightIntake;

    double x, y, rx, lf, lb, rf, rb, denominator = 0;

    public static double power = 0.75;
    public static double target = 0;

    @Override
    public void runOpMode() {

        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
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
                leftIntake.setPosition(target);
            }
            if (gamepad1.a) {
                leftIntake.setPosition(target);
                rightIntake.setPosition(target);
            }
            if (gamepad1.y) {
                rightIntake.setPosition(target);
            }

            telemetry.addData("left",leftIntake.getPosition());
            telemetry.addData("right",rightIntake.getPosition());

            telemetry.update();

        }
    }
}