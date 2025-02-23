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
    public static double right =.1;
    public static double low = 0;
    public static double high = 0;

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
            x = -gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            lf = (y + x - rx) / denominator;
            lb = (y - x - rx) / denominator;
            rf = (y - x + rx) / denominator;
            rb = (y + x + rx) / denominator;

            //bumper & trigger controls

            if (gamepad2.right_bumper) {
                robo.claw(0.435 );
            } else {
                robo.claw(0);
            }

            if (gamepad2.right_trigger > 0.2) {
                robo.setMotorPowers(lf * 0.2, lb * 0.2, rf * 0.2, rb * 0.2);
            } else {
                robo.setMotorPowers(lf, lb, rf, rb);
            }

            if(gamepad1.left_trigger > 0.2){
                robo.setIntake(1);
            } else if(gamepad1.left_bumper){
                robo.setIntake(-1);
            } else {
                robo.setIntake(0);
            }

            // x,y,a,b

            if (gamepad1.dpad_down) {

                robo.verticalSlides(1);
            }
            else if (gamepad1.dpad_up){

                robo.verticalSlides(-1);
            } else {
                robo.horizontalSlides(0);
            }

            if (gamepad1.a) {
                robo.horizontalSlides(0.75);
            } else if (gamepad1.b) {
                robo.horizontalSlides(-0.75);
            } else {
                robo.verticalSlides(0);
            }


            // dpad controls

            if (gamepad2.dpad_up) {
                robo.armFoward(0.5);
            } else if (gamepad2.dpad_down) {
                robo.armBack(0.5);
            } else if(gamepad2.dpad_right){
                robo.rotateClaw(.35);
            } else if(gamepad2.dpad_left){
                robo.rotateClaw(.92);
            } else {
                robo.armFoward(0);
            }

            telemetry.addData("LF", robo.leftFront.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("RB", robo.rightBack.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("JST LEFT", robo.getRightArmEncoderPosition());
            telemetry.addData("JST RIGHT", robo.getLeftArmEncoderPosition());
            telemetry.addData("leftpower", robo.leftAxon.getPower());
            telemetry.addData("rightpower", robo.rightAxon.getPower());

            telemetry.update();

            }
        }
    }
