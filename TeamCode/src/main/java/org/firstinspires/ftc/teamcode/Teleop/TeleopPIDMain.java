package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Config.robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@TeleOp(name="full teleop")
public class TeleopPIDMain extends LinearOpMode {
    robot robo;

    double x, y, rx, lf, lb, rf, rb, denominator = 0;

    public static double power = 0.75;
    public static double right =0.86;
    public static double left = .23;
    public static double low = 0;
    public static double high = 0;

    public static double clawOpen = .3;
    public static double clawClosed = .57;

    public static long CYCLE_MS = 50;
    public OpenCvCamera cam;

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

            if (gamepad2.x) {
                robo.claw(clawClosed );
            } else {
                robo.claw(clawOpen);
            }

            if (gamepad2.right_trigger > 0.2) {
                robo.setMotorPowers(lf * 0.2, lb * 0.2, rf * 0.2, rb * 0.2);
            } else {
                robo.setMotorPowers(lf, lb, rf, rb);
            }


            // x,y,a,b

            if (gamepad1.x) {

                robo.verticalSlides(1);
            }
            else if (gamepad1.y){

                robo.verticalSlides (-1);
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

            if (gamepad2.right_bumper){
                robo.intakeIn();
            } else if (gamepad2.right_trigger>.2){
                robo.intakeOut();
            } else {
                robo.intake.setPower(0);
            }

            if (gamepad2.left_bumper){
                robo.intakeUp();
            } else if (gamepad2.left_trigger>.2){
                robo.intakeDown();
            }



            // dpad controls

            if (gamepad2.dpad_down) {
                robo.armFoward();
            } else if (gamepad2.dpad_up) {
                robo.armBack();
            } else if(gamepad2.dpad_right){
                robo.rotateClaw(right);
            } else if(gamepad2.dpad_left){
                robo.rotateClaw(left);
            }

            telemetry.addData("LF", robo.leftFront.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("RB", robo.rightBack.getPower());
            telemetry.addData("RF", robo.rightFront.getPower());
            telemetry.addData("ra", robo.rightAxon.getPosition());
            telemetry.addData("la", robo.leftAxon.getPosition());
            telemetry.addData("vert slides",robo.leftThing.getCurrentPosition());

            telemetry.update();

            }
        }
    }
