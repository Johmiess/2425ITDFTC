package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="tele-test")
public class TeleopTest extends LinearOpMode {
    /**
     * TESTING TELEOP PLAYGROUND:
     * CURRENTLY BEING USED FOR ARM AND CLAW TESTS
     */
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
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            robo.leftFront.setPower(y + x + rx);
            robo.leftBack.setPower(y - x + rx);
            robo.rightFront.setPower(y - x - rx);
            robo.rightBack.setPower(y + x - rx);

//
//            if (gamepad1.y) {
//                robo.armFoward(0.15);
//            }
//            else if (gamepad1.a) {
//                robo.armBack(0.15);
//            } else if(gamepad1.b){
//                robo.clawSpinClockWise(0.5);
//            } else if(gamepad1.x){
//                robo.clawSpinCounterClockWise(0.5);
//            } else {
//                // set power to zero to keep still
//                robo.armFoward(0);
//            }
//
//            telemetry.addData("RIGHTARM", robo.getRightArmEncoderPosition());
//            telemetry.addData("LEFTARM", robo.getLeftArmEncoderPosition());
//            telemetry.update();
        }
    }
}
