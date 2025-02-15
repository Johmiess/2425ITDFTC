package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name= "teleop")
public class TeleopTest extends LinearOpMode {
    /**
     * TESTING TELEOP PLAYGROUND:
     * CURRENTLY BEING USED FOR ARM AND CLAW TESTS
     */
//    robot robo;

    private DcMotor horiShift = null;
    private DcMotor intake = null;
    public static double intakeSpool = 0.4;
    public static double scoringSpool = 1;

    @Override
    public void runOpMode() {
//        robo = new robot(this);
//        robo.init();

        horiShift = hardwareMap.get(DcMotorEx.class, "horiShift");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        horiShift.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        while (opModeInInit()) {
            telemetry.addLine("starting");

            /*1000*/
            telemetry.update();

        }

        while (opModeIsActive()) {
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            robo.leftFront.setPower(y + x + rx);
//            robo.leftBack.setPower(y - x + rx);
//            robo.rightFront.setPower(y - x - rx);
//            robo.rightBack.setPower(y + x - rx);

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


            if (gamepad1.dpad_up) {
                intake.setPower(0.5);
                horiShift.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                horiShift.setPower(-0.5);
                intake.setPower(-0.5);
            } else if (gamepad1.dpad_right) {
                intake.setPower(0.5);
                horiShift.setPower(-0.5);
            } else if (gamepad1.dpad_left) {
                intake.setPower(-0.5);
                horiShift.setPower(0.5);
            }

//            telemetry.addData("RIGHTARM", robo.getRightArmEncoderPosition());
//            telemetry.addData("LEFTARM", robo.getLeftArmEncoderPosition());
            telemetry.addData("HORISHIFT", intake.getPower());
            telemetry.addData("INTAKE", horiShift.getPower());
            telemetry.update();

        }
    }
}
