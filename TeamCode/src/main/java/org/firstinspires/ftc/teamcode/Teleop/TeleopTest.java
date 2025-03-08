package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name= "TEST: ARM")
public class TeleopTest extends LinearOpMode {
    /**
     * TESTING TELEOP PLAYGROUND:
     * CURRENTLY BEING USED FOR ARM AND CLAW TESTS
     */
//    robot robo;

    public ServoImplEx rightAxon, leftAxon;

    public static double rightPos = 1;
    public static double leftPos = 1;

    public static double rightPos2 = 0;
    public static double leftPos2 = 0;

    /**
     * // 0.05 init
     * // 0.25 post-scoring
     * // 0.45 90 degrees
     * // 0.85 flat
     * **/

    @Override
    public void runOpMode() {
//        robo = new robot(this);
//        robo.init();

        leftAxon = hardwareMap.get(ServoImplEx.class, "leftAxon");
        rightAxon = hardwareMap.get(ServoImplEx.class, "rightAxon");


        while (opModeInInit()) {
            telemetry.addLine("starting");

            /*1000*/
            telemetry.update();

        }

        while (opModeIsActive()) {

            if(gamepad1.dpad_up){
                rightAxon.setPosition(rightPos);
                leftAxon.setPosition(leftPos);
            } else if(gamepad1.dpad_down){
                rightAxon.setPosition(rightPos2);
                leftAxon.setPosition(leftPos2);
            } else if(gamepad1.dpad_right){
                rightAxon.setPosition(0.5);
                leftAxon.setPosition(0.5);
            }




//            telemetry.addData("RIGHTARM", robo.getRightArmEncoderPosition());
//            telemetry.addData("LEFTARM", robo.getLeftArmEncoderPosition());
            telemetry.addData("leftAxon",leftAxon.getPosition());
            telemetry.addData("RightAxon", rightAxon.getPosition());
            telemetry.update();

        }
    }
}
