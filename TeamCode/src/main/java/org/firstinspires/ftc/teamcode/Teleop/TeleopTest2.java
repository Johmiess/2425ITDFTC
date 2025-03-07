package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Config.LiftUtil;
import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@TeleOp(name="TELEOP TEST 2")
public class TeleopTest2 extends LinearOpMode {
    public static double open = .3;
    public static double close = .55;
    public static double armUp = 0;
    public static double armDown = 0;
    public static double vertUp = 0;
    public static double vertDown = 0;

    @Override
    public void runOpMode() {
        robot robo = new robot(this);
        robo.init();
        while (opModeInInit()) {
            telemetry.addLine("starting");

            /*1000*/
            telemetry.update();

        }
        while (opModeIsActive()) {
            if (gamepad1.left_trigger>.2){
                robo.claw(open);
            } else if (gamepad1.left_bumper){
                robo.claw(close);
            }
            if (gamepad1.right_trigger>.2){
                robo.setPos(armDown);
            } else if (gamepad1.right_bumper){
                robo.setPos(armUp);
            }
            if (gamepad1.x) {
                robo.vertSlidesPIDup(LiftUtil.target);
            }
            else if (gamepad1.y){
                robo.vertSlidesPIDdown(LiftUtil.downTarget);
            } else {
                robo.horizontalSlides(0);
            }
            telemetry.addData("leftArm",robo.leftAxon.getPosition());
            telemetry.addData("rightArm",robo.rightAxon.getPosition());
            telemetry.addData("claw",robo.claw.getPosition());
            telemetry.addData("vert",robo.leftThing.getCurrentPosition());

            telemetry.update();

        }
    }
}