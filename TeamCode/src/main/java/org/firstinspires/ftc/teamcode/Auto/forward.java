package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.robot;

@Config
@Autonomous(name="forward")
public class forward extends LinearOpMode {
    robot robo;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robo = new robot(this);
        robo.init();

        while (opModeInInit()) {
            telemetry.addLine("starting");

            /*1000*/
            telemetry.update();
            runtime.reset();
        }

        while (opModeIsActive() && (runtime.seconds() < 2)) {
            robo.setMotorPowers(1);
        }
    }
}
