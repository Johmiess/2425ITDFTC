package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name = "red_asc", group = "Autonomous")

public class red_asc extends LinearOpMode {
    public class Arm{
        private CRServoImplEx rightAxon, leftAxon;
        public Arm(HardwareMap hardwareMap){
            rightAxon = hardwareMap.get(CRServoImplEx.class, "rightAxon");
            leftAxon = hardwareMap.get(CRServoImplEx.class, "leftAxon");
        }
        public class up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightAxon.setPower((-.15));
                leftAxon.setPower((.15));
                return false;
            }
        }
        public class down implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightAxon.setPower((.15));
                leftAxon.setPower((-.15));
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds()<1){
                }
                return false;
            }
        }
        public Action up(){return new up();}
        public Action down(){return new down();}
    }
    public class Lift {
        private DcMotorEx vertShift;

        public Lift(HardwareMap hardwareMap) {
            vertShift = hardwareMap.get(DcMotorEx.class, "vertShift");
        }

        public class up implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                vertShift.setPower(0.15);
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() < .5) {
                }
                return false;
            }
        }
        public Action up() {return new Lift.up();}

    }
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-24, -60, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        TrajectoryActionBuilder p1 = drive.actionBuilder(initialPose)
                .setTangent(0)
                .lineToX(-40)
                .setTangent(-Math.PI/2)
                .lineToY(0)
//                .turn(Math.toRadians(-90))
                .setTangent(Math.PI)
                .lineToX(-20);

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action a1 = p1.build();

        Actions.runBlocking(
                new SequentialAction(
                    a1,
                    lift.up(),
                    arm.down()
                )
        );
    }
}