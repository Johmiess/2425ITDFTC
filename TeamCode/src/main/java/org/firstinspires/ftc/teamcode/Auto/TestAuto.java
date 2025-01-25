package org.firstinspires.ftc.teamcode.Auto;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Mat;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Intake{
        private DcMotorEx intake;
        public Intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
        }
        public void in(){
            intake.setPower(1);
        }
        public void out(){
            intake.setPower(-1);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(24, -70, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder p1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-40))
                .setTangent(-Math.PI/2)
                .lineToY(-30);
        TrajectoryActionBuilder p2 = p1.endTrajectory().fresh()
                .lineToY(-40)
                .setTangent(Math.PI)
                .lineToX(48);
        TrajectoryActionBuilder p3 = p2.endTrajectory().fresh()
                .setTangent(Math.PI/2)
                .lineToY(-30);
        TrajectoryActionBuilder p4 =p3.endTrajectory().fresh()
                .lineToY(-60);
        TrajectoryActionBuilder p5 = p4.endTrajectory().fresh()
                .lineToY(-40)
                .waitSeconds(1)
                .lineToY(-60);
        TrajectoryActionBuilder p6 = p5.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -40))
                .setTangent(Math.PI/2)
                .lineToY(-30);
        TrajectoryActionBuilder p7 = p6.endTrajectory().fresh()
                .lineToY(-40)
                .strafeTo(new Vector2d(60,-60));


        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());



        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action a1 = p1.build();
        Action a2 = p2.build();
        Action a3 = p3.build();
        Action a4 = p4.build();
        Action a5 = p5.build();
        Action a6 = p6.build();
        Action a7 = p7.build();

        Actions.runBlocking(
                new SequentialAction(
                    a1,
                    claw.openClaw(),
                    a2,
                    a3,
                    a4,
                    a5,
                    a6,
                    a7
                )
        );
    }
}