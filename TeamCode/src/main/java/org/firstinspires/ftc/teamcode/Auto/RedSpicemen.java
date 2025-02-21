package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RedSpicemen", group = "Autonomous")
public class RedSpicemen extends LinearOpMode {

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
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class Arm{
        private CRServoImplEx rightAxon, leftAxon;
        private ElapsedTime time;


        public Arm(HardwareMap hardwareMap){leftAxon = hardwareMap.get(CRServoImplEx.class, "leftAxon"); rightAxon = hardwareMap.get(CRServoImplEx.class, "rightAxon");time = new ElapsedTime();}
        public class ArmUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                time.reset();
                while (time.seconds()<.3){
                    rightAxon.setPower(1);
                    leftAxon.setPower(1);
                }
                rightAxon.setPower(0);
                leftAxon.setPower(0);
                return false;
            }
        }
        public Action armUp() {return new Arm.ArmUp();}
        public class ArmDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                time.reset();
                while (time.seconds()<.25){
                    rightAxon.setPower(-1);
                    leftAxon.setPower(-1);
                }
                rightAxon.setPower(0);
                leftAxon.setPower(0);
                return false;
            }
        }
        public Action armDown() {return new Arm.ArmDown();}

        public Action slidesUp() {return new Arm.ArmUp();}

    }

    public class VertSlide {
        private DcMotorEx leftThing, rightThing;
        private ElapsedTime time;

        public VertSlide(HardwareMap hardwareMap){leftThing = hardwareMap.get(DcMotorEx.class, "leftThing"); rightThing = hardwareMap.get(DcMotorEx.class, "rightThing");time = new ElapsedTime();}
        public void verticalSlides(double speed) {
            leftThing.setPower(-speed);
            rightThing.setPower(speed);
        }
        public class slidesUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                time.reset();
                while (time.seconds()<.25){
                    verticalSlides(0.5);
                }
                leftThing.setPower(0);
                rightThing.setPower(0);
                return false;
            }
        }

        public class slideDown implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                time.reset();
                while (time.seconds()<.25){
                   verticalSlides(.5);
                }
                leftThing.setPower(0.5);
                rightThing.setPower(0.5);
                return false;
            }
        }

        public Action slideUp() {return new VertSlide.slidesUp();}

        public Action slideDown() {return new VertSlide.slideDown();  }
    }


    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -59.5, Math.toRadians(90)));
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(51,-5))
                .setTangent(Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(62, -5),Math.PI/2)
                .lineToY(-50)
                .lineToY(-30)
                .splineTo(new Vector2d(69,-5),Math.PI/2)
                .setTangent(Math.PI/2)
                .lineToY(-50);
//                .lineToY(0);
//                .lineToY(-57)
//                .stopAndAdd(arm.armUp())
//                .stopAndAdd(arm.armUp())
//                .stopAndAdd(arm.armUp())
//                .stopAndAdd(claw.closeClaw())
//                .waitSeconds(.5)
//                .strafeTo(new Vector2d(0, -30))
//                .stopAndAdd(arm.armDown())
//                .stopAndAdd(arm.armDown())
//                .lineToY(-25);

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.openClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action traj = temp.build();

        Actions.runBlocking(
                new SequentialAction(
                        traj
                )
        );
    }
}