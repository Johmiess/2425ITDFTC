package org.firstinspires.ftc.teamcode.Auto;

import android.app.slice.Slice;

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

    public static double iterationTime = 0;

    public static double speed = 0;

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

    public class VertSlide {
        private DcMotorEx leftThing, rightThing;
        private ElapsedTime time;


        public VertSlide(HardwareMap hardwareMap){leftThing = hardwareMap.get(DcMotorEx.class, "leftThing"); rightThing = hardwareMap.get(DcMotorEx.class, "rightThing");time = new ElapsedTime();}
        public void verticalSlides(double speed) {
            leftThing.setPower(-speed);
            rightThing.setPower(-speed);
        }
        public double vals(){
            return leftThing.getPower();
        }
        public class slidesUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                time.reset();
                iterationTime = .1;
                while (time.seconds()< iterationTime){
                    verticalSlides(1);
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
                while (time.seconds()< iterationTime ){
                   verticalSlides(.5);
                }
                leftThing.setPower(0);
                rightThing.setPower(0);
                return false;
            }
        }

        public Action slideUp() {return new VertSlide.slidesUp();}

        public Action slideDown() {return new VertSlide.slideDown();}
    }

    public class Arm{
        private CRServoImplEx leftAxon, rightAxon;
        private DcMotorEx encoder;
        private int position;
        private ElapsedTime time;

        public Arm(HardwareMap hardwareMap){
            leftAxon = hardwareMap.get(CRServoImplEx.class,"leftAxon");
            rightAxon = hardwareMap.get(CRServoImplEx.class, "rightAxon");
            encoder = hardwareMap.get(DcMotorEx.class,"intake");
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public class clockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftAxon.setPower(1);
                rightAxon.setPower(-1);
                return false;
            }
        }
        public class counterclockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                leftAxon.setPower(-1);
                rightAxon.setPower(1);
                return false;
            }
        }
        public class armUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                while (time.seconds() < .8) {
                    leftAxon.setPower(1);
                    rightAxon.setPower(1);
                }
                leftAxon.setPower(0);
                rightAxon.setPower(0);
                position = encoder.getCurrentPosition();
                while (position<=.5/*value for 180*/) {
                    leftAxon.setPower(-1);
                    rightAxon.setPower(1);
                }
                leftAxon.setPower(0);
                rightAxon.setPower(0);
                return false;
            }
        }
        public Action armUp(){return new Arm.armUp();}
    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -59.5, Math.toRadians(90)));
        VertSlide slide = new VertSlide(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
                .stopAndAdd(arm.armUp());

        TrajectoryActionBuilder score = drive.actionBuilder(new Pose2d(0,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(1,1));

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.openClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("x",slide.vals());
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