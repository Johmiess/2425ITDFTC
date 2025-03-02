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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RedSpicemenPlus1", group = "Autonomous")
public class RedSpicemenPlus1 extends LinearOpMode {


    public static double iterationTime = .11;

    public static double clawiterationTime = .355;

    public static double initToScoringTime = 0.3;
    public static double ScoringToPickUpTime = 0.8;
    public static double pickUpToScoringTime = 0.8;
    public static double clawClosedPos = 0.4;

    public static double clawOpenPos = .1;

    public static double speed = 0;

    /**
     * // 0.05 init
     * // 0.25 post-scoring
     * // 0.45 90 degrees
     * // 0.85 flat
     * **/


    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawClosedPos);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawOpenPos);
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
                   verticalSlides(-.5);
                }
                leftThing.setPower(0);
                rightThing.setPower(0);
                return false;
            }
        }

        public Action slideUp() {return new slidesUp();}

        public Action slideDown() {return new slideDown();}
    }

    public class Arm{
        private ServoImplEx leftAxon, rightAxon;
        private ServoImplEx rotate;
        private ElapsedTime time;

        public Arm(HardwareMap hardwareMap){
            leftAxon = hardwareMap.get(ServoImplEx.class,"leftAxon");
            rightAxon = hardwareMap.get(ServoImplEx.class, "rightAxon");
            rotate = hardwareMap.get(ServoImplEx.class,"rotate");
        }
        public class clockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rotate.setPosition(.92);
                return false;
            }
        }
        public class counterclockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rotate.setPosition(.35);
                return false;
            }
        }

        public class armInit implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(.95);
                rightAxon.setPosition(.95);
                return false;
            }
        }

        public class armPostScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(0.20 );
                rightAxon.setPosition(0.20 );
                return false;
            }
        }
        public class armPreScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(0.5);
                rightAxon.setPosition(0.5);
                return false;
            }
        }

        public class armPickUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(0.9);
                rightAxon.setPosition(0.9);
                return false;
            }
        }
        public Action armInit(){return new armInit();}

        public Action armPickUp(){return new armPickUp();}
        public Action armPreScoring(){return new armPreScoring();}

        public Action armPostScoring(){return new armPostScoring();}

        public Action clockwise(){return new clockwise();}
        public Action counterclockwise(){return new counterclockwise();}

    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -56, Math.toRadians(90)));
        VertSlide slide = new VertSlide(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
                .stopAndAdd(arm.armPreScoring())
                .strafeTo(new Vector2d(10,-24))
                .stopAndAdd(slide.slideUp())
                .stopAndAdd(arm.counterclockwise())
                .waitSeconds(1)
                .stopAndAdd(arm.armPostScoring())
                .waitSeconds(1)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .stopAndAdd(arm.armPickUp())
                .waitSeconds(.5)
                .setTangent(-Math.PI/2)
                .afterDisp(50, slide.slideDown())
                .lineToX(60)
                .lineToY(-56)
                .strafeTo(new Vector2d(60,-56))
                .waitSeconds(2)
                .stopAndAdd(claw.closeClaw())
                .waitSeconds(.5)
                .stopAndAdd(arm.armPreScoring())
                .stopAndAdd(arm.clockwise())
                .afterDisp(50, slide.slideUp())
                .strafeTo(new Vector2d(5,-24))
                .stopAndAdd(arm.armPostScoring())
                .waitSeconds(.5)
                .stopAndAdd(claw.openClaw());






        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(arm.clockwise());
        Actions.runBlocking(arm.armInit());
        ElapsedTime time = new ElapsedTime();
        while (time.seconds()<2){
            telemetry.addData("doing","nothing");
        }
        Actions.runBlocking(claw.closeClaw());




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