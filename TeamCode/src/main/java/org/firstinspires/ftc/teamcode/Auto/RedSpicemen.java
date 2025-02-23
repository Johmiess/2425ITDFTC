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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RedSpicemen", group = "Autonomous")
public class RedSpicemen extends LinearOpMode {

    public static double iterationTime = 0;

    public static double clawiterationTime = .355;

    public static double initToScoringTime = 0.3;
    public static double ScoringToPickUpTime = 0.8;
    public static double pickUpToScoringTime = 0.8;
    public static double clawClosedPos = 0.47;

    public static double clawOpenPos = 0;




    public static double speed = 0;

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
        private ServoImplEx rotate;
        private ElapsedTime time;

        public Arm(HardwareMap hardwareMap){
            leftAxon = hardwareMap.get(CRServoImplEx.class,"leftAxon");
            rightAxon = hardwareMap.get(CRServoImplEx.class, "rightAxon");
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

        public class armInitToScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                while (time.seconds() < initToScoringTime) {
                    leftAxon.setPower(1);
                    rightAxon.setPower(1);
                }
                leftAxon.setPower(0);
                rightAxon.setPower(0);
                return false;
            }
        }

        public class armPickUpToScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                while (time.seconds() < pickUpToScoringTime) {
                    leftAxon.setPower(1);
                    rightAxon.setPower(1);
                }
                leftAxon.setPower(0);
                rightAxon.setPower(0);
                return false;
            }
        }
        public class armScoringToPickUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                time.reset();
                while (time.seconds() < ScoringToPickUpTime) {
                    leftAxon.setPower(-1);
                    rightAxon.setPower(-1);
                }
                leftAxon.setPower(0);
                rightAxon.setPower(0);
                return false;
            }
        }
        public Action armInitToScoring(){return new armInitToScoring();}

        public Action armPickUpToScoring(){return new armPickUpToScoring();}
        public Action armScoringToPickUp(){return new armScoringToPickUp();}
        public Action clockwise(){return new Arm.clockwise();}
        public Action counterclockwise(){return new Arm.counterclockwise();}

    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -59.5, Math.toRadians(90)));
        VertSlide slide = new VertSlide(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
//                .splineTo(new Vector2d(52, -5),Math.PI/2)
//                .lineToY(-50)
//                .lineToY(-30)
//                .splineTo(new Vector2d(62, -5),Math.PI/2)
//                .lineToY(-50)
//                .lineToY(-30)
//                .splineTo(new Vector2d(71,-5),Math.PI/2)
//                .setTangent(Math.PI/2)
//                .lineToY(-50)
//                //grab spec
//                .afterDisp(50, slide.slideUp() )
////                .afterDisp(60, arm.arm() )
//                .strafeTo(new Vector2d(10,-30))
////                .stopAndAdd(slide.slideUp())
//                .waitSeconds(.5)
//                .setTangent(-Math.PI/2)
//                .afterDisp(50, slide.slideDown() )
//                .strafeTo(new Vector2d(65,-55))
//                .waitSeconds(.5)
//                .afterDisp(50, slide.slideUp() )
//                .strafeTo(new Vector2d(5,-30))
//                .waitSeconds(.5)
//                .setTangent(-Math.PI/2)
////                .stopAndAdd(slide.slideDown())
//                .afterDisp(50, slide.slideDown() )
//                .strafeTo(new Vector2d(65,-55))
//                .waitSeconds(.5)
//                .afterDisp(50, slide.slideUp() )
//                .strafeTo(new Vector2d(0,-30))
//                .stopAndAdd(arm.armUp());
                .stopAndAdd(arm.armInitToScoring())
                .stopAndAdd(arm.armScoringToPickUp())
                .stopAndAdd(arm.armPickUpToScoring());



        TrajectoryActionBuilder score = drive.actionBuilder(new Pose2d(0,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(1,1))
                .stopAndAdd(arm.counterclockwise())
                .stopAndAdd(arm.clockwise())
                .lineToY(-30);

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());
        Actions.runBlocking(arm.clockwise());



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