//package org.firstinspires.ftc.teamcode.Auto;
//
//import androidx.annotation.NonNull;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Config
//@Autonomous(name = "BlueSpicemen", group = "Autonomous")
//public class BlueSpicemen extends LinearOpMode {
//
//
//    public static double iterationTime = .13;
//
//    public static double clawiterationTime = .355;
//
//    public static double initToScoringTime = 0.3;
//    public static double ScoringToPickUpTime = 0.8;
//    public static double pickUpToScoringTime = 0.8;
//    public static double clawClosedPos = 0.35;
//
//    public static double clawOpenPos = 0;
//
//    public static double speed = 0;
//
//    /**
//     * // 0.05 init
//     * // 0.25 post-scoring
//     * // 0.45 90 degrees
//     * // 0.85 flat
//     * **/
//
//
//    public class Claw {
//        private Servo claw;
//
//        public Claw(HardwareMap hardwareMap) {
//            claw = hardwareMap.get(Servo.class, "claw");
//        }
//
//        public class CloseClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(clawClosedPos);
//                return false;
//            }
//        }
//        public Action closeClaw() {
//            return new Claw.CloseClaw();
//        }
//
//        public class OpenClaw implements Action {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                claw.setPosition(clawOpenPos);
//                return false;
//            }
//        }
//        public Action openClaw() {
//            return new Claw.OpenClaw();
//        }
//    }
//    public class VertSlide {
//        private DcMotorEx leftThing, rightThing;
//        private ElapsedTime time;
//
//
//        public VertSlide(HardwareMap hardwareMap){leftThing = hardwareMap.get(DcMotorEx.class, "leftThing"); rightThing = hardwareMap.get(DcMotorEx.class, "rightThing");time = new ElapsedTime();}
//        public void verticalSlides(double speed) {
//            leftThing.setPower(-speed);
//            rightThing.setPower(-speed);
//        }
//        public double vals(){
//            return leftThing.getPower();
//        }
//        public class slidesUp implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                time.reset();
//                while (time.seconds()< iterationTime){
//                    verticalSlides(1);
//                }
//                leftThing.setPower(0);
//                rightThing.setPower(0);
//                return false;
//            }
//        }
//
//        public class slideDown implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet){
//                time.reset();
//                while (time.seconds()< iterationTime ){
//                    verticalSlides(-.5);
//                }
//                leftThing.setPower(0);
//                rightThing.setPower(0);
//                return false;
//            }
//        }
//
//        public Action slideUp() {return new VertSlide.slidesUp();}
//
//        public Action slideDown() {return new VertSlide.slideDown();}
//    }
//
//    public class Arm{
//        private ServoImplEx leftAxon, rightAxon;
//        private ServoImplEx rotate;
//        private ElapsedTime time;
//
//        public Arm(HardwareMap hardwareMap){
//            leftAxon = hardwareMap.get(ServoImplEx.class,"leftAxon");
//            rightAxon = hardwareMap.get(ServoImplEx.class, "rightAxon");
//            rotate = hardwareMap.get(ServoImplEx.class,"rotate");
//        }
//        public class clockwise implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                rotate.setPosition(.92);
//                return false;
//            }
//        }
//        public class counterclockwise implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                rotate.setPosition(.35);
//                return false;
//            }
//        }
//
//        public class armInit implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftAxon.setPosition(0.00);
//                rightAxon.setPosition(0.00);
//                return false;
//            }
//        }
//
//        public class armPostScoring implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftAxon.setPosition(0.20 );
//                rightAxon.setPosition(0.20 );
//                return false;
//            }
//        }
//        public class armPreScoring implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftAxon.setPosition(0.5);
//                rightAxon.setPosition(0.5);
//                return false;
//            }
//        }
//
//        public class armPickUp implements Action{
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                leftAxon.setPosition(0.9);
//                rightAxon.setPosition(0.9);
//                return false;
//            }
//        }
//        public Action armInit(){return new armInit();}
//
//        public Action armPickUp(){return new armPickUp();}
//        public Action armPreScoring(){return new armPreScoring();}
//
//        public Action armPostScoring(){return new armPostScoring();}
//
//        public Action clockwise(){return new Arm.clockwise();}
//        public Action counterclockwise(){return new Arm.counterclockwise();}
//
//    }
//
//
//
//
//    @Override
//    public void runOpMode() {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(24, -59.5, Math.toRadians(-90)));
//        VertSlide slide = new VertSlide(hardwareMap);
//        Arm arm = new Arm(hardwareMap);
//        Claw claw = new Claw(hardwareMap);
//
//        // vision here that outputs position
//
//        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
//                .stopAndAdd(arm.armPreScoring())
//                .strafeTo(new Vector2d(-10,24))
//                .stopAndAdd(slide.slideUp())
//                .waitSeconds(1)
//                .stopAndAdd(arm.armPostScoring())
//                .waitSeconds(1)
//                .stopAndAdd(claw.openClaw())
//                .waitSeconds(1)
//                .stopAndAdd(arm.armPickUp())
//                .waitSeconds(.5)
//                .setTangent(Math.PI/2)
//                .afterDisp(50, slide.slideDown())
//                .strafeTo(new Vector2d(-24,54))
//                .setTangent(-Math.PI/2)
//                .lineToY(52)
//                .splineTo(new Vector2d(-52, 5),-Math.PI/2)
//                .lineToY(50)
//                .lineToY(30)
//                .splineTo(new Vector2d(-62, 5),-Math.PI/2)
//                .lineToY(50)
//                .lineToY(30)
//                .splineTo(new Vector2d(-71,5),-Math.PI/2)
//                .setTangent(-Math.PI/2)
//                .lineToY(50);
//
//
//
//
//
//
//
//
//
//        TrajectoryActionBuilder score = drive.actionBuilder(new Pose2d(0,-30,Math.toRadians(90)))
//                .strafeTo(new Vector2d(1,1))
//                .stopAndAdd(arm.counterclockwise())
//                .stopAndAdd(arm.clockwise())
//                .lineToY(-30);
//
//        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.openClaw());
//        Actions.runBlocking(arm.clockwise());
//        Actions.runBlocking(arm.armInit());
//        ElapsedTime time = new ElapsedTime();
//        while (time.seconds()<2){
//            telemetry.addData("doing","nothing");
//        }
//        Actions.runBlocking(claw.closeClaw());
//
//
//
//
//        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addData("x",slide.vals());
//            telemetry.update();
//        }
//
//        telemetry.update();
//        waitForStart();
//
//        if (isStopRequested()) return;
//        Action traj = temp.build();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        traj
//                )
//        );
//    }
//}