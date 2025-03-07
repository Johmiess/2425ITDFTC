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

import org.firstinspires.ftc.teamcode.Config.LiftUtil;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RedSpicemen", group = "Autonomous")
public class RedSpicemen extends LinearOpMode {


    public static double iterationTime = .13;
    public static double clawClosedPos = .55;

    public static double armPos = 0.265;


    public static double clawOpenPos = .3;
    public static double clawLeft = .23;
    public static double clawRight = .9;
    public static double preScore = 0.15;
    public static double postScore = 0.01;
    public static double pickUp = 0.35;

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
            return new Claw.CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(clawOpenPos);
                return false;
            }
        }
        public Action openClaw() {
            return new Claw.OpenClaw();
        }
    }
    public class VertSlide {
        private DcMotorEx leftThing, rightThing;
        private ElapsedTime time;

        private double output;


        public VertSlide(HardwareMap hardwareMap){
            leftThing = hardwareMap.get(DcMotorEx.class, "leftThing");
            rightThing = hardwareMap.get(DcMotorEx.class, "rightThing");
            time = new ElapsedTime();
            leftThing.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftThing.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftThing.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("ready","ye");}

        public void verticalSlides(double speed) {
            leftThing.setPower(speed);
            rightThing.setPower(speed);
        }
        public double vals(){
            return leftThing.getCurrentPosition();
        }

        public double output(){
            return output;
        }
        public void reset(){
            LiftUtil.vertSlideintegralSum = 0;
            LiftUtil.horiSlideintegralSum = 0;
        }
        public class slideUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                ElapsedTime timer = new ElapsedTime();
                while (timer.seconds() < 3) {
                    double currentPos = (leftThing.getCurrentPosition());
                    LiftUtil.AutoVertSlidesError = LiftUtil.target - currentPos;
                    LiftUtil.AutoVertSlideintegralSum += LiftUtil.vertSlidesError;
                    output = (LiftUtil.AutoVertSlidesUpP * LiftUtil.AutoVertSlidesError) + (LiftUtil.AutoVertSlidesUpI * LiftUtil.AutoVertSlideintegralSum) + (LiftUtil.AutoVertSlidesUpD) + LiftUtil.AutoVertSlidesA;
                    verticalSlides(-output);
                    LiftUtil.AutoVertSlidesLastError = LiftUtil.AutoVertSlidesError;
                    telemetry.addData("target",LiftUtil.target);
                    telemetry.addData("error",LiftUtil.AutoVertSlidesError);
                    telemetry.addData("Output", output());
                    telemetry.addData("x",vals());
                    telemetry.addData("y",currentPos);
                    telemetry.update();
                }
                reset();
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

        public Action slideUp() {return new VertSlide.slideUp();}

        public Action slideDown() {return new VertSlide.slideDown();}
    }

    public class Arm{
        private ServoImplEx leftAxon, rightAxon;
        private ServoImplEx rotate;
        private ElapsedTime time;
        static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        static final int    CYCLE_MS    =   50;     // period of each cycle

        public double CURR_POS     =  0;     // Maximum rotational position

        public double TARGET_POS     =  .15;     // Maximum rotational position
        static final double MIN_POS     = .05;

        public double pos = 0.256;

        boolean rampUp = true;

        public Arm(HardwareMap hardwareMap){
            leftAxon = hardwareMap.get(ServoImplEx.class,"leftAxon");
            rightAxon = hardwareMap.get(ServoImplEx.class, "rightAxon");
            rotate = hardwareMap.get(ServoImplEx.class,"rotate");
        }


        public class clockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rotate.setPosition(clawRight);
                telemetry.addData("claw", rotate.getPosition());
                telemetry.update();
                return false;
            }
        }
        public class counterclockwise implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                rotate.setPosition(clawLeft);
                telemetry.addData("claw", rotate.getPosition());
                telemetry.update();
                return false;
            }
        }

        public class armInit implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(armPos);
                rightAxon.setPosition(armPos);
                telemetry.addData("LA", leftAxon.getPosition());
                telemetry.addData("RA", rightAxon.getPosition());
                telemetry.update();

                return false;
            }
        }

        public class armPostScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(postScore);
                rightAxon.setPosition(postScore);
                telemetry.addData("LA", leftAxon.getPosition());
                telemetry.addData("RA", rightAxon.getPosition());
                telemetry.update();

                return false;
            }
        }
        public class armPreScoring implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(preScore);
                rightAxon.setPosition(preScore);
                telemetry.addData("LA", leftAxon.getPosition());
                telemetry.addData("RA", rightAxon.getPosition());
                telemetry.update();

                return false;
            }
        }


        public class armPickUp implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                leftAxon.setPosition(pickUp);
                rightAxon.setPosition(pickUp);
                return false;
            }
        }
        public Action armInit(){return new armInit();}

        public Action armPickUp(){return new armPickUp();}
        public Action armPreScoring(){return new armPreScoring();}

        public Action armPostScoring(){return new armPostScoring();}

        public Action clockwise(){return new Arm.clockwise();}
        public Action counterclockwise(){return new Arm.counterclockwise();}

    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8, -62.5, Math.toRadians(90)));
        VertSlide slide = new VertSlide(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // vision here that outputs position

        TrajectoryActionBuilder temp = drive.actionBuilder(drive.pose)
                /*.stopAndAdd(arm.armPreScoring())
              .strafeTo(new Vector2d(10,-24))
              .stopAndAdd(slide.slideUp())
              .waitSeconds(1)
              .stopAndAdd(arm.armPostScoring())
              .waitSeconds(1)
              .stopAndAdd(claw.openClaw())
              .waitSeconds(1)
              .stopAndAdd(arm.armPickUp())
              .waitSeconds(.5)
              .setTangent(-Math.PI/2)
              .afterDisp(50, slide.slideDown())
              .strafeTo(new Vector2d(24,-54))
              .setTangent(-Math.PI/2)
              .lineToY(-52)*/
                .setTangent(Math.PI)
                .lineToX(0)
                .setTangent(Math.PI/2)
                .lineToY(-25)
                .stopAndAdd(arm.armPreScoring())
                .waitSeconds(1)
                .stopAndAdd(slide.slideUp())
                .stopAndAdd(arm.armPostScoring())
                .waitSeconds(.5)
                .stopAndAdd(claw.openClaw())
                .waitSeconds(1)
                .lineToY(-50)
;
        //grab spec
//                .strafeTo(new Vector2d(65,-54))
//                .waitSeconds(2)
//                .stopAndAdd(claw.closeClaw());
/*                .stopAndAdd(arm.armPreScoring())
                .afterDisp(50, slide.slideUp() )
                .strafeTo(new Vector2d(7,-27))
//                .stopAndAdd(arm.armPostScoring())
                .stopAndAdd(claw.openClaw())
                .waitSeconds(.5)
                .stopAndAdd(arm.armPickUp())
                .afterDisp(50, slide.slideDown() )
                .strafeTo(new Vector2d(65,-54))
                .waitSeconds(1)
                .stopAndAdd(claw.closeClaw())
                .stopAndAdd(arm.armPreScoring())
//                .stopAndAdd(arm.armPostScoring())
                .afterDisp(50, slide.slideUp() )
                .strafeTo(new Vector2d(2,-27))
                .stopAndAdd(arm.armPreScoring());*/










        TrajectoryActionBuilder score = drive.actionBuilder(new Pose2d(0,-30,Math.toRadians(90)))
                .strafeTo(new Vector2d(1,1))
                .stopAndAdd(arm.counterclockwise())
                .stopAndAdd(arm.clockwise())
                .lineToY(-30);

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(arm.armInit());
        Actions.runBlocking(arm.counterclockwise());
        Actions.runBlocking(claw.openClaw());
        ElapsedTime time = new ElapsedTime();
        while (time.seconds()<2){
            telemetry.addData("doing","nothing");
        }
        Actions.runBlocking(claw.closeClaw());
        time.reset();
        while (time.seconds()<1){
            telemetry.addData("doing","nothing");
        }
        Actions.runBlocking(arm.clockwise());





        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("x",slide.vals());
            telemetry.addData("GET OUTPUT", slide.output());
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