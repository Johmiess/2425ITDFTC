/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.

 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)

 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.

 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".

 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.

 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */
@Config
public class  robot {
    // declaring all the variables that are gonna be used
    public LinearOpMode myOpMode;
    public OpMode notMyopMode;
    public DcMotorEx leftFront, leftBack, rightFront, rightBack, rightThing, leftThing, intake;
    public ServoImplEx claw;
    public ServoImplEx rightAxon, leftAxon;

    public AnalogInput leftArm, rightArm;
    public  double output;
    public double currentPos;
    public robot(LinearOpMode opmode) {
        myOpMode = opmode;
    }
    public robot(OpMode opmode) {
        notMyopMode = opmode;
    }



    public double close_pos = 0.69;
    public double open_pos = 0.10;


    public void init() {
        leftFront = myOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = myOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");

//        intake = myOpMode.hardwareMap.get(DcMotorEx.class, "intake");
        rightThing = myOpMode.hardwareMap.get(DcMotorEx.class, "rightThing");
        leftThing = myOpMode.hardwareMap.get(DcMotorEx.class, "leftThing");
        leftAxon = myOpMode.hardwareMap.get(ServoImplEx.class, "leftAxon");
        rightAxon = myOpMode.hardwareMap.get(ServoImplEx.class, "rightAxon");
        // analog for our axon encoder postions
        leftArm =  myOpMode.hardwareMap.get(AnalogInput.class, "leftArm");
        rightArm =  myOpMode.hardwareMap.get(AnalogInput.class, "rightArm");
        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
// get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees

//        claw = myOpMode.hardwareMap.get(ServoImplEx.class, "claw");

//        claw.setPosition(0);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightThing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftThing.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        setMotorPowers(0);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);




    }

    public void setMotorPowers(double speed) {
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }
    public void setMotorPowers(double lf, double lb,double rf, double rb) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }


    /**
     * Two different PID methods for vert slides
     * @param target
     */
    public void vertSlidesPIDup(double target){
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime oneSec = new ElapsedTime();
        //currentPos = (lift.getCurrentPosition());
        LiftUtil.vertSlidesError = target - currentPos;
        LiftUtil.vertSlideintegralSum += LiftUtil.vertSlidesError;
        double derivative = (LiftUtil.vertSlidesError - LiftUtil.VertSlidesLastError) / timer.seconds();
        output = (LiftUtil.vertSlidesUpP * LiftUtil.vertSlidesError) + (LiftUtil.vertSlidesUpI * LiftUtil.vertSlideintegralSum) + (LiftUtil.vertSlidesUpD * derivative) + (LiftUtil.vertSlidesA);
        output = (LiftUtil.vertSlidesUpP * LiftUtil.vertSlidesError) + (LiftUtil.vertSlidesUpI * LiftUtil.vertSlideintegralSum) + (LiftUtil.vertSlidesA);
        //lift.setPower(output);
        LiftUtil.VertSlidesLastError = LiftUtil.vertSlidesError;
    }

    public void vertSlidesPIDdown(double target){
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime oneSec = new ElapsedTime();
        //currentPos = (lift.getCurrentPosition());
        LiftUtil.vertSlidesError = target - currentPos;
        LiftUtil.vertSlideintegralSum += LiftUtil.vertSlidesError;
        double derivative = (LiftUtil.vertSlidesError - LiftUtil.VertSlidesLastError) / timer.seconds();
        output = (LiftUtil.vertSlidesDownP * LiftUtil.vertSlidesError) + (LiftUtil.vertSlidesDownI * LiftUtil.vertSlideintegralSum) + (LiftUtil.vertSlidesA);
        //lift.setPower(output);
        LiftUtil.VertSlidesLastError = LiftUtil.vertSlidesError;
    }

    /**
     * PID FOR HORIZONTAL SLIDES
     * @param target
     */
    public void horiSlidesPID(double target){
        ElapsedTime timer = new ElapsedTime();
//        currentPos = (lift.getCurrentPosition());
        LiftUtil.horiSlidesError = target - currentPos;
        LiftUtil.horiSlideintegralSum += LiftUtil.horiSlidesError;
        output = (LiftUtil.horiSlidesUpP * LiftUtil.horiSlidesError) + (LiftUtil.horiSlidesUpI * LiftUtil.horiSlideintegralSum) + (LiftUtil.horiSlidesA);
        //lift.setPower(output);
        LiftUtil.VertSlidesLastError = LiftUtil.vertSlidesError;
    }

    /**
     * ARM PID BASED OFF OF THE LEFT ARM ENCODER POSITION :)
     * @param target
     */
    public void armPID(double target){
        ElapsedTime timer = new ElapsedTime();
        currentPos = (getLeftArmEncoderPosition());
        double rightPos = getRightArmEncoderPosition();
        LiftUtil.armError = target - currentPos;
        LiftUtil.armIntegralSum += LiftUtil.armError;
        output = (LiftUtil.armP * LiftUtil.armError) + (LiftUtil.armI * LiftUtil.armIntegralSum) + (LiftUtil.armA) + LiftUtil.armA;
        LiftUtil.armLastError = LiftUtil.armError;
        leftAxon.setPosition(output);
        rightAxon.setPosition((currentPos-output)+rightPos);
    }






//    public void intake(double sp) {
//        intake.setPower(sp);
//    }


    /**
     * verticalSlidesUP & verticalSlidesDown: Running leftThing and rightThing in oppsite directions should spin the
     * verticle
     * @param speed
     */

    //positive speed = up, negative speed = down
    public void horizontalSlides(double speed) {
        leftThing.setPower(-speed);
        rightThing.setPower(speed);
    }

    public void verticalSlides(double speed){
        leftThing.setPower(speed);
        rightThing.setPower(speed);
    }

    /**
    Methods: armFoward and armBack
    to make arm spin foward, Right Axon should be powered negative, left postive. For
    right vice versa
    - DONT MAKE THE ARM FOWARD/BACKWARD SPEED MORE THAN 0.25
     **/
    public void clawSpinClockWise(double power){
        rightAxon.setPosition((-power));
        leftAxon.setPosition((power));
    }

    public void clawSpinCounterClockWise(double power){
        rightAxon.setPosition((power));
        leftAxon.setPosition((-power));
    }

    /** to make claw spin:
    (relative to the front)
    both postive: clockwise
    both negative: counterclockwise
      **/

    public void armBack (double power){ //claw backword
        rightAxon.setPosition(-power);
        leftAxon.setPosition(-power);
    }

    public void armFoward (double power){ // clawfoward
        rightAxon.setPosition(power);
        leftAxon.setPosition(power);
    }

    public void setIntake (double power){ // setting power to intake
        intake.setPower(power);
    }





    /**
     getRightArmEncoderPosition & getleftArmEncoderPosition
     @return Encoder postion of arm
     **/
    public double getRightArmEncoderPosition(){ return  rightArm.getVoltage() / 3.3 * 360;}
    public double getLeftArmEncoderPosition(){return  leftArm.getVoltage() / 3.3 * 360;}



    public void claw(double posi){
        claw.setPosition(posi);
    }

}