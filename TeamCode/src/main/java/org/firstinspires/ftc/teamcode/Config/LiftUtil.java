package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config

public class LiftUtil {
    public static double LIFTP = 0.001;
    public static double LIFTI = 0.00002;

    public static double LIFTD = 1;
    public static double integralSum = 0;

    public static double time = 2;
    //reduce power by .025 if 13
    public static double power = .325;

    public static double lastError = .25;

    public static double LIFTA = 0;

    public static double target = 800;
    public static double target0 = 50;


    public static double DOWNLIFTP = 0.000004;
    public static double DOWNLIFTI = 0.0000;
    public static double DOWNLIFTD = 0;

    public static double V4bPos1 = 0;
    public static double V4bPos2 = 0.8;

    public static double Intakefailsafe = 2;

}