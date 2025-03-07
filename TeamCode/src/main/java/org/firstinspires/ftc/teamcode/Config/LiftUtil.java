package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config

public class  LiftUtil {

    public static double vertSlidesError = 0;

    public static double AutoVertSlidesError = 0;


    public static double target = 12200;
    public static double downTarget = 700;

// vslide
    public static double vertSlideintegralSum = 0;

    public static double VertSlidesLastError = .25;

    public static double vertSlidesA = 0;

    public static double vertSlidesUpP = 0.02;
    public static double vertSlidesUpI = 0.0000005;

    public static double vertSlidesUpD = 0;

    public static double vertSlidesDownP = 0.02;
    public static double vertSlidesDownI = 0.0000005;

    // hori slide

    public static double horiSlidesError = 0;

    public static double horiSlidesUpP = 0.001;
    public static double horiSlidesUpI = 0.00002;

    public static double horiSlidesUpD = 1;
    public static double horiSlideintegralSum = 0;

    public static double horiSlidesA = 0;

    public static double horiSlidesLastError = 0;

// auto vslide

    public static double AutoVertSlideintegralSum = 0;

    public static double AutoVertSlidesLastError = .25;

    public static double AutoVertSlidesA = 0;

    public static double AutoVertSlidesUpP = 0.000192;
    public static double AutoVertSlidesUpI = 0.000005;

    public static double AutoVertSlidesUpD = 0;

    public static double AutoVertSlidesDownP = 0;
    public static double AutoVertSlidesDownI = 0.0000005;










}