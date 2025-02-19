package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;

@Config

public class LiftUtil {

    public static double vertSlidesError = 0;

    public static double vertSlidesUpP = 0.001;
    public static double vertSlidesUpI = 0.00002;

    public static double vertSlidesUpD = 1;
    public static double vertSlideintegralSum = 0;

    public static double VertSlidesLastError = .25;

    public static double vertSlidesA = 0;

    public static double vertSlidesDownP = 0.000004;
    public static double vertSlidesDownI = 0.0000;

    public static double horiSlidesError = 0;

    public static double horiSlidesUpP = 0.001;
    public static double horiSlidesUpI = 0.00002;

    public static double horiSlidesUpD = 1;
    public static double horiSlideintegralSum = 0;

    public static double horiSlidesA = 0;

    public static double horiSlidesLastError = .25;

    public static double armError = 0;

    public static double armP = 0.001;
    public static double armI = 0;

    public static double armD = 1;
    public static double armIntegralSum = 0;

    public static double armA = 0;

    public static double armLastError = .25;





}