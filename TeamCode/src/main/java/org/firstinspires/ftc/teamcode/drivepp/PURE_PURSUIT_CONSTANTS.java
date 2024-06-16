package org.firstinspires.ftc.teamcode.drivepp;

public final class PURE_PURSUIT_CONSTANTS {
    private PURE_PURSUIT_CONSTANTS() {
    }
    //point injecion and smoothing
    public static double spacing = 0.5;
    public static double weightData = 0.1;
    public static double weightSmooth = 0.1;
    public static double tolerance = 0.00001;
    //point calculations
    public static double maxVelocity = 50;
    public static double maxAcceleration = 50;
    public static double k = 1;
}
