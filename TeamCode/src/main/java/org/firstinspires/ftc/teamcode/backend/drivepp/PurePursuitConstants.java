package org.firstinspires.ftc.teamcode.backend.drivepp;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PurePursuitConstants {
    public static double xP = 0.095;
    public static double xD = 0.011;

    public static double yP = 0.09;
    public static double yD = 0.011;

    public static double hP = 1.1;
    public static double hD = 0.045;

    public static double MAX_TRANSLATIONAL_SPEED = 1;
    public static double MAX_ROTATIONAL_SPEED = 1;
    public static double X_GAIN = 1.00;

    public static double ALLOWED_TRANSLATIONAL_ERROR = 1;
    public static double ALLOWED_HEADING_ERROR = 0.03;
}