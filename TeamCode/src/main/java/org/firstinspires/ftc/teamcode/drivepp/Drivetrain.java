package org.firstinspires.ftc.teamcode.drivepp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    public static double lateralMultiplier = 1.0;
    public static double trackWidth = 1.0;
    public static double wheelBase = 1.0;
    public static double VX_WEIGHT = 1.0;
    public static double VY_WEIGHT = 1.0;
    public static double OMEGA_WEIGHT = 1.0;
    double d = (trackWidth + wheelBase) / 2.0;


    public Drivetrain(HardwareMap hardwareMap) {
        this.FL = hardwareMap.get(DcMotor.class, "FL");
        this.FR = hardwareMap.get(DcMotor.class, "FR");
        this.BL = hardwareMap.get(DcMotor.class, "BL");
        this.BR = hardwareMap.get(DcMotor.class, "BR");
    }

    public void setPower(double power) {
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    public void setSplitPower(double leftPower, double rightPower) {
        FL.setPower(leftPower);
        BL.setPower(leftPower);
        FR.setPower(rightPower);
        BR.setPower(rightPower);
    }

    public void setRobotWeightedDrivePower(Pose drivePower) {
        Pose vel = drivePower;
        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1) {
            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
            vel = new Pose(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.getHeading());
        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.getHeading());
        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.getHeading());
        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.getHeading());
    }

    public void setFieldWeightedDrivePower(Pose drivePower, double heading) {
        Vector fieldFrame = new Vector(drivePower.getX(), drivePower.getY()).rotate(-heading);
        Pose vel = new Pose(fieldFrame.getX(), fieldFrame.getY(), drivePower.getHeading());

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.getHeading()) > 1) {
            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
            vel = new Pose(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.getHeading());
        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.getHeading());
        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.getHeading());
        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.getHeading());
    }
}
