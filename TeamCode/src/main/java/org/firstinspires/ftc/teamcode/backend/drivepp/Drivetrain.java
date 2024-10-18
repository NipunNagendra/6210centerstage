package org.firstinspires.ftc.teamcode.backend.drivepp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;


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

        this.FL.setDirection(DcMotorSimple.Direction.FORWARD);
        this.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FR.setDirection(DcMotorSimple.Direction.FORWARD);
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

        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.heading);
        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.heading);
        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.heading);
        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.heading);
    }

    public void setFieldWeightedDrivePower(Pose drivePower, double heading) {
        Vector2D fieldFrame = new Vector2D(drivePower.getX(), drivePower.getY()).rotate(heading);
        Pose vel = new Pose(fieldFrame.getX(), fieldFrame.getY(), drivePower.heading);

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.heading) > 1) {
            double denom = VX_WEIGHT * Math.abs(drivePower.getX()) + VY_WEIGHT * Math.abs(drivePower.getY()) + OMEGA_WEIGHT * Math.abs(drivePower.heading);
            vel = new Pose(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.heading).scale(1/denom);
        }

        FL.setPower(vel.getX() - lateralMultiplier * vel.getY() - d * vel.heading);
        FR.setPower(vel.getX() + lateralMultiplier * vel.getY() + d * vel.heading);
        BL.setPower(vel.getX() + lateralMultiplier * vel.getY() - d * vel.heading);
        BR.setPower(vel.getX() - lateralMultiplier * vel.getY() + d * vel.heading);
    }

    public void setTankPower(double f,double t){
        FL.setPower(f+t);
        FR.setPower(f-t);
        BL.setPower(f+t);
        BR.setPower(f-t);
    }


}
