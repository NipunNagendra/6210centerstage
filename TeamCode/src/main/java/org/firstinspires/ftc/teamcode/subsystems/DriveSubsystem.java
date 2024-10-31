package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.geometry.Pose;

public class DriveSubsystem extends SubsystemBase {

    private final DcMotor FL;
    private final DcMotor FR;
    private final DcMotor BL;
    private final DcMotor BR;

    public DriveSubsystem(final HardwareMap hardwareMap){
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        this.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.BR.setDirection(DcMotorSimple.Direction.FORWARD);
        this.FR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }
    //sets robot relative power, raw function dont do calc here just give power
    public void setHolonomicPower(Pose drivePower) {
        Pose vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.heading) > 1) {
            double denom =  Math.abs(drivePower.getX()) + Math.abs(drivePower.getY()) + Math.abs(drivePower.heading);
            vel = new Pose(
                    drivePower.getX(),
                    drivePower.getY(),
                    drivePower.heading).scale(1/denom);
        }


        FL.setPower(vel.getX() - vel.getY() -  vel.heading);
        FR.setPower(vel.getX() + vel.getY() +  vel.heading);
        BL.setPower(vel.getX() +  vel.getY() - vel.heading);
        BR.setPower(vel.getX() -  vel.getY() +  vel.heading);
    }
    public void setTankPower(double f,double t){
        FL.setPower(f+t);
        FR.setPower(f-t);
        BL.setPower(f+t);
        BR.setPower(f-t);
    }


}