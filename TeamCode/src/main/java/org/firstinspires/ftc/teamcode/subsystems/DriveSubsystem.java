//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.backend.commandbaselibrary.SubsystemBase;
//import org.firstinspires.ftc.teamcode.util.geometry.Pose;
//
//public class DriveSubsystem extends SubsystemBase{
//
//    private final DcMotor FL;
//    private final DcMotor FR;
//    private final DcMotor BL;
//    private final DcMotor BR;
//
//    public DriveSubsystem(final HardwareMap hardwareMap){
//        FL = hardwareMap.get(DcMotor.class, "FL");
//        FR = hardwareMap.get(DcMotor.class, "FR");
//        BL = hardwareMap.get(DcMotor.class, "BL");
//        BR = hardwareMap.get(DcMotor.class, "BR");
//
//        FL.setDirection(DcMotor.Direction.REVERSE);
//        BL.setDirection(DcMotor.Direction.REVERSE);
//    }
//
//    public void setPower(double power){
//        FL.setPower(power);
//        FR.setPower(power);
//        BL.setPower(power);
//        BR.setPower(power);
//    }
//    //sets robot relative power, raw function dont do calc here just give power
//    public void setHolonomicPower(Pose drivePower) {
//        FL.setPower(drivePower.getX() - drivePower.getY() + drivePower.heading);
//        BL.setPower(drivePower.getX() + drivePower.getY() + drivePower.heading);
//        FR.setPower(drivePower.getX() + drivePower.getY() - drivePower.heading);
//        BR.setPower(drivePower.getX() - drivePower.getY() - drivePower.heading);
//    }
//    public void setTankPower(double f,double t){
//        FL.setPower(f+t);
//        FR.setPower(f-t);
//        BL.setPower(f+t);
//        BR.setPower(f-t);
//    }
//
//
//}