package org.firstinspires.ftc.teamcode.util.libs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Roadrunner.driverr.SampleMecanumDrive;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;


public class Movement {

    HardwareMap robot;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    public final double FL_PROPORTION;
    public final double FR_PROPORTION;
    public final double BL_PROPORTION;
    public final double BR_PROPORTION;
    public static Pose2d vel;
    public HashMap<String, Boolean> buttons = new HashMap<String, Boolean>();

    public static Vector2d input;


    public SampleMecanumDrive drive;


    public Movement(HardwareMap hardwareMap) {
        this.robot = hardwareMap;
        drive = new SampleMecanumDrive(hardwareMap);

        FL = robot.get(DcMotor.class, "FL");
        FR = robot.get(DcMotor.class, "FR");
        BL = robot.get(DcMotor.class, "BL");
        BR = robot.get(DcMotor.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //motors are reversed than what they shld be
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL_PROPORTION = 1;
        FR_PROPORTION = 1;
        BL_PROPORTION = 1;
        BR_PROPORTION = 1;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double[] holonomicDrive(double leftX, double leftY, double rightX) {
        double[] motorpowers = new double[4];
        motorpowers[0] = leftY - leftX - rightX;
        motorpowers[1] = leftY + leftX + rightX;
        motorpowers[2] = leftY + leftX - rightX;
        motorpowers[3] = leftY - leftX + rightX;
        return motorpowers;
    }

    public void fieldDrive(double leftX, double leftY, double rightX, double imu, double targetAngle) {
        input = new Vector2d(
                leftY,
                leftX
        ).rotated(-imu);


        vel = CustomMecanumKinematics.normalizedDrivePowerPose(new Pose2d(
                        input.getX(),
                        input.getY(),
                        rightX+targetAngle
                        //make this imu to lock in
                ));

        setPowers(CustomMecanumKinematics.inverseKinematicModel(vel));

    }



    public void setPowers(double[] motorPower) {

        FL.setPower(motorPower[0] * FL_PROPORTION);
        FR.setPower(motorPower[1] * FR_PROPORTION);
        BL.setPower(motorPower[2] * BL_PROPORTION);
        BR.setPower(motorPower[3] * BR_PROPORTION);
    }

    public void setPowers(double fr, double fl, double br, double bl) {

        FR.setPower(fr);
        FL.setPower(fl);
        BR.setPower(br);
        BL.setPower(bl);
    }


    public void kill(){
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }


    public boolean isPressed(String name, boolean button){
        boolean output = false;

        //If the hashmap doesn't already contain the key
        if (!buttons.containsKey(name)) {
            buttons.put(name, false);
        }

        boolean buttonWas = buttons.get(name);
        if (button != buttonWas && button == true){
            output = true;
        }

        buttons.put(name, button);

        return output;
    }

    public List<Double> getWheelPowers(){
        return Arrays.asList(FL.getPower(), FR.getPower(), BL.getPower(), BR.getPower());
    }
}

