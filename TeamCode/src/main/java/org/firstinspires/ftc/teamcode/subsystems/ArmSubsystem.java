package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;

public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx armMotor;
    private final DcMotorEx extendo;


    private double armStart;
    private double armTarget;
    private static final double ARM_F = 0.3;
    private static final double TICK_PER_RAD = ((((1 + (46.0 / 11.0))) * (1 + (46.0 / 11.0))) * 28) / (2 * Math.PI) / 0.333;
    private static final double ARM_MIN = 0;
    private static final double ARM_MAX = 1.9;
    private static double power = 0;
    CustomPIDFCoefficients pid = new CustomPIDFCoefficients(3, 0, 0, ARM_F);

    private PIDFController armPID;

    public ArmSubsystem(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        this.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        this.extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        this.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.extendo.setDirection(DcMotorEx.Direction.REVERSE);
        this.extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        this.armPID = new PIDFController(pid);
        this.armStart = armMotor.getCurrentPosition();
        this.armTarget = 0.0; // Start at the initial position
    }

    public double armAngle() {
        return ((-armMotor.getCurrentPosition()) / TICK_PER_RAD)- Math.toRadians(28);
    }

    public void setRawArmPower(double power) {
        armMotor.setPower(power);
    }
    public void setRawExtendoPower(double power) {
        extendo.setPower(power);
    }


    public void moveArmTo(double targetAngle) {
//        double minFeedforward = Math.cos(armAngle()) * 0.1 / ARM_F;
//        double maxFeedforward = Math.cos(armAngle()) * (2 + (getExtendoPosition() / 2800)) * -0.4;
//
//        double normalizedPosition = getExtendoPosition() / 2800.0;
//        double scaledPosition = Math.pow(normalizedPosition, 3);
//
//        // Only apply feedforward if extendo position is above a threshold (to avoid excess power)
//        if (normalizedPosition > 0.1) {  // Change threshold as needed
//            double interpolatedFeedforward = minFeedforward + scaledPosition * (maxFeedforward - minFeedforward);
//            power = interpolatedFeedforward;
//        } else {
//            power = 0;  // No power if position is too small
//        }

        armPID.updateFeedForwardInput(Math.cos(armAngle())*(1+Math.pow((getExtendoPosition()/2800),2)));
        armPID.setTargetPosition(targetAngle);
        armPID.updatePosition(armAngle());
        armMotor.setPower(-armPID.runPIDF());
    }

    public void manualControl(double stickInput) {
        if (Math.signum(stickInput) == 1) {
            armTarget = armAngle() - (stickInput * 0.1);
        }
        else {
            armTarget = armAngle() - (stickInput * 0.2);
        }
        // Adjust the speed multiplier here
        moveArmTo(armTarget);

    }


    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }
    public double getExtendoPosition() {
        return extendo.getCurrentPosition();
    }
    public double getExtendoPower() {
        return power;
    }
    public double getExtendoCurrent() {
        return extendo.getCurrent(CurrentUnit.AMPS);
    }
    public double getArmTarget(){
        return armPID.getTargetPosition();
    }

}
