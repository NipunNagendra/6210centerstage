package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.PIDFController;
import org.firstinspires.ftc.teamcode.util.CustomPIDFCoefficients;

public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx armMotor;
    private final DcMotorEx extendo;


    private double armStart;
    private double armTarget;
    private static final double ARM_F = -0.2;
    private static final double TICK_PER_RAD = ((((1 + (46.0 / 11.0))) * (1 + (46.0 / 11.0))) * 28) / (2 * Math.PI) / 0.333;
    private static final double ARM_MIN = 0;
    private static final double ARM_MAX = 1.9;
    private static double power = 0;
    CustomPIDFCoefficients pid = new CustomPIDFCoefficients(0, 0, 0, ARM_F);

    private PIDFController armPID;

    public ArmSubsystem(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        this.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
        return (-armMotor.getCurrentPosition()) / TICK_PER_RAD;
    }

    public void setRawArmPower(double power) {
        armMotor.setPower(power);
    }
    public void setRawExtendoPower(double power) {
        extendo.setPower(power);
    }


    public void moveArmTo(double targetAngle) {
        power=Math.cos((armAngle()))*(2*(getExtendoPosition()/2987));
        armPID.updateFeedForwardInput(Math.cos((armAngle()))*(2*(getExtendoPosition()/2987)));
        armPID.setTargetPosition(targetAngle);
        armPID.updatePosition(armAngle());
        armMotor.setPower(armPID.runPIDF());
    }

    public void manualControl(double stickInput) {
        armTarget -= stickInput * 0.2; // Adjust the speed multiplier here
//        if (armTarget < ARM_MIN) {
//            armTarget = ARM_MIN;
//        } else if (armTarget > ARM_MAX) {
//            armTarget = ARM_MAX;
//        }
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


}
