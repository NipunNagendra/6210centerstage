package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class ArmSubsystem extends SubsystemBase {

    private final DcMotorEx armMotor;
    private final DcMotorEx extendo;
    public static double armP=2;
    public static double armD=0.01;
    public static double exP;
    public static double exD;


    private double armStart;
    private double armTarget= -Math.toRadians(-28);
    private double extendoTarget= 0;

    private static final double ARM_F = 0.3;
    private static final double TICK_PER_RAD = ((((1 + (46.0 / 11.0))) * (1 + (46.0 / 11.0))) * 28) / (2 * Math.PI) / 0.333;
    private static final double ARM_MIN = Math.toRadians(-25);
    private static final double ARM_MAX = Math.toRadians(84);
    private static final double EXTENDO_MIN = 0;
    private static final double EXTENDO_MAX = 2900;
    private static double power = 0;
    CustomPIDFCoefficients pid = new CustomPIDFCoefficients(armP, 0, armD, ARM_F);
    CustomPIDFCoefficients extendoPidConstants = new CustomPIDFCoefficients(0.001, 0, 0, 0);

    private PIDFController armPID;
    private PIDFController extendoPID;


    public ArmSubsystem(HardwareMap hardwareMap) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        this.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        this.extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        this.armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.extendo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.extendo.setDirection(DcMotorEx.Direction.REVERSE);
        this.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extendo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        this.armPID = new PIDFController(pid);
        this.extendoPID = new PIDFController(extendoPidConstants);
        this.extendoPID.reset();
//        this.armPID.reset();
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
            if(targetAngle>ARM_MAX){
                targetAngle=ARM_MAX;
            }
            if(targetAngle<ARM_MIN){
                targetAngle=ARM_MIN;
            }
            armTarget=targetAngle;
    }

    @Override
    public void periodic(){
        armPID.updateFeedForwardInput(Math.cos(armAngle())*(1+Math.pow((getExtendoPosition()/2800),2)));
        armPID.setTargetPosition(armTarget);
        armPID.updatePosition(armAngle());
        armMotor.setPower(-armPID.runPIDF());

        extendoPID.setTargetPosition(extendoTarget);
        extendoPID.updatePosition(getExtendoPosition());
        double val = extendoPID.runPIDF();
        extendo.setPower(Math.signum(val)*(Math.sqrt(Math.abs(val))));
    }



    public void moveExtendoTo(double targetTicks) {
        extendoTarget=targetTicks;
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
