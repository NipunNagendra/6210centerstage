//package org.firstinspires.ftc.teamcode.;
//
//import android.util.Log;
//
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.optimizations.RingBuffer;
//import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;
//import org.firstinspires.ftc.teamcode.util.rrutil.EncoderSensor;
//import org.firstinspires.ftc.teamcode.utils.AngleUtil;
//import org.firstinspires.ftc.teamcode.utils.Pose2d;
//import org.firstinspires.ftc.teamcode.utils.SparkFunOTOS;
//import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
//import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
//import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
//
//public class Sensors {
//    private LynxModule controlHub, expansionHub;
//    private final HardwareMap hardwareMap;
//
//    //private IMU imu;
//    private int[] odometry = new int[]{0, 0, 0};
//
//    private int slidesEncoder;
//    private double slidesVelocity;
//    private boolean slidesDown = false;
//    private boolean intakeTriggered = false;
//    private boolean depositTouched = false;
//
//    private final AnalogInput[] analogEncoders = new AnalogInput[2];
//    private double backUltrasonicDist, frontUltrasonicDist = 0;
//    public double[] analogVoltages = new double[analogEncoders.length];
//
//    private double voltage;
//
//    HuskyLens.Block[] huskyLensBlocks;
//
//    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;
//
//    private EncoderSensor parallelEncoder, perpendicularEncoder;
//
//    public static double voltageK = 0.3;
//
//    private IMU imu;
//
//    public Sensors(HardwareMap hardwareMap) {
//        this.hardwareMap = hardwareMap;
//        parallelEncoder = new EncoderSensor(hardwareMap.get(DcMotorEx.class, "BR"));
//        perpendicularEncoder = new EncoderSensor(hardwareMap.get(DcMotorEx.class, "FR"));
//        parallelEncoder.resetEncoder();
//        perpendicularEncoder.resetEncoder();
//        imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
//                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
//
//        imu.initialize(parameters);
//        imu.resetYaw();
//
//        initSensors(hardwareMap);
//    }
//
//    private void initSensors(HardwareMap hardwareMap) {
//        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
//        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//
//        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
//        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//
//        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//    }
//
//    public void update() {
//        updateControlHub();
//        updateExpansionHub();
//    }
//
//    private double imuUpdateTime = 15;
//    public double timeTillNextIMUUpdate = imuUpdateTime;
//    public boolean imuJustUpdated = false;
//
//    private double voltageUpdateTime = 5000;
//    long lastVoltageUpdatedTime = System.currentTimeMillis();
//
//    private double huskyUpdateTime = 100;
//    long lastHuskyLensUpdatedTime = System.currentTimeMillis();
//    public boolean huskyJustUpdated = false;
//
//    private void updateControlHub() {
//        odometry[0] = parallelEncoder.getCurrentPosition();
//        odometry[1] = perpendicularEncoder.getCurrentPosition();
//
//        long currTime = System.currentTimeMillis();
//
//        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
//            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//            lastVoltageUpdatedTime = currTime;
//        }
//
//    }
//
//    private void updateExpansionHub() {
//        try {
//        } catch (Exception e) {
//            Log.e("******* Error due to ", e.getClass().getName());
//            e.printStackTrace();
//            Log.e("******* fail", "expansion hub failed");
//        }
//    }
//
//
//    public int[] getOdometry() {
//        return odometry;
//    }
//
//    public int getSlidesPos() {
//        return slidesEncoder;
//    }
//
//    public double getSlidesVelocity() {
//        return slidesVelocity;
//    }
//
//    public boolean isSlidesDown() {
//        return slidesDown;
//    }
//
//    public boolean isIntakeTriggered() {
//        return intakeTriggered;
//    }
//
//    public boolean isDepositTouched() {
//        return depositTouched;
//    }
//
//    public double getVoltage() {
//        return voltage;
//    }
//
//    public double getBackDist() {
//        return backUltrasonicDist;
//    }
//
//    public double getFrontDist() {
//        return frontUltrasonicDist;
//    }
//
//    public HuskyLens.Block[] getHuskyLensBlocks() {
//        return huskyLensBlocks;
//    }
//
////
////    public void updateDrivetrainMotorCurrents() {
////        leftFrontMotorCurrent = robot.drivetrain.leftFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
////        leftRearMotorCurrent = robot.drivetrain.leftRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
////        rightRearMotorCurrent = robot.drivetrain.rightRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
////        rightFrontMotorCurrent = robot.drivetrain.rightFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
////
////        TelemetryUtil.packet.put("leftFrontMotorCurrent", leftFrontMotorCurrent);
////        TelemetryUtil.packet.put("leftRearMotorCurrent", leftRearMotorCurrent);
////        TelemetryUtil.packet.put("rightRearMotorCurrent", rightRearMotorCurrent);
////        TelemetryUtil.packet.put("rightFrontMotorCurrent", rightFrontMotorCurrent);
////    }
//
//    private double previousAngle = 0.0;
//    private int numRotations = 0;
//
//    private void addToCumulativeHeading(double angle) {
//        if (Math.abs(angle - previousAngle) >= Math.toRadians(180)) {
//            numRotations += Math.signum(previousAngle);
//        }
//        previousAngle = angle;
//    }
//}