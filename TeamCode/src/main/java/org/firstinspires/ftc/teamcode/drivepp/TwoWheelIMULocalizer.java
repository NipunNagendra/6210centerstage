package org.firstinspires.ftc.teamcode.drivepp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drivepp.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class TwoWheelIMULocalizer {
    private Encoder parallelEncoder, perpendicularEncoder;
    private IMU imu;
    private Pose pose = new Pose(0, 0, 0);

    private static final double WHEEL_RADIUS = .68897638; // in inches
    private static final double TICKS_PER_REV = 8192;
    private static final double GEAR_RATIO = 1.0; // output/input
    private static final double PARALLEL_OFFSET = -1.73; // in inches, distance of the parallel odometry pod from the center
    private static final double PERPENDICULAR_OFFSET = 0.5; // in inches, distance of the perpendicular odometry pod from the center

    private int lastParallelEncoderPos = 0;
    private int lastPerpendicularEncoderPos = 0;

    double offset = 0;

    public TwoWheelIMULocalizer(HardwareMap hardwareMap) {
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BR"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        parallelEncoder.resetEncoder();
        perpendicularEncoder.resetEncoder();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
        imu.resetYaw();

        this.pose.x = PARALLEL_OFFSET;
        this.pose.y = PERPENDICULAR_OFFSET;
    }
    public void update() {
        // Read the current encoder positions
        int parallelEncoderPos = parallelEncoder.getCurrentPosition();
        int perpendicularEncoderPos = perpendicularEncoder.getCurrentPosition();

        // Read the IMU heading
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + offset;
        heading = normalizeAngle(heading);
        // Calculate the change in encoder positions
        int deltaParallel = parallelEncoderPos - lastParallelEncoderPos;
        int deltaPerpendicular = perpendicularEncoderPos - lastPerpendicularEncoderPos;

        // Update the last encoder positions
        lastParallelEncoderPos = parallelEncoderPos;
        lastPerpendicularEncoderPos = perpendicularEncoderPos;

        // Calculate the distance traveled by each pod
        double distanceParallel = encoderTicksToInches(deltaParallel);
        double distancePerpendicular = encoderTicksToInches(deltaPerpendicular);

        // Calculate the robot's displacement
        double deltaY = distanceParallel * Math.cos(heading) + distancePerpendicular * Math.sin(heading);
        double deltaX = -distanceParallel * Math.sin(heading) + distancePerpendicular * Math.cos(heading);

        // Update the pose
        pose = new Pose(
                pose.x + deltaX,
                pose.y + deltaY,
                heading
        );
    }

    private double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    private double normalizeAngle(double angle) {
        return AngleUnit.normalizeRadians(angle);
    }

    public Pose getPose() {
        return pose;
    }

    public int getParallelEncoderPos() {
        return parallelEncoder.getCurrentPosition();
    }

    public int getPerpendicularEncoderPos() {
        return perpendicularEncoder.getCurrentPosition();
    }
    public void setPose(Pose pose) {
        this.pose = pose;
    }

    // Method to set the starting position using individual coordinates and heading
    public void setPose(double x, double y, double heading) {
        this.pose = new Pose(x+PARALLEL_OFFSET, y+PERPENDICULAR_OFFSET, heading);
        offset=heading;
    }
}