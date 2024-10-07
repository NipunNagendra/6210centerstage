package org.firstinspires.ftc.teamcode.backend.localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;
import org.firstinspires.ftc.teamcode.backend.optimizations.RingBuffer;
import org.firstinspires.ftc.teamcode.util.rrutil.EncoderSensor;

public class TwoWheelIMULocalizerLegacy {
    private EncoderSensor parallelEncoder, perpendicularEncoder;
    private IMU imu;
    private Pose pose = new Pose(0, 0, 0);

    private static final double WHEEL_RADIUS = 0.94482; // in inches
    private static final double TICKS_PER_REV = 2000;
    private static final double GEAR_RATIO = 1.0; // output/input
    private static final double PARALLEL_OFFSET = 0; // in inches, distance of the parallel odometry pod from the center
    private static final double PERPENDICULAR_OFFSET = 0; // in inches, distance of the perpendicular odometry pod from the center

    private int lastParallelEncoderPos = 0;
    private int lastPerpendicularEncoderPos = 0;

    private ElapsedTime elapsedTime;
    private Vector2D velocity;

    double offset = 0;

    private RingBuffer poseHistory;

    public TwoWheelIMULocalizerLegacy(HardwareMap hardwareMap) {
        parallelEncoder = new EncoderSensor(hardwareMap.get(DcMotorEx.class, "FL"));
        perpendicularEncoder = new EncoderSensor(hardwareMap.get(DcMotorEx.class, "BR"));
        parallelEncoder.setDirection(EncoderSensor.Direction.REVERSE);
        perpendicularEncoder.setDirection(EncoderSensor.Direction.REVERSE);
        parallelEncoder.resetEncoder();
        perpendicularEncoder.resetEncoder();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
        imu.resetYaw();

        this.pose.x = PARALLEL_OFFSET;
        this.pose.y = PERPENDICULAR_OFFSET;

        elapsedTime = new ElapsedTime();
        this.velocity = new Vector2D(0,0);

        this.poseHistory = new RingBuffer(3);
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
        velocity = calculateRobotRelativeVelocity(distancePerpendicular, distanceParallel);

        // Update the pose
        pose = new Pose(
                pose.x + deltaX,
                pose.y + deltaY,
                heading
        );

        poseHistory.put(pose);
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
//TODO: CHANGE TO HEADING VELOCITY!!!!
    public Vector2D getVelocity() {
        return velocity;
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

    public void setPose(double x, double y, double heading) {
        this.pose = new Pose(x+PARALLEL_OFFSET, y+PERPENDICULAR_OFFSET, heading);
        offset=heading;
    }

    private Vector2D calculateRobotRelativeVelocity(double deltaX, double deltaY){ {
        double dt = elapsedTime.seconds();
        elapsedTime.reset();
        return new Vector2D(deltaX / dt, deltaY / dt);
    }
    }

    public Pose readPoseHistory(int index) {
        return poseHistory.read(index);
}

    public Pose[] getPoseHistory() {
        return poseHistory.getOldestToLatest();
    }


}