package org.firstinspires.ftc.teamcode.backend.localizers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.rrutil.EncoderSensor;


public class TwoWheelLocalizer extends Localizer {
    private EncoderSensor parallelEncoder, perpendicularEncoder;
    private IMU imu;
    private double lastHeading;

    public TwoWheelLocalizer(HardwareMap hardwareMap) {
        super(hardwareMap);

        encoders[0] = new Encoder(new Pose(0,7.5),  -1, hardwareMap.get(DcMotorEx.class, "FL")); // parr
        encoders[1] = new Encoder(new Pose(-8, 0),  -1, hardwareMap.get(DcMotorEx.class, "BL")); // perp (7.1660442092285175)

        encoders[0].resetEncoder();
        encoders[1].resetEncoder();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
        imu.resetYaw();

    }

    @Override
    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime-lastTime)/1.0E9;
        lastTime = currentTime;

        // Odometry
        double deltaRight = encoders[0].getDelta();
        double deltaBack = encoders[1].getDelta();
        double rightY = encoders[0].y;
        double backX = encoders[1].x;

        double deltaHeading = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - heading);
        relDeltaY = deltaBack - deltaHeading*backX;
        relDeltaX = deltaRight + deltaHeading*rightY;
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        // constant accel
        Pose relDelta = new Pose(relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime,relDelta,currentPose);

        x = currentPose.x;
        y = currentPose.y;

        heading = currentPose.heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
    }
}