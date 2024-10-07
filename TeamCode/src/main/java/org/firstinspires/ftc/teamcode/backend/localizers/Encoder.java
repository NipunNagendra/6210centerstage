package org.firstinspires.ftc.teamcode.backend.localizers;

import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {
    private final static int CPS_STEP = 0x10000;
    public double ticksToInches;
    public int lastVal;
    public int currentVal;
    public double scaleFactor;
    public double x;
    public double y;

    private DcMotorEx motor;
    private NanoClock clock;

    private Direction direction;

    private int lastPosition;
    private int velocityEstimateIdx;
    private double[] velocityEstimates;
    private double lastUpdateTime;

    public Encoder(Pose point, double scaleFactor, DcMotorEx motor) {
        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.984252; // for 50mm wheels
        ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;
        x = point.getX();
        y = point.getY();
        currentVal = 0;
        lastVal = currentVal;
        this.scaleFactor = scaleFactor;
        this.motor = motor;
        this.clock = NanoClock.system();

        this.direction = Direction.FORWARD;
        this.lastPosition = 0;
        this.velocityEstimates = new double[3];
        this.lastUpdateTime = clock.seconds();
    }

    public void update(int currentPos) {
        lastVal = currentVal;
        currentVal = currentPos;
    }

    public double getDelta() {
        return (double)(currentVal - lastVal) * ticksToInches * scaleFactor;
    }

    public double getCurrentDist() {
        return (double)(currentVal) * ticksToInches * scaleFactor;
    }

    // EncoderSensor logic
    private static double inverseOverflow(double input, double estimate) {
        int real = (int) input & 0xffff;
        real += ((real % 20) / 4) * CPS_STEP;
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    private int getMultiplier() {
        return getDirection().getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    public double getRawVelocity() {
        int multiplier = getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
