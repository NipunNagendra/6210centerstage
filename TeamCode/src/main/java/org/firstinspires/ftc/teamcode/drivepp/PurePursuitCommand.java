package org.firstinspires.ftc.teamcode.drivepp;

import static org.firstinspires.ftc.teamcode.drivepp.PurePursuitConstants.*;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localizers.TwoWheelIMULocalizerLegacy;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

@Config
public class PurePursuitCommand {
    private final Drivetrain drivetrain;
    private final TwoWheelIMULocalizerLegacy localizer;
    private final PurePursuitPath purePursuitPath;
    private final Pose endPose;

    private boolean PID = false;
    private boolean finished = false;

    public static BasicPID xController = new BasicPID(new PIDCoefficients(xP, 0.0, xD));
    public static BasicPID yController = new BasicPID(new PIDCoefficients(yP, 0.0, yD));
    public static BasicPID hController = new BasicPID(new PIDCoefficients(hP, 0.0, hD));

    private ElapsedTime accelLimit;
    private final double ACCEL_LIMIT = 0.5;

    private ElapsedTime timer;

    public PurePursuitCommand(PurePursuitPath purePursuitPath, HardwareMap hardwareMap) {
        this.drivetrain = new Drivetrain(hardwareMap);
        this.localizer = new TwoWheelIMULocalizerLegacy(hardwareMap);
        localizer.setPose(0,0,0);
        this.purePursuitPath = purePursuitPath;
        this.endPose = purePursuitPath.getEndPose();
    }

    public void execute() {
        if (accelLimit == null) accelLimit = new ElapsedTime();
        if (purePursuitPath.isFinished()) PID = true;

        localizer.update();
        Pose robotPose = localizer.getPose();
        Pose targetPose = purePursuitPath.update(robotPose);

        if (PID && timer == null) {
            timer = new ElapsedTime();
        }

        if (PID && targetPose.subt(robotPose).toVec2D().magnitude() < PurePursuitConstants.ALLOWED_TRANSLATIONAL_ERROR
                && Math.abs(targetPose.subt(robotPose).heading) < PurePursuitConstants.ALLOWED_HEADING_ERROR) finished = true;

        if (targetPose.heading - robotPose.heading > Math.PI) targetPose.heading -= 2 * Math.PI;
        if (targetPose.heading - robotPose.heading < -Math.PI) targetPose.heading += 2 * Math.PI;

        double xPower = xController.calculate(targetPose.x, robotPose.x);
        double yPower = yController.calculate(targetPose.y, robotPose.y);
        double hPower = -hController.calculate(targetPose.heading, robotPose.heading);

        double x_rotated = xPower * Math.cos(-robotPose.heading) - yPower * Math.sin(-robotPose.heading);
        double y_rotated = xPower * Math.sin(-robotPose.heading) + yPower * Math.cos(-robotPose.heading);

        hPower = Range.clip(hPower, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED);
        x_rotated = Range.clip(x_rotated, -MAX_TRANSLATIONAL_SPEED / X_GAIN, MAX_TRANSLATIONAL_SPEED / X_GAIN);
        y_rotated = Range.clip(y_rotated, -MAX_TRANSLATIONAL_SPEED, MAX_TRANSLATIONAL_SPEED);

        drivetrain.setFieldWeightedDrivePower(new Pose(x_rotated * X_GAIN, y_rotated, hPower).scale(Math.min(accelLimit.seconds() / ACCEL_LIMIT, 1)), 0);
    }

    public boolean isFinished() {
        return PID && finished || (timer != null && timer.milliseconds() > 2000);
    }

    public void end(boolean interrupted) {
        drivetrain.setFieldWeightedDrivePower(new Pose(), 0);
    }
}
