package org.firstinspires.ftc.teamcode.backend.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.util.geometry.Pose;



import java.util.ArrayList;

public class Localizer {

    public Encoder[] encoders;
    protected long lastTime = System.nanoTime();

    public double x = 0;
    public double y = 0;
    public double heading = 0;

    protected double odoHeading = 0;

    public Pose expected = new Pose(0, 0, 0);
    public Pose currentPose = new Pose(0,0,0);
    public Pose currentVel = new Pose(0,0,0);
    public Pose relCurrentVel = new Pose(0,0,0);
    public Pose relCurrentAcc = new Pose(0,0,0);
    public Pose currentPowerVector = new Pose(0,0,0);

    protected ConstantAccelMath constAccelMath = new ConstantAccelMath();

    protected ArrayList<Pose> poseHistory = new ArrayList<Pose>();
    protected ArrayList<Pose> relHistory = new ArrayList<Pose>();
    protected ArrayList<Long> nanoTimes = new ArrayList<Long>();

    protected Pose aprilTagPose = new Pose(0,0,0);

    protected double maxVel = 0.0;
    protected double startHeadingOffset = 0;
    protected String color;
    protected String expectedColor;

    public Localizer(HardwareMap hardwareMap) {

        encoders = new Encoder[2];


        relHistory.add(new Pose(0,0,0));
        poseHistory.add(new Pose(0,0,0));
        nanoTimes.add(Long.valueOf(0));
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++){
            this.encoders[i].update(encoders[i]);
        }
    }
    double headingOffset = 0;
    public void setPose(double x, double y, double h) {
        this.x = x;
        this.y = y;
        headingOffset += h - this.heading;
        currentPose = new Pose(x, y, h);
    }

    public Pose getPoseEstimate() {
        return new Pose(currentPose.x, currentPose.y, currentPose.heading);
    }


    public void setPoseEstimate(Pose Pose) {
        setPose(Pose.getX(), Pose.getY(), Pose.getHeading());
        startHeadingOffset = Pose.getHeading();
    }

    public Pose getRelativePoseVelocity() {
        return new Pose(relCurrentVel.x, relCurrentVel.y, relCurrentVel.heading);
    }

    double fidelity = 1E-8;

    public double relDeltaX;
    public double relDeltaY;
    public double distanceTraveled = 0;

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (double)(currentTime-lastTime)/1.0E9;
        lastTime = currentTime;

        // Odometry
        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double deltaBack = encoders[2].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;
        double backX = encoders[2].x;

        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        relDeltaY = deltaBack - deltaHeading*backX;
        relDeltaX = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);
        distanceTraveled += Math.sqrt(relDeltaX*relDeltaX+relDeltaY*relDeltaY);

        // constant accel
        Pose relDelta = new Pose(relDeltaX,relDeltaY,deltaHeading);
        constAccelMath.calculate(loopTime,relDelta,currentPose);

        x = currentPose.x;
        y = currentPose.y;
        heading = currentPose.heading;

        relHistory.add(0,relDelta);
        nanoTimes.add(0, currentTime);
        poseHistory.add(0,currentPose.clone());

        updateVelocity();
        updateExpected();
    }

    public Pose interpolatedPastPose;

    public void findPastInterpolatedPose(long poseTime) {
        int indexOfDesiredNanoTime = 0;

        for (long time : nanoTimes) {
            if (time > poseTime) {
                indexOfDesiredNanoTime++;
            } else {
                break;
            }
        }

        indexOfDesiredNanoTime = Math.min(indexOfDesiredNanoTime, nanoTimes.size()-1);

        Pose pastTimeRobotPose = poseHistory.get(indexOfDesiredNanoTime).clone();
        Pose pastTimeRobotPose2 = poseHistory.get(Math.max(0, indexOfDesiredNanoTime-1)).clone();

        if (indexOfDesiredNanoTime != 0) {
            Pose errorInPastPoses = new Pose(
                    pastTimeRobotPose2.x - pastTimeRobotPose.x,
                    pastTimeRobotPose2.y - pastTimeRobotPose.y,
                    headingClip(pastTimeRobotPose2.heading - pastTimeRobotPose.heading)
            );

            double timeWeight = (double) (poseTime - nanoTimes.get(indexOfDesiredNanoTime)) /
                    (double) (nanoTimes.get(Math.max(0, indexOfDesiredNanoTime - 1)) - nanoTimes.get(indexOfDesiredNanoTime));

            interpolatedPastPose = new Pose(
                    pastTimeRobotPose.x + errorInPastPoses.x * timeWeight,
                    pastTimeRobotPose.y + errorInPastPoses.y * timeWeight,
                    pastTimeRobotPose.heading + errorInPastPoses.heading * timeWeight
            );
        } else {
            interpolatedPastPose = pastTimeRobotPose;
        }
    }

    double headingDif = 0.0;
    boolean firstLoop = true;
    double lastImuHeading;
    double lastOdoHeading;

    double imuMerge = 0;

    public void updateHeadingWithIMU(double imuHeading) {
    }

    public void updatePowerVector(double[] p){
        for (int i = 0; i < p.length; i ++){
            p[i] = Math.max(Math.min(p[i],1),-1);
        }
        double forward = (p[0] + p[1] + p[2] + p[3]) / 4;
        double left = (-p[0] + p[1] - p[2] + p[3]) / 4; //left power is less than 1 of forward power
        double turn = (-p[0] - p[1] + p[2] + p[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2;
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;
        long start = nanoTimes.size() != 0 ? nanoTimes.get(0) : 0;
        for (int i = 0; i < nanoTimes.size(); i++){
            totalTime = (double)(start - nanoTimes.get(i)) / 1.0E9;
            if (totalTime <= targetVelTimeEstimate){
                actualVelTime = totalTime;
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            }
        }
        if (actualVelTime != 0) {
            relCurrentVel = new Pose(
                    (relDeltaXTotal) / actualVelTime,
                    (relDeltaYTotal) / actualVelTime,
                    (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime
            );

            currentVel = new Pose(
                    relCurrentVel.x*Math.cos(heading) - relCurrentVel.y*Math.sin(heading),
                    relCurrentVel.x*Math.sin(heading) + relCurrentVel.y*Math.cos(heading),
                    relCurrentVel.heading
            );
        }
        else {
            currentVel = new Pose(0, 0, 0);
            relCurrentVel = new Pose(0, 0, 0);
        }
        while (lastIndex + 1 < nanoTimes.size()){
            nanoTimes.remove(nanoTimes.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }

    double a = 0.0045;
    double b = -0.0728;
    double c = 0.8062;
    double d = Math.sqrt(c/a);

    protected void updateExpected() {
        double totalVel = Math.sqrt(Math.pow(currentVel.x, 2) + Math.pow(currentVel.y, 2));
        double distance = getExpectedDistance(totalVel);

        if (totalVel <= d) {
            distance = totalVel*(getExpectedDistance(d)/d);
        }

        if (totalVel <= 5) {
            expected.x = x;
            expected.y = y;
            return;
        }

        expected.x = x + distance * currentVel.x/totalVel;
        expected.y = y + distance * currentVel.y/totalVel;
        expected.heading = heading;
    }

    private double getExpectedDistance (double x) {
        return a*Math.pow(x,2) + b*x + c;
    }

    protected MovingAverage movingAverage = new MovingAverage(100);

    public double combineHeadings(double headingError) {
        movingAverage.addData(headingError);
        double averageError = movingAverage.getMovingAverageForNum();
        movingAverage.updateValsRetroactively(averageError);
        return movingAverage.getMovingAverageForNum();
    }
    public static double headingClip(double value) {
        while(value >= Math.PI) {
            value -= 2*Math.PI;
        }
        while(value <= -Math.PI) {
            value += 2*Math.PI;
        }
        return value;
    }
    public Pose getExpectedPose() {
        return new Pose(expected.x, expected.y, expected.heading);
    }
}