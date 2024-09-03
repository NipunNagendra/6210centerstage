package org.firstinspires.ftc.teamcode.drivepp;


import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;

import java.util.Collections;
import java.util.LinkedList;

public class PurePursuitPath {
    private final LinkedList<Waypoint> waypoints = new LinkedList<>();
    private int targetIdx = 1;
    private boolean finished;

    public PurePursuitPath(Waypoint... ws) {
        if (ws.length < 2) throw new IllegalArgumentException();
        Collections.addAll(waypoints, ws);
        if(waypoints.getLast().getType() != Waypoint.Type.POSE) throw new IllegalArgumentException();
    }

    public Pose update(Pose robot) {
        Waypoint prev = waypoints.get(targetIdx - 1);
        Waypoint target = waypoints.get(targetIdx);

        double distance = robot.distanceTo(target.getPoint());

        if(distance > target.getRadius()){
            Point intersection = PurePursuitUtil.lineCircleIntersection(
                    prev.getPoint(), target.getPoint(), robot, target.getRadius());
            Pose targetPose;

            if(target.getType() == Waypoint.Type.POSE){
                targetPose = new Pose(intersection, ((Pose)target.getPoint()).heading);
            }else{
                double robotAngle = AngleUnit.normalizeRadians(robot.heading);

                Vector2D intersectionVector = new Vector2D(intersection.x - robot.x, intersection.y - robot.y);
                double forwardAngle = intersectionVector.atan(); // This is the target angle
                double backwardAngle = AngleUnit.normalizeRadians(forwardAngle + Math.PI);

                double angleToForward = AngleUnit.normalizeRadians(forwardAngle - robotAngle);
                double angleToBackward = AngleUnit.normalizeRadians(backwardAngle - robotAngle);

                double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
                autoAngle = AngleUnit.normalizeRadians(autoAngle);
                targetPose = new Pose(intersection, autoAngle);
            }

            return targetPose;
        }else{
            if(targetIdx == waypoints.size() - 1){
                finished = true;
                return getEndPose();
            }else{
                targetIdx++;
                return update(robot);
            }
        }
    }

    public boolean isFinished(){
        return finished;
    }

    public Pose getEndPose(){
        return (Pose) waypoints.getLast().getPoint();
    }

    public double getRadius(){
        return waypoints.get(targetIdx).getRadius();
    }
    public LinkedList<Waypoint> getWaypoints(){
        return waypoints;
    }
}