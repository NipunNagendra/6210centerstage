package org.firstinspires.ftc.teamcode.legacy;

import java.util.ArrayList;
import java.util.List;

public class pathBuilder {

    public static Waypoint[] injectWaypoints(Waypoint[] points, double spacing) {
        if (points.length < 2 || spacing <= 0) {
            return points;
        }

        List<Waypoint> newPoints = new ArrayList<>();
        for (int i = 0; i < points.length - 1; i++) {
            Waypoint start_point = points[i];
            Waypoint end_point = points[i + 1];
            newPoints.add(start_point);

            double vectorMagnitude = calculateVectorMagnitude(start_point, end_point);
            int numPointsThatFit = (int) Math.ceil(vectorMagnitude / spacing);

            double xIncrement = ((end_point.x - start_point.x) / vectorMagnitude) * spacing;
            double yIncrement = ((end_point.y - start_point.y) / vectorMagnitude) * spacing;

            for (int j = 1; j < numPointsThatFit; j++) {
                double newX = start_point.x + j * xIncrement;
                double newY = start_point.y + j * yIncrement;
                newPoints.add(new Waypoint(newX, newY));
            }
        }
        newPoints.add(points[points.length - 1]);
        return newPoints.toArray(new Waypoint[0]);
    }

    public static double calculateVectorMagnitude(Waypoint point1, Waypoint point2){
        return Math.sqrt(Math.pow(point2.x - point1.x, 2) + Math.pow(point2.y - point1.y, 2));
    }

    public static Waypoint[] pointSmoothing(Waypoint[] waypointArray, double weightData, double weightSmooth, double tolerance) {
        Waypoint[] smoothedWaypointArray = deepCopy(waypointArray);
        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < waypointArray.length - 1; i++) {
                Waypoint originalPoint = waypointArray[i];
                Waypoint smoothedPoint = smoothedWaypointArray[i];

                double xAux = smoothedPoint.x;
                double yAux = smoothedPoint.y;

                smoothedPoint.x += weightData * (originalPoint.x - smoothedPoint.x) +
                        weightSmooth * (smoothedWaypointArray[i - 1].x + smoothedWaypointArray[i + 1].x - 2.0 * smoothedPoint.x);
                smoothedPoint.y += weightData * (originalPoint.y - smoothedPoint.y) +
                        weightSmooth * (smoothedWaypointArray[i - 1].y + smoothedWaypointArray[i + 1].y - 2.0 * smoothedPoint.y);

                change += Math.abs(xAux - smoothedPoint.x) + Math.abs(yAux - smoothedPoint.y);
            }
        }
        return smoothedWaypointArray;
    }

    private static Waypoint[] deepCopy(Waypoint[] original) {
        Waypoint[] copy = new Waypoint[original.length];
        for (int i = 0; i < original.length; i++) {
            copy[i] = new Waypoint(original[i]);
        }
        return copy;
    }

    public static Waypoint[] processWaypoints(Waypoint[] original, double spacing, double weightData, double weightSmooth, double tolerance) {
        Waypoint[] injectedWaypoints = injectWaypoints(original, spacing);
        Waypoint[] smoothedWaypoints = pointSmoothing(injectedWaypoints, weightData, weightSmooth, tolerance);
        smoothedWaypoints[0].setEndpoint(true);
        smoothedWaypoints[smoothedWaypoints.length-1].setEndpoint(true);
        return smoothedWaypoints;
    }

}
