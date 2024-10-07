package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.backend.drivegvf.CubicBezierCurve;
import org.firstinspires.ftc.teamcode.util.geometry.Point;
import org.firstinspires.ftc.teamcode.util.geometry.Pose;
import org.firstinspires.ftc.teamcode.backend.drivepp.PurePursuitPath;
import org.firstinspires.ftc.teamcode.backend.drivepp.Waypoint;
import org.firstinspires.ftc.teamcode.util.geometry.Vector2D;

import java.util.LinkedList;
import java.util.List;

public class TelemetryDrawer {

    private final Canvas fieldOverlay;

    public TelemetryDrawer(TelemetryPacket packet) {
        this.fieldOverlay = packet.fieldOverlay();
    }

    public void drawPath(List<Point> path, String color) {
        fieldOverlay.setStroke(color);
        for (int i = 0; i < path.size() - 1; i++) {
            Point p1 = path.get(i);
            Point p2 = path.get(i + 1);
            fieldOverlay.strokeLine(p1.x, p1.y, p2.x, p2.y);
        }
    }

    public void drawPose(Pose pose, String color, double radius) {
        fieldOverlay.setStroke(color);
        fieldOverlay.strokeCircle(pose.x, pose.y, radius);
        fieldOverlay.strokeLine(pose.x, pose.y, pose.x + Math.cos(pose.heading) * radius, pose.y + Math.sin(pose.heading) * radius);
    }

    public void drawWaypoints(PurePursuitPath path, String color, double radius) {
        fieldOverlay.setStroke(color);
        for (Waypoint waypoint : path.getWaypoints()) {
            fieldOverlay.strokeCircle(waypoint.getPoint().x, waypoint.getPoint().y, radius);
        }
    }
    public void drawBezierCurve(CubicBezierCurve curve, String color) {
        fieldOverlay.setStroke(color);
        double radius = 2;
        LinkedList<Vector2D> points = curve.getControlPoints();
        for (int i = 0; i < points.size() - 1; i++) {
            fieldOverlay.strokeCircle(points.get(i).x, points.get(i).y, radius);
        }
        int SAMPLE_DENSITY = 50;
        for (int i = 0; i < SAMPLE_DENSITY; i++) {
            Vector2D vec0 = curve.calculate(i / (double) SAMPLE_DENSITY);
            Vector2D vec1 = curve.calculate((i + 1) / (double) SAMPLE_DENSITY);
            fieldOverlay.setStroke("#FF0000"); // Set color to red
            fieldOverlay.strokeLine(vec0.getX(), vec0.getY(), vec1.getX(), vec1.getY());
        }

    }
}
