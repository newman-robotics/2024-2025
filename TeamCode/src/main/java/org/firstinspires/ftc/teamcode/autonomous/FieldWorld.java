package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents all objects on the field.
 * Very much TODO
 * **/
public class FieldWorld {
    public static class AxisAlignedBoundingBox {
        public double x1, y1, x2, y2;

        public AxisAlignedBoundingBox(double x1, double y1, double x2, double y2) {
            if (x1 >= x2 || y1 >= y2) throw new IllegalArgumentException("It is illegal to create an AABB with x1 >= x2 or y1 >= y2!");
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }

        public boolean isWithin(double x, double y) {
            return (this.x1 <= x && x <= this.x2) && (this.y1 <= y && y <= this.y2);
        }

        public boolean overlaps(AxisAlignedBoundingBox other) {
            return !(this.x2 < other.x1 || this.1 > other.x2 || this.y2 < other.y1 || this.y1 > other.y2);
        }

        public boolean willOverlapWithTranslation(double x, double y, AxisAlignedBoundingBox other) {
            return other.overlaps(new AxisAlignedBoundingBox(this.x1 + x, this.y1 + y, this.x2 + x, this.y2 + y));
        }

        public boolean intersectsLine(double x1, double y1, double x2, double y2) {
            return !((x1 > this.x2 && x2 > this.x2) ||
                    (x1 < this.x1 && x2 < this.x1) ||
                    (y1 > this.y2 && y2 > this.y2) ||
                    (y1 < this.y1 && y2 < this.y1));
        }

        public Pair<Double, Double> getCentrePoints() {
            return new Pair<>((x2 - x1) / 2. + x1, (y2 - y1) / 2. + y1);
        }
    }

    private final List<AxisAlignedBoundingBox> objects;
    private AxisAlignedBoundingBox robot;
    // The boundary of the playing field, but inset by 18 inches
    // (so that if the robot is outside of it, it can't move unless moving would put it back inside)
    private AxisAlignedBoundingBox internalBoundary;

    public FieldWorld() {
        this.objects = new ArrayList<AxisAlignedBoundingBox>();
    }

    public void addObject(AxisAlignedBoundingBox bbox) {
        this.objects.add(bbox);
    }

    public void setRobotPos(AxisAlignedBoundingBox newPos) {
        this.robot = newPos;
    }

    public void setBoundary(AxisAlignedBoundingBox boundary) {
        this.internalBoundary = new AxisAlignedBoundingBox(boundary.x1 - 9., boundary.y1 - 9., boundary.x2 - 9., boundary.y2 - 9.);
    }

    public void translateRobot(double x, double y) {
        this.robot.x1 += x;
        this.robot.x2 += x;
        this.robot.y1 += y;
        this.robot.y2 += y;
    }

    public boolean isColliding() {
        for (AxisAlignedBoundingBox object : this.objects) if (this.robot.overlaps(object)) return true;
        return false;
    }

    //[x, y] is measured from the centre of the robot (which we can assume is 18x18)
    public boolean willCollideAlongVector(double x, double y) {
        Pair<Double, Double> cp = this.robot.getCentrePoints();
        for (AxisAlignedBoundingBox object : this.objects) if (object.intersectsLine(cp.first, cp.second, cp.first + x, cp.second + y)) return true;
        return false;
    }

    //The robot should always be overlapping the internal boundary.
    public boolean willBeWithinBoundary(double x, double y) {
        return this.robot.willOverlapWithTranslation(x, y, this.internalBoundary);
    }
}
