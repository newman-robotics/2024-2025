package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.List;

public class Path {
    //The headings to set the odometry to *before* moving to the target.
    private final List<Double> headings;
    //The odometry targets. We move to the point first and then rotate to the desired heading.
    private final List<PathTarget> odometryTargets;
    private final GoBildaPinpointDriver odometry;
    private int stage = 0;
    private int lastX = 0, lastY = 0;

    Path(GoBildaPinpointDriver odometry, List<Double> headings, List<PathTarget> odometryTargets) {
        this.odometry = odometry;
        this.headings = headings;
        this.odometryTargets = odometryTargets;
    }

    public boolean isDone() {
        return this.stage >= this.headings.size();
    }

    //maybe (probably) not necessary
    private static double error(double input) {
        return input;
        //return Math.round(input * (1 << GlobalConstants.AUTONOMOUS_ACCURACY_BITS));
    }

    public void runNextStage(LinearOpMode parent) {
        if (this.isDone()) {
            RobotLog.i("done! (runNextStage)");
            return;
        }

        double headingTarget = Path.error(this.headings.get(this.stage));
        double headingReading;

        do {
            if (parent.isStopRequested()) return;

            headingReading = Path.error(this.odometry.getHeading());

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.headings.size())
                    .add("Heading", headingReading)
                    .add("Target heading", headingTarget)
                    .update();

            if (headingReading > headingTarget) AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, 0.2);
            else if (headingReading < headingTarget) AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, -0.2);
        } while (headingReading != headingTarget);

        double distanceTarget = Path.error(this.odometryTargets.get(this.stage).angle);
        double distanceReading;
        int tempLastX, tempLastY;

        do {
            if (parent.isStopRequested()) return;

            int x = (int)DistanceUnit.INCH.fromMm(this.odometry.getPosX()) - this.lastX;
            int y = (int)DistanceUnit.INCH.fromMm(this.odometry.getPosY()) - this.lastY;
            distanceReading = Path.error(x * x + y * y);

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.headings.size())
                    .add("Distance from last pos", distanceReading)
                    .add("Target distance from last pos", distanceTarget)
                    .update();

            AutoUtil.Drivetrain.assertAndGet().setPowers(0., -0.2, 0.);

            tempLastX = x;
            tempLastY = y;
        } while (distanceReading != distanceTarget);

        this.lastX = tempLastX;
        this.lastY = tempLastY;

        headingTarget = Path.error(this.odometryTargets.get(this.stage).angle);

        do {
            if (parent.isStopRequested()) return;

            headingReading = Path.error(this.odometry.getHeading());

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.headings.size())
                    .add("Heading", headingReading)
                    .add("Target heading", headingTarget)
                    .update();

            AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, headingReading > headingTarget ? 0.2 : (headingReading < headingTarget ? -0.2 : 0));
        } while (headingReading != headingTarget);

        double theta = this.odometryTargets.get(this.stage).angle;
        if (!Double.isNaN(theta)) {
            headingTarget = Math.round(theta * Math.pow(10, GlobalConstants.AUTONOMOUS_ACCURACY_BITS));

            do {
                if (parent.isStopRequested()) return;

                headingReading = Math.round(this.odometry.getHeading() * Math.pow(10, GlobalConstants.AUTONOMOUS_ACCURACY_BITS));

                this.odometry.update();

                AutoUtil.ChainTelemetry.assertAndGet()
                        .add("Stage", this.getStage())
                        .add("Total stages", this.headings.size())
                        .add("Heading", headingReading)
                        .add("Target heading", headingTarget)
                        .update();

                if (headingReading > headingTarget)
                    AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, 0.2);
                else if (headingReading < headingTarget)
                    AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, -0.2);

                //RobotLog.i("{Stage=" + this.getStage() + ", TotalStages=" + this.headings.size() + ", Heading=" + this.odometry.getHeading() + ", TargetHeading=" + this.headings.get(this.stage) + "}");
            } while (headingReading != headingTarget);
        }

        ++this.stage;
    }

    public int getStage() {
        return this.stage;
    }

    private static class PathTarget {
        public final double distance;
        public final double angle;

        public PathTarget(double distance, double angle) {
            this.distance = distance;
            this.angle = angle;
        }
    }

    public static class Builder {
        private final List<CameraHandler.FieldPos> positions;
        private final GoBildaPinpointDriver odometry;

        public Builder(GoBildaPinpointDriver odometry) {
            this.positions = new ArrayList<>();
            this.odometry = odometry;
        }

        /**
         * Target is in coordinates relative to where the odometer starts.
         * +X is forwards, +Y is right, theta is in radians
         * If theta is Double.NaN, it is ignored
         * **/
        public Builder andThen(CameraHandler.FieldPos target) {
            this.positions.add(target);
            return this;
        }

        public Path build() {
            List<Double> headings = new ArrayList<>();

            headings.add(Math.atan2(this.positions.get(0).y, this.positions.get(0).x));
            RobotLog.i("added first heading " + headings.get(0));

            for (int i = 0; i < this.positions.size() - 1; ++i) {
                double rise = this.positions.get(i + 1).y - this.positions.get(i).y;
                double run = this.positions.get(i + 1).x - this.positions.get(i).x;
                double heading = Math.atan2(rise, run);
                headings.add(heading);

                RobotLog.i("added heading " + heading);
            }

            List<PathTarget> targets = new ArrayList<>();
            for (CameraHandler.FieldPos pos : this.positions) {
                targets.add(new PathTarget(pos.x * pos.x + pos.y * pos.y, pos.theta));
            }

            return new Path(this.odometry, headings, targets);
        }
    }
}
