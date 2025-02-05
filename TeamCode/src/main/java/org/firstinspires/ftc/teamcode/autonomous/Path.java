package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Path {
    //The odometry targets. We move to the point first and then rotate to the desired heading.
    private final List<PathTarget> odometryTargets;
    private final GoBildaPinpointDriver odometry;
    private int stage = 0;
    private int lastX = 0, lastY = 0;

    Path(GoBildaPinpointDriver odometry, List<PathTarget> odometryTargets) {
        this.odometry = odometry;
        this.odometryTargets = odometryTargets;
    }

    public boolean isDone() {
        return this.stage >= this.odometryTargets.size();
    }

    //definitely necessary
    private static double error(double input) {
        return Math.round(input * (1 << GlobalConstants.AUTONOMOUS_ACCURACY_BITS));
    }

    public void runNextStage(LinearOpMode parent) {
        if (this.isDone()) {
            RobotLog.i("done! (runNextStage)");
            return;
        }

        double headingTarget = Path.error(this.odometryTargets.get(this.stage).preAngle);
        double headingReading;

        do {
            if (parent.isStopRequested()) return;

            headingReading = Path.error(this.odometry.getHeading());

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.odometryTargets.size())
                    .add("Heading (degrees) ", Math.toDegrees(headingReading))
                    .add("Target heading (degrees) ", Math.toDegrees(headingTarget))
                    .update();

            RobotLog.i(String.format(Locale.UK, "[RAD] Cur: %f\t\t -> Tar: %f", headingReading, headingTarget));

            if (headingReading > headingTarget) AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, 0.2);
            else if (headingReading < headingTarget) AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, -0.2);
        } while (headingReading != headingTarget);

        double distanceTarget = this.odometryTargets.get(this.stage).distance;
        double distanceReading;
        int tempLastX, tempLastY;

        do {
            if (parent.isStopRequested()) return;

            int x = (int)DistanceUnit.INCH.fromMm(this.odometry.getPosX()) - this.lastX;
            int y = (int)DistanceUnit.INCH.fromMm(this.odometry.getPosY()) - this.lastY;
            distanceReading = x * x + y * y;

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.odometryTargets.size())
                    .add("Distance from last pos", distanceReading)
                    .add("Target distance from last pos", distanceTarget)
                    .update();

            RobotLog.i(String.format(Locale.UK, "[MMS] Cur: %f\t\t -> Tar: %f", distanceReading, distanceTarget));

            AutoUtil.Drivetrain.assertAndGet().setPowers(0., -0.2, 0.);

            tempLastX = x;
            tempLastY = y;
        } while (distanceReading < distanceTarget);

        this.lastX = tempLastX;
        this.lastY = tempLastY;

        headingTarget = Path.error(this.odometryTargets.get(this.stage).angle);

        do {
            if (parent.isStopRequested()) return;

            headingReading = Path.error(this.odometry.getHeading());

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.odometryTargets.size())
                    .add("Heading (radians) ", headingReading)
                    .add("Target heading (radians) ", headingTarget)
                    .update();

            RobotLog.i(String.format(Locale.UK, "[RAD] Cur: %f\t\t -> Tar: %f", headingReading, headingTarget));

            AutoUtil.Drivetrain.assertAndGet().setPowers(0, 0, headingReading > headingTarget ? 0.2 : (headingReading < headingTarget ? -0.2 : 0));
        } while (headingReading != headingTarget);

        ++this.stage;
    }

    public int getStage() {
        return this.stage;
    }

    private static class PathTarget {
        public final double preAngle;
        public final double distance;
        public final double angle;

        public PathTarget(double preAngle, double distance, double angle) {
            this.preAngle = preAngle;
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
            List<PathTarget> targets = new ArrayList<>();
            CameraHandler.FieldPos pos = this.positions.get(0);
            targets.add(new PathTarget(Math.atan2(this.positions.get(0).y, this.positions.get(0).x), pos.x * pos.x + pos.y * pos.y, pos.theta));

            for (int i = 0; i < this.positions.size() - 1; ++i) {
                pos = this.positions.get(i + 1);
                CameraHandler.FieldPos lastPos = this.positions.get(i);

                double rise = pos.y - lastPos.y;
                double run = pos.x - lastPos.x;
                double heading = Math.atan2(rise, run);

                targets.add(new PathTarget(heading, pos.x * pos.x + pos.y * pos.y, pos.theta));
            }

            return new Path(this.odometry, targets);
        }
    }
}
