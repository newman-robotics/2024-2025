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
    private final List<CameraHandler.FieldPos> odometryTargets;
    private final GoBildaPinpointDriver odometry;
    private int stage = 0;

    Path(GoBildaPinpointDriver odometry, List<Double> headings, List<CameraHandler.FieldPos> odometryTargets) {
        this.odometry = odometry;
        this.headings = headings;
        this.odometryTargets = odometryTargets;
    }

    public boolean isDone() {
        return this.stage >= this.headings.size();
    }

    private static double error(double input) {
        return Math.round(input * (1 << GlobalConstants.AUTONOMOUS_ACCURACY_BITS));
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

        double xTarget = Path.error(this.odometryTargets.get(this.stage).x);
        double yTarget = Path.error(this.odometryTargets.get(this.stage).y);
        double xReading, yReading;

        do {
            if (parent.isStopRequested()) return;

            xReading = Path.error(DistanceUnit.INCH.fromMm(this.odometry.getPosX()));
            yReading = Path.error(DistanceUnit.INCH.fromMm(this.odometry.getPosY()));

            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Stage", this.getStage())
                    .add("Total stages", this.headings.size())
                    .add("X position", xReading)
                    .add("Y position", yReading)
                    .add("Target X position", xTarget)
                    .add("Target Y position", yTarget)
                    .update();

            AutoUtil.Drivetrain.assertAndGet().setPowers(0., -0.2, 0.);
        } while (xReading != xTarget || yReading != yTarget);

        headingTarget = Path.error(this.odometryTargets.get(this.stage).theta);

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

        ++this.stage;
    }

    public int getStage() {
        return this.stage;
    }

    public static class Builder {
        private final List<CameraHandler.FieldPos> positions;
        private final GoBildaPinpointDriver odometry;

        public Builder(GoBildaPinpointDriver odometry) {
            this.positions = new ArrayList<>();
            this.odometry = odometry;
        }

        /**
         * Target is in coordinates relative to where the
         * odometer starts
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

            return new Path(this.odometry, headings, this.positions);
        }
    }
}
