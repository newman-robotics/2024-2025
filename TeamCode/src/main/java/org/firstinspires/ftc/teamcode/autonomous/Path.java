package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.util.ArrayList;
import java.util.List;

public class Path {
    //The headings to set the odometry to *before* moving to the target.
    private List<Double> headings;
    //The odometry targets. We move to the point first and then rotate to the desired heading.
    private List<CameraHandler.FieldPos> odometryTargets;
    private GoBildaPinpointDriver odometry;
    private int stage;
    private AutoUtil.Drivetrain drivetrain;

    public Path(AutoUtil.Drivetrain drivetrain, GoBildaPinpointDriver odometry, List<Double> headings, List<CameraHandler.FieldPos> odometryTargets) {
        this.odometry = odometry;
        this.headings = headings;
        this.odometryTargets = odometryTargets;
        this.stage = 0;
    }

    public void runNextStage() {
        while (this.odometry.getHeading() != this.headings.get(this.stage)) {

        }

        ++this.stage;
    }

    public static class Builder {
        private List<CameraHandler.FieldPos> positions;
        private GoBildaPinpointDriver odometry;
        private AutoUtil.Drivetrain drivetrain;

        public Builder(AutoUtil.Drivetrain drivetrain, GoBildaPinpointDriver odometry) {
            this.positions = new ArrayList<>();
            this.odometry = odometry;
            this.drivetrain = drivetrain;
        }

        public Builder andThen(CameraHandler.FieldPos target) {
            this.positions.add(target);
            return this;
        }

        public Path build() {


            return new Path(this.drivetrain, this.odometry, null, null);
        }
    }
}
