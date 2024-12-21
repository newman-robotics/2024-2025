package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "DepressingAutonomous")
public class DepressingAutonomous extends LinearOpMode {
    /**
     * Runs the OpMode if the camera failed to pick up the robot's location.
     * **/
    public static void runFallbackOpMode() {
        LinearOpMode thiz = AutoUtil.getOpMode();

        while (thiz.opModeIsActive()) {
            PinpointTest.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Data in inches and degrees.")
                    .add("Odometer X position", PinpointTest.odometry.getPosition().getX(DistanceUnit.INCH))
                    .add("Odometer Z position", PinpointTest.odometry.getPosition().getY(DistanceUnit.INCH))
                    .add("Odometer angle", PinpointTest.odometry.getPosition().getHeading(AngleUnit.DEGREES))
                    .update();
        }
    }

    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        PinpointTest.internalRunOpMode(true);
    }
}
