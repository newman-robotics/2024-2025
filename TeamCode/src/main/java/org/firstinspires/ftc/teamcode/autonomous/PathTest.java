package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

@Autonomous
public class PathTest extends LinearOpMode {
    Path path;

    private void report() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.ChainTelemetry.init(this.telemetry);
        AutoUtil.Drivetrain.initAndGet(this.hardwareMap);

        GoBildaPinpointDriver odometry = this.hardwareMap.get(GoBildaPinpointDriver.class, GlobalConstants.ODOMETRY_NAME);
        odometry.resetPosAndIMU();

        this.path = new Path.Builder(odometry)
                .andThen(new CameraHandler.FieldPos(12, -48, Double.NaN))
                .andThen(new CameraHandler.FieldPos(108, -48, Double.NaN))
                .andThen(new CameraHandler.FieldPos(108, 48, Double.NaN))
                .andThen(new CameraHandler.FieldPos(12, 48, Double.NaN))
                .andThen(new CameraHandler.FieldPos(0, 0, 0))
                .build();

        this.waitForStart();

        while (!this.path.isDone()) {
            if (this.isStopRequested()) {
                RobotLog.i("prematurely stopped!");
                break;
            }
            this.path.runNextStage(this);
        }

        RobotLog.i("done! (runOpMode)");
    }
}
