package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

@Autonomous(name="PinpointTest")
public class PinpointTest extends LinearOpMode {
    public GoBildaPinpointDriver odometry;

    public void runOpMode() {
        this.odometry = this.hardwareMap.get(GoBildaPinpointDriver.class, GlobalConstants.ODOMETRY_NAME);
        this.odometry.setOffsets(GlobalConstants.ODOMETRY_X_OFFSET, GlobalConstants.ODOMETRY_Y_OFFSET);
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();

        this.waitForStart();

        while (this.opModeIsActive()) {
            this.odometry.update();
        }
    }
}
