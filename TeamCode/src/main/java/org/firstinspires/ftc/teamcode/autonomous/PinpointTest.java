package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.lang.reflect.Field;
import java.util.concurrent.atomic.AtomicReference;

@Autonomous(name="PinpointTest")
public class PinpointTest extends LinearOpMode {
    public GoBildaPinpointDriver odometry;
    public static AtomicReference<CameraHandler.FieldPos> lastPosition = new AtomicReference<>(null);

    public void runOpMode() {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        this.odometry = this.hardwareMap.get(GoBildaPinpointDriver.class, GlobalConstants.ODOMETRY_NAME);
        this.odometry.setOffsets(GlobalConstants.ODOMETRY_X_OFFSET, GlobalConstants.ODOMETRY_Y_OFFSET);
        this.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odometry.resetPosAndIMU();

        CameraHandler.CameraFrameCallback callback;
        Camera camera;

        try {
            callback = new CameraHandler.CameraFrameCallback((frame) -> PinpointTest.lastPosition.set(CameraHandler.getLocationOnBoard(frame)));
            camera = CameraHandler.createCamera(this.hardwareMap, GlobalConstants.CAMERA_X_SIZE, GlobalConstants.CAMERA_Y_SIZE, callback);
            if (camera == null) throw new RuntimeException("Camera not created (check logs for details)");
            CameraStreamServer.getInstance().setSource(callback.getCameraStreamSource());
        } catch (CameraException | AutoUtil.OpModeInterruptedException e) {
            throw new RuntimeException(e);
        }

        while (PinpointTest.lastPosition.get() == null) if (!this.opModeInInit()) {
            camera.close();
            throw new RuntimeException(new AutoUtil.OpModeInterruptedException("Interrupted while waiting for camera frame pos!"));
        }

        CameraHandler.FieldPos lastPositionCurrentValue = PinpointTest.lastPosition.get();
        this.odometry.setPosition(new GoBildaPinpointDriver.Pose2D(DistanceUnit.INCH, lastPositionCurrentValue.x, lastPositionCurrentValue.y, AngleUnit.RADIANS, lastPositionCurrentValue.theta));

        callback.terminate();
        camera.close();

        this.waitForStart();

        while (this.opModeIsActive()) {
            this.odometry.update();

            AutoUtil.ChainTelemetry.assertAndGet()
                    .add("Data in inches and degrees.")
                    .add("Odometer X position", this.odometry.getPosition().getX(DistanceUnit.INCH))
                    .add("Odometer Z position", this.odometry.getPosition().getY(DistanceUnit.INCH))
                    .add("Odometer angle", this.odometry.getPosition().getHeading(AngleUnit.DEGREES))
                    .update();
        }
    }
}
