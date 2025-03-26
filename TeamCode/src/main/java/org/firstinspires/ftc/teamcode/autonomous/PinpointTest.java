package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.util.concurrent.atomic.AtomicReference;

@Disabled
@Autonomous(name="PinpointTest")
public class PinpointTest extends LinearOpMode {
    public static GoBildaPinpointDriver odometry;
    public static AtomicReference<CameraHandler.FieldPos> lastPosition = new AtomicReference<>(null);

    private static void runProperOpMode() {
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

    /**
     * This is an ugly solution, but the poor programmers always have too much demanded of them...
     * **/
    public static void internalRunOpMode(boolean allowFallback) {
        LinearOpMode thiz = AutoUtil.getOpMode();

        PinpointTest.odometry = thiz.hardwareMap.get(GoBildaPinpointDriver.class, GlobalConstants.ODOMETRY_NAME);
        PinpointTest.odometry.setOffsets(GlobalConstants.ODOMETRY_X_OFFSET, GlobalConstants.ODOMETRY_Y_OFFSET);
        PinpointTest.odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        PinpointTest.odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        PinpointTest.odometry.resetPosAndIMU();

        CameraHandler.CameraFrameCallback callback;
        Camera camera;

        try {
            callback = new CameraHandler.CameraFrameCallback((frame) -> PinpointTest.lastPosition.set(CameraHandler.getLocationOnBoard(frame)));
            camera = CameraHandler.createCamera(thiz.hardwareMap, GlobalConstants.CAMERA_X_SIZE, GlobalConstants.CAMERA_Y_SIZE, callback);
            if (camera == null) throw new RuntimeException("Camera not created (check logs for details)");
            CameraStreamServer.getInstance().setSource(callback.getCameraStreamSource());
        } catch (CameraException | AutoUtil.OpModeInterruptedException e) {
            throw new RuntimeException(e);
        }

        while (PinpointTest.lastPosition.get() == null) if (!thiz.opModeInInit()) {
            camera.close();
            RobotLog.e("Interrupted while waiting for camera frame pos!");
            break;
        }

        AutoUtil.ChainTelemetry.assertAndGet().add("Robot position finalised!");

        boolean isFallback;

        if (PinpointTest.lastPosition.get() != null) {
            CameraHandler.FieldPos lastPositionCurrentValue = PinpointTest.lastPosition.get();
            PinpointTest.odometry.setPosition(new GoBildaPinpointDriver.Pose2D(DistanceUnit.INCH, lastPositionCurrentValue.x, lastPositionCurrentValue.y, AngleUnit.RADIANS, lastPositionCurrentValue.theta));
            isFallback = false;
        } else {
            if (!allowFallback) throw new RuntimeException(new AutoUtil.OpModeInterruptedException("OpMode interrupted in init!"));
            PinpointTest.odometry.setPosition(new GoBildaPinpointDriver.Pose2D(DistanceUnit.INCH, GlobalConstants.SIMPLE_STARTING_POS_X, GlobalConstants.SIMPLE_STARTING_POS_Y, AngleUnit.RADIANS, 0));
            isFallback = true;
        }

        callback.terminate();
        camera.close();

        thiz.waitForStart();

        if (isFallback) DepressingAutonomous.runFallbackOpMode();
        else PinpointTest.runProperOpMode();
    }

    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        PinpointTest.internalRunOpMode(false);
    }
}
