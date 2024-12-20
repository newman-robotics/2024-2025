package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;

@Disabled
@Autonomous(name="AutoMain")
public class AutoMain extends LinearOpMode {
    /**
     * The main function.
     * **/
    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        RobotLog.i("Initialising...");

        Camera camera;
        try {
            RobotLog.i("Creating callback...");
            CameraHandler.CameraFrameCallback callback = new CameraHandler.CameraFrameCallback(CameraHandler::getLocationOnBoard);
            RobotLog.i("Creating camera...");
            camera = CameraHandler.createCamera(this.hardwareMap, GlobalConstants.CAMERA_X_SIZE, GlobalConstants.CAMERA_Y_SIZE, callback);
            if (camera == null) throw new RuntimeException("Failed to open camera (check logs for details)");
            RobotLog.i("Setting camera stream source...");
            CameraStreamServer.getInstance().setSource(callback.getCameraStreamSource());
        } catch (CameraException | AutoUtil.OpModeInterruptedException e) {
            throw new RuntimeException(e);
        }

        this.waitForStart();

        camera.close();
    }
}
