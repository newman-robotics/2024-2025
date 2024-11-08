package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;

@Autonomous(name="AutoMain")
public class AutoMain extends LinearOpMode {
    public static int X_SIZE = 640;
    public static int Y_SIZE = 480;

    /**
     * The main function.
     * **/
    @Override
    public void runOpMode() throws InterruptedException {
        Camera camera;

        AutoUtil.setOpMode(this);

        try {
            CameraFrameCallback callback = new CameraFrameCallback((Mat) -> {}, X_SIZE, Y_SIZE);
            /*CameraCaptureSession.CaptureCallback callback = new CameraCaptureSession.CaptureCallback() {
                @Override
                public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                    RobotLog.i("received frame from camera!");
                }
            };*/
            camera = CameraHandler.createCamera(this.hardwareMap, X_SIZE, Y_SIZE, callback);
            if (camera == null) throw new RuntimeException("Failed to open camera (check logs for details)");
            CameraStreamServer.getInstance().setSource(callback.getCameraStreamSource());
        } catch (CameraException | AutoUtil.OpModeInterruptedException e) {
            throw new RuntimeException(e);
        }

        this.waitForStart();

        camera.close();
    }
}
