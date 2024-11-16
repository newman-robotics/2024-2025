package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.opencv.core.Mat;

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

        RobotLog.i("Initialising...");

        try {
            RobotLog.i("Creating callback...");
            CameraFrameCallback callback = new CameraFrameCallback((mat) -> {});
            RobotLog.i("Creating camera...");
            camera = CameraHandler.createCamera(this.hardwareMap, X_SIZE, Y_SIZE, callback);
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
