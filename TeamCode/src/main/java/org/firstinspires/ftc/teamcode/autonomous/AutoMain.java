package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.opencv.core.Mat;

@Autonomous(name="AutoMain")
public class AutoMain extends LinearOpMode {
    public static int X_SIZE = 1280;
    public static int Y_SIZE = 720;

    /**
     * The main function.
     * **/
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            CameraFrameCallback callback = new CameraFrameCallback(CameraHandler::getLocationOnBoard, X_SIZE, Y_SIZE);
            Camera camera = CameraHandler.createCamera(this.hardwareMap, X_SIZE, Y_SIZE, callback);
            CameraStreamServer.getInstance().setSource(callback.getCameraStreamSource());
        } catch (CameraException e) {
            throw new RuntimeException(e);
        }
        this.waitForStart();
    }
}
