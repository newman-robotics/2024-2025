package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;

import java.nio.ByteBuffer;
import java.util.function.Consumer;

/**
 * A wrapper for a frame callback. Handles streaming to the DS and conversion to OpenCV types.
 * **/
public class CameraFrameCallback implements CameraCaptureSession.CaptureCallback {
    private final Consumer<Mat> callback;
    private Bitmap lastBitmap;

    /**
     * Creates a camera frame callback from the given consumer. For some reason, there's no way to dynamically fetch the size of a frame from the camera callback itself, so they must be provided here.
     * @param callback The consumer to be called on every frame.
     * **/
    public CameraFrameCallback(Consumer<Mat> callback) {
        this.callback = callback;
        RobotLog.i("Constructed CameraFrameCallback...");
    }

    /**
     * Override function. Don't call this from your own code.
     * **/
    @Override
    public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
        RobotLog.i("CameraFrameCallback::onNewFrame(" + cameraFrame.getFrameNumber() + ")...");

        byte[] rawData = cameraFrame.getImageData();
        if (rawData.length == 0) {
            RobotLog.e("Failed to find camera frame image data!");
            this.lastBitmap = null;
            return;
        }

        this.lastBitmap = request.createEmptyBitmap();
        cameraFrame.copyToBitmap(this.lastBitmap);
        Mat cvFrame = new MatOfByte(rawData).reshape(0, new int[]{cameraFrame.getSize().getHeight(), cameraFrame.getSize().getWidth()});
        RobotLog.i("CameraFrame size: (" + cameraFrame.getSize().getWidth() + ", " + cameraFrame.getSize().getHeight() + "), Mat size: (" + cvFrame.size().width + ", " + cvFrame.size().height + ")");
        callback.accept(cvFrame);
    }

    /**
     * Creates and returns a CameraStreamSource.
     * @return A CameraStreamSource that can be uploaded to the CameraStreamServer.
     * **/
    public CameraStreamSource getCameraStreamSource() {
        return continuation -> continuation.dispatch(new ContinuationResult<org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>>() {
            @Override
            public void handle(org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap> bitmapConsumer) {
                if (CameraFrameCallback.this.lastBitmap == null) RobotLog.e("Attempting to send null bitmap!");
                else bitmapConsumer.accept(CameraFrameCallback.this.lastBitmap);
            }
        });
    }
}
