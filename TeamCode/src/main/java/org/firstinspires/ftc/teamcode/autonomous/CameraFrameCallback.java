package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.function.Consumer;

/**
 * A wrapper for a frame callback. Handles streaming to the DS and conversion to OpenCV types.
 * **/
public class CameraFrameCallback implements CameraCaptureSession.CaptureCallback {
    private final Consumer<Mat> callback;
    private final int xSize;
    private final int ySize;
    private final Bitmap lastBitmap;

    /**
     * Creates a camera frame callback from the given consumer. For some reason, there's no way to dynamically fetch the size of a frame from the camera callback itself, so they must be provided here.
     * @param callback The consumer to be called on every frame.
     * @param xSize The width of the image. Must be the same as what is passed to the camera.
     * @param ySize The height of the image. Must be the same as what is passed to the camera.
     * **/
    public CameraFrameCallback(Consumer<Mat> callback, int xSize, int ySize) {
        this.callback = callback;
        this.xSize = xSize;
        this.ySize = ySize;
        this.lastBitmap = Bitmap.createBitmap(xSize, ySize, Bitmap.Config.RGB_565);
    }

    /**
     * Override function. Don't call this from your own code.
     * **/
    @Override
    public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
        //If cameraFrame is already YCbCr, I will lose it...
        cameraFrame.copyToBitmap(this.lastBitmap);
        Mat rgbFrame = new Mat(this.xSize, this.ySize, CvType.CV_8U, ByteBuffer.wrap(cameraFrame.getImageData()));
        Mat yuvFrame = new Mat(this.xSize, this.ySize, CvType.CV_8U);
        Imgproc.cvtColor(rgbFrame, yuvFrame, Imgproc.COLOR_RGB2YUV);
        callback.accept(yuvFrame);
    }

    /**
     * Creates and returns a CameraStreamSource.
     * @return A CameraStreamSource that can be uploaded to the CameraStreamServer.
     * **/
    public CameraStreamSource getCameraStreamSource() {
        return new CameraStreamSource() {
            @Override
            public void getFrameBitmap(Continuation<? extends org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>> continuation) {
                continuation.dispatch(new ContinuationResult<org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>>() {
                    @Override
                    public void handle(org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap> bitmapConsumer) {
                        bitmapConsumer.accept(CameraFrameCallback.this.lastBitmap);
                    }
                });
            }
        };
    }
}
