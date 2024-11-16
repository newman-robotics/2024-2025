package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.constants.UvcFrameFormat;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * A wrapper for a frame callback. Handles streaming to the DS and conversion to OpenCV types.
 * **/
public class CameraFrameCallback implements CameraCaptureSession.CaptureCallback {
    private final Consumer<Mat> callback;
    private Bitmap lastBitmap;

    /**
     * Creates a camera frame callback from the given consumer.
     * @param callback The consumer to be called on every frame. (The frame is RGB.)
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
        if (cameraFrame.getFrameNumber() % 10 == 0) RobotLog.i("CameraFrameCallback::onNewFrame(" + cameraFrame.getFrameNumber() + ")...");

        byte[] rawData = cameraFrame.getImageData();
        if (rawData.length == 0) {
            RobotLog.e("Failed to find camera frame image data!");
            this.lastBitmap = null;
            return;
        }

        this.lastBitmap = request.createEmptyBitmap();
        Bitmap bmp = request.createEmptyBitmap();
        cameraFrame.copyToBitmap(bmp);
        Mat cvFrame = new Mat();
        Bitmap bmp2 = bmp.copy(Bitmap.Config.RGB_565, false);
        Utils.bitmapToMat(bmp2, cvFrame);
        //moving this here for now
        //if you see this I forgot to remove this
        List<Mat> corners = new ArrayList<Mat>();
        Mat ids = new Mat();
        CameraHandler.detector.detectMarkers(cvFrame, corners, ids);
        for (int i = 0; i < corners.size(); ++i) {
            RobotLog.i("Drawing lines...");
            Imgproc.line(cvFrame,
                    new Point((int)corners.get(i).get(0, 0)[0], (int)corners.get(i).get(0, 1)[0]),
                    new Point((int)corners.get(i).get(1, 0)[0], (int)corners.get(i).get(1, 1)[0]),
                    new Scalar(0, 1, 0));
            Imgproc.line(cvFrame,
                    new Point((int)corners.get(i).get(1, 0)[0], (int)corners.get(i).get(1, 1)[0]),
                    new Point((int)corners.get(i).get(2, 0)[0], (int)corners.get(i).get(2, 1)[0]),
                    new Scalar(0, 1, 0));
            Imgproc.line(cvFrame,
                    new Point((int)corners.get(i).get(2, 0)[0], (int)corners.get(i).get(2, 1)[0]),
                    new Point((int)corners.get(i).get(3, 0)[0], (int)corners.get(i).get(3, 1)[0]),
                    new Scalar(0, 1, 0));
            Imgproc.line(cvFrame,
                    new Point((int)corners.get(i).get(3, 0)[0], (int)corners.get(i).get(3, 1)[0]),
                    new Point((int)corners.get(i).get(0, 0)[0], (int)corners.get(i).get(0, 1)[0]),
                    new Scalar(0, 1, 0));
        }
        //inefficient but useful for debugging
        Utils.matToBitmap(cvFrame, this.lastBitmap);
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
