package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.ImageFormat;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.DetectorParameters;
import org.opencv.objdetect.Dictionary;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;

/**
 * Static functions related to the camera.
 * **/
public class CameraHandler {
    //Calibrated for Logitech C270 (see teamwebcamcalibrations.xml)
    public static Mat cameraMatrix;
    public static MatOfDouble distCoeffs;
    //too lazy to recreate this multiple times
    public static ArucoDetector detector = new ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11));

    static {
        RobotLog.i("Creating calibration matrices...");

        float[] cameraMatrixRaw = {822.317f, 0.f, 319.495f, 0.f, 822.317f, 242.502f, 0.f, 0.f, 1.f};
        CameraHandler.cameraMatrix = new MatOfFloat(cameraMatrixRaw)/*.reshape(0, new int[]{3, 3})*/;

        RobotLog.i("Created camera matrix...");

        double[] distCoeffs = {-0.0449369, 1.17277, 0., 0., -3.63244, 0., 0., 0.};
        CameraHandler.distCoeffs = new MatOfDouble(distCoeffs);

        RobotLog.i("CameraHandler setup completed");
    }

    /**
     * Field coordinate system:
     * All coordinates range from 0 to 1.
     * X runs from the blue basket to the red parking zone.
     * Y runs from the blue basket to the blue parking zone.
     * Therefore, (0, 0) is at the blue basket, (1, 0) is at
     * the red parking zone, (0, 1) is at the blue parking
     * zone, and (1, 1) is at the red basket.
     * Why is it like this? I don't know! I made it up!
     * **/
    public static class FieldPos {
        public final double x;
        public final double y;

        public FieldPos(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Creates a camera. Also sets it up with the given frame callback.
     * That way, I don't have to worry about it later.
     * NOTE TO SELF: The camera must be named "webcam".
     *
     * @param map The invoking OpMode's hardware map.
     * @param xSize The width of the capture.
     * @param ySize The height of the capture.
     * @param frameCallback A function wrapper to process frames.
     * @return The created camera, or null if an error occurred. Possibly not necessary, but good to have nonetheless.
     * **/
    @Nullable
    public static Camera createCamera(@NonNull HardwareMap map, int xSize, int ySize, @NonNull CameraCaptureSession.CaptureCallback frameCallback) throws CameraException, AutoUtil.OpModeInterruptedException {
        RobotLog.i("Getting cameraManager...");
        CameraManager cameraManager = ClassFactory.getInstance().getCameraManager();
        RobotLog.i("Getting serial thread pool...");
        Executor serialThreadPool = ((CameraManagerInternal)cameraManager).getSerialThreadPool();
        RobotLog.i("Getting WebcamName...");
        WebcamName name = map.get(WebcamName.class, GlobalConstants.WEBCAM_NAME);
        AutoUtil.Counter cameraCreated = new AutoUtil.Counter(1);
        AutoUtil.Value<Camera> cameraWrapper = new AutoUtil.Value<Camera>();
        RobotLog.i("Opening camera...");
        cameraManager.requestPermissionAndOpenCamera(new Deadline(5000, TimeUnit.MILLISECONDS), name, Continuation.create(serialThreadPool, new Camera.StateCallback() {
            @Override public void onOpened(@NonNull Camera camera) {
                RobotLog.i("Camera " + camera + " successfully opened");
                cameraWrapper.value = camera;
                cameraCreated.decrement();
            }
            @Override public void onOpenFailed(@NonNull CameraName cameraName, @NonNull Camera.OpenFailure reason) {
                RobotLog.e("Camera with name " + cameraName + " failed to open with reason " + reason);
                cameraCreated.decrement();
            }
            @Override public void onClosed(@NonNull Camera camera) {RobotLog.i("Camera " + camera + " closed");}
            @Override public void onError(@NonNull Camera camera, Camera.Error error) {
                RobotLog.e("Error during creation of camera " + camera + ": " + error);
                camera.close();
                cameraCreated.decrement();
            }
        }));
        RobotLog.i("Waiting...");
        AutoUtil.waitOnCounter(cameraCreated, 5000);
        Camera camera = cameraWrapper.value;
        if (camera != null) {
            RobotLog.i("About to create camera capture session with camera " + camera);
            AutoUtil.Counter cameraOpened = new AutoUtil.Counter(1);
            CameraCaptureSession session = camera.createCaptureSession(Continuation.create(serialThreadPool, new CameraCaptureSession.StateCallback() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        int fps = session.getCamera().getCameraName().getCameraCharacteristics().getMaxFramesPerSecond(ImageFormat.YUY2, new Size(xSize, ySize));
                        CameraCaptureRequest request = camera.createCaptureRequest(ImageFormat.YUY2, new Size(xSize, ySize), fps);

                        RobotLog.i("About to call session.startCapture(): session = " + session + ", request = " + request);

                        session.startCapture(request, frameCallback, Continuation.create(serialThreadPool, new CameraCaptureSession.StatusCallback() {
                            @Override
                            public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                RobotLog.i("Camera capture sequence completed with " + lastFrameNumber + " frames");
                            }
                        }));
                        cameraOpened.decrement();
                    } catch (CameraException e) {
                        session.close();
                        RobotLog.e("Exception occurred while configuring camera: " + e);
                        throw new RuntimeException(e);
                    } catch (NullPointerException e) {
                        e.printStackTrace();
                        throw e;
                    }
                }

                @Override
                public void onClosed(@NonNull CameraCaptureSession session) {
                    RobotLog.i("Capture session closed!");
                }
            }));
            boolean timedOut = AutoUtil.waitOnCounter(cameraOpened, 5000);
            if (timedOut) {
                RobotLog.e("Timed out while opening the camera!");
                session.close();
                camera.close();
                return null;
            }
        }
        return camera;
    }

    /**
     * Tries to determine the location of the robot from the given tag. Currently unable to utilise multiple tags.
     * @param cornerSet The corners of the detection.
     * @param id The id of the detection.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    private static FieldPos getLocationFromDetection(Mat cornerSet, int id) {
        RobotLog.i(String.format(Locale.UK, "Detection: ID %d, corners [(%f, %f), (%f, %f), (%f, %f), (%f, %f)]", id, cornerSet.get(0, 0)[0], cornerSet.get(0, 1)[0], cornerSet.get(1, 0)[0], cornerSet.get(1, 1)[0], cornerSet.get(2, 0)[0], cornerSet.get(2, 1)[0], cornerSet.get(3, 0)[0], cornerSet.get(3, 1)[0]));
        /*
        MatOfPoint3f objectPoints = new MatOfPoint3f();
        MatOfPoint2f imageCorners = new MatOfPoint2f(new Point(tag.p[0], tag.p[1]), new Point(tag.p[2], tag.p[3]), new Point(tag.p[4], tag.p[5]), new Point(tag.p[6], tag.p[7]));

        Mat rvec = new Mat(1, 3, CvType.CV_32F), tvec = new Mat(1, 3, CvType.CV_32F);
        boolean out = Calib3d.solvePnP(objectPoints, imageCorners, CameraHandler.cameraMatrix, CameraHandler.distCoeffs, rvec, tvec);
        */
        return null;
    }

    /**
     * Calculates the robot's position on the field from a camera frame.
     * @param frame The frame from the camera. Must be in YUV/YCbCr format.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    public static FieldPos getLocationOnBoard(Mat frame) {
        List<Mat> corners = new ArrayList<Mat>(0);
        Mat ids = new Mat();
        CameraHandler.detector.detectMarkers(frame, corners, ids);
        for (int i = 0; i < corners.size(); ++i) {
            FieldPos position = getLocationFromDetection(corners.get(i), (int)ids.get(i, 0)[0]);
            if (position != null) return position;
        }
        return null;
    }
}
