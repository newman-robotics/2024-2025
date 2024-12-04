package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.objdetect.ArucoDetector;
import org.opencv.objdetect.Objdetect;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.function.BiConsumer;
import java.util.function.BinaryOperator;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Collector;
import java.util.stream.Collectors;

/**
 * Static functions related to the camera.
 * Basically AutoUtil if AutoUtil included camera code.
 * **/
public class CameraHandler {
    //Calibrated for Logitech C270 (see teamwebcamcalibrations.xml)
    public static Mat cameraMatrix;
    public static MatOfDouble distCoeffs;
    public static final ArucoDetector detector = new ArucoDetector(Objdetect.getPredefinedDictionary(Objdetect.DICT_APRILTAG_36h11));

    /**
     * Creates the contours of an AprilTag at the given offset.
     * @param offset The offset of the AprilTag along its wall, measured in inches to the centre of the AprilTag.
     * @param isAlongXWall Whether the AprilTag is along a wall that increases in the X direction (i.e. if offset measures an X coordinate or a Z coordinate.)
     * @param isAlongHighWall Whether the non-dominant coordinate (i.e. the one that offset does not measure) is 1.
     * **/
    private static MatOfPoint3f getContours(double offset, boolean isAlongXWall, boolean isAlongHighWall) {
        double staticCoordinate = isAlongHighWall ? 1. : 0.;

        Point3 first = new Point3(isAlongXWall ? offset - 2. : staticCoordinate, 6.25, isAlongXWall ? staticCoordinate : offset - 2.);
        Point3 second = new Point3(isAlongXWall ? offset + 2. : staticCoordinate, 6.25, isAlongXWall ? staticCoordinate : offset + 2.);
        Point3 third = new Point3(isAlongXWall ? offset + 2. : staticCoordinate, 2.25, isAlongXWall ? staticCoordinate : offset + 2.);
        Point3 fourth = new Point3(isAlongXWall ? offset - 2. : staticCoordinate, 2.25, isAlongXWall ? staticCoordinate : offset - 2.);

        return new MatOfPoint3f(first, second, third, fourth);
    }

    //Format: 11 - 12 - 13 - 14 - 15 - 16
    private static final MatOfPoint3f[] aprilTagWorldContours = {
        CameraHandler.getContours(24., true, true),
        CameraHandler.getContours(72., false, false),
        CameraHandler.getContours(24., true, false),
        CameraHandler.getContours(120., true, false),
        CameraHandler.getContours(72., false, true),
        CameraHandler.getContours(120., true, true),
    };

    private static final MatOfPoint3f aprilTagOriginContours = CameraHandler.getContours(0., true, false);

    static {
        RobotLog.i("Creating calibration matrices...");

        float[] cameraMatrixRaw = {822.317f, 0.f, 319.495f, 0.f, 822.317f, 242.502f, 0.f, 0.f, 1.f};
        CameraHandler.cameraMatrix = new MatOfFloat(cameraMatrixRaw).reshape(0, new int[]{3, 3});

        RobotLog.i("Created camera matrix...");

        double[] distCoeffs = {-0.0449369, 1.17277, 0., 0., -3.63244, 0., 0., 0.};
        CameraHandler.distCoeffs = new MatOfDouble(distCoeffs);

        RobotLog.i("CameraHandler setup completed");
    }

    /**
     * Field coordinate system:
     * All coordinates range from 0 to 144 (inches).
     * X runs from the blue basket to the red parking zone.
     * Y runs from the blue basket to the blue parking zone.
     * Therefore, (0, 0) is at the blue basket, (144, 0) is at
     * the red parking zone, (0, 144) is at the blue parking
     * zone, and (144, 144) is at the red basket.
     * Additionally, 0 degrees is pointing from -X to +X.
     * Why is it like this? I don't know! I made it up!
     * **/
    public static class FieldPos {
        public final double x;
        public final double y;
        public final double theta;

        public FieldPos(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        @Override
        public String toString() {
            return "FieldPos{" +
                    "x=" + x +
                    ", y=" + y +
                    ", theta=" + theta +
                    '}';
        }
    }

    /**
     * A wrapper for a frame callback. Handles streaming to the DS and conversion to OpenCV types.
     * **/
    public static class CameraFrameCallback implements CameraCaptureSession.CaptureCallback {
        private final Consumer<Mat> callback;
        private Bitmap lastBitmap;
        private final ExecutorService executor = Executors.newFixedThreadPool(GlobalConstants.MAX_CAMERA_PROCESSING_THREADS);

        /**
         * Creates a camera frame callback from the given consumer.
         * @param callback The consumer to be called asynchronously on every frame. (The frame is RGB.)
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

            executor.submit(() -> {
                Mat cvFrame = new Mat();
                Bitmap bmp2 = bmp.copy(Bitmap.Config.RGB_565, false);
                Utils.bitmapToMat(bmp2, cvFrame);
                callback.accept(cvFrame);
            });
        }

        /**
         * Creates and returns a CameraStreamSource.
         * @return A CameraStreamSource that can be uploaded to the CameraStreamServer.
         * **/
        public CameraStreamSource getCameraStreamSource() {
            return continuation -> continuation.dispatch((ContinuationResult<org.firstinspires.ftc.robotcore.external.function.Consumer<Bitmap>>) bitmapConsumer -> {
                if (CameraFrameCallback.this.lastBitmap == null) RobotLog.e("Attempting to send null bitmap!");
                else bitmapConsumer.accept(CameraFrameCallback.this.lastBitmap);
            });
        }
    }

    /**
     * Creates a camera. Also sets it up with the given frame callback.
     * That way, I don't have to worry about it later.
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
        AutoUtil.Value<Camera> cameraWrapper = new AutoUtil.Value<>();
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

                        session.startCapture(request, frameCallback, Continuation.create(serialThreadPool, (session1, cameraCaptureSequenceId, lastFrameNumber) -> RobotLog.i("Camera capture sequence completed with " + lastFrameNumber + " frames")));
                        cameraOpened.decrement();
                    } catch (CameraException e) {
                        session.close();
                        RobotLog.e("Exception occurred while configuring camera: " + e);
                        throw new RuntimeException(e);
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
     * Converts a Mat of 4 points to a MatOfPoint2f.
     * @param cornerSet The set of corners of the object.
     * @return A MatOfPoint2f representative of the original Mat's corners.
     * **/
    private static MatOfPoint2f convertCornerSet(Mat cornerSet) {
        //RobotLog.i("CameraHandler.convertCornerSet: cornerSet = " + cornerSet);

        double firstx = cornerSet.get(0, 0)[0];
        double firsty = cornerSet.get(0, 0)[1];
        Point first = new Point(firstx, firsty);

        double secondx = cornerSet.get(0, 1)[0];
        double secondy = cornerSet.get(0, 1)[1];
        Point second = new Point(secondx, secondy);

        double thirdx = cornerSet.get(0, 2)[0];
        double thirdy = cornerSet.get(0, 2)[1];
        Point third = new Point(thirdx, thirdy);

        double fourthx = cornerSet.get(0, 3)[0];
        double fourthy = cornerSet.get(0, 3)[1];
        Point fourth = new Point(fourthx, fourthy);

        return new MatOfPoint2f(first, second, third, fourth);
    }

    /**
     * Tries to determine the location of the robot from the given tag. Currently unable to utilise multiple tags.
     * @param cornerSet The corners of the detection.
     * @param id The id of the detection.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    private static FieldPos getLocationFromDetection(Mat cornerSet, int id) {
        //MatOfPoint3f objectPoints = CameraHandler.aprilTagWorldContours[id - 11];
        MatOfPoint2f imageCorners = CameraHandler.convertCornerSet(cornerSet);

        RobotLog.i("Solving PnP...");

        Mat rvec = new Mat(3, 1, CvType.CV_64F), tvec = new Mat(3, 1, CvType.CV_64F);
        boolean out = Calib3d.solvePnP(CameraHandler.aprilTagOriginContours, imageCorners, CameraHandler.cameraMatrix, CameraHandler.distCoeffs, rvec, tvec);

        if (!out) {
            RobotLog.e("CameraHandler::getLocationFromDetection: solvePnP failed!");
            return null;
        }

        RobotLog.i("Solved PnP; calculating world pos...");

        Mat rmat = new Mat();
        Calib3d.Rodrigues(rvec, rmat);

        RobotLog.i("Reversing rotation...");

        Mat origin = Mat.zeros(1, 1, CvType.CV_64FC3);
        Mat cameraRotationChannels = new Mat();
        Core.transform(origin, cameraRotationChannels, rmat);

        RobotLog.i("Applying channel transform...");

        //transform from 1x1 with 3 channels to 3x1 with 1 channel
        Mat cameraRotation = Mat.zeros(3, 1, CvType.CV_64F);
        cameraRotation.put(0, 0, cameraRotationChannels.get(0, 0));

        RobotLog.i("Reversing translation...");

        Mat cameraTranslation = new Mat(3, 1, CvType.CV_64F);
        Core.add(cameraRotation, tvec, cameraTranslation);

        RobotLog.i("Putting telemetry...");

        AutoUtil.ChainTelemetry telemetry = AutoUtil.ChainTelemetry.get();
        assert telemetry != null;
        telemetry.add("Data in inches and (probably) radians.")
            .add("Detection ID: ", id)
            //.add("Relative X: ", tvec.get(0, 0)[0]).add("Relative Y: ", tvec.get(1, 0)[0]).add("Relative Z: ", tvec.get(2, 0)[0])
            //.add("Relative Yaw:   ", rvec.get(0, 0)[0]).add("Relative Pitch: ", rvec.get(1, 0)[0]).add("Relative Roll:  ", rvec.get(2, 0)[0])
            .add("World origin X: ", cameraTranslation.get(0, 0)[0]).add("World origin Y: ", cameraTranslation.get(0, 1)[0]).add("World origin Z: ", cameraTranslation.get(0, 2)[0])
            .update();

        return null;
    }

    /**
     * Calculates the robot's position on the field from a camera frame.
     * @param frame The frame from the camera.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    public static FieldPos getLocationOnBoard(Mat frame) {
        List<Mat> corners = new ArrayList<>(0);
        Mat ids = new Mat();
        CameraHandler.detector.detectMarkers(frame, corners, ids);
        for (int i = 0; i < corners.size(); ++i) {
            FieldPos position = getLocationFromDetection(corners.get(i), (int)ids.get(i, 0)[0]);
            if (position != null) return position;
        }
        return null;
    }
}
