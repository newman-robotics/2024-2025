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
import org.opencv.calib3d.Calib3d;
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

    //NOTE: internally we use inches, but we convert to fields after (1 field = 144 inches)
    //Format: 11 - 12 - 13 - 14 - 15 - 16
    private static final MatOfPoint3f[] aprilTagWorldContours = {
        CameraHandler.getContours(24., true, true),
        CameraHandler.getContours(72., false, false),
        CameraHandler.getContours(24., true, false),
        CameraHandler.getContours(120., true, false),
        CameraHandler.getContours(72., false, true),
        CameraHandler.getContours(120., true, true),
    };

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
     * Converts a Mat of 4 points to a MatOfPoint2f.
     * @param cornerSet The set of corners of the object.
     * @return A MatOfPoint2f representative of the original Mat's corners.
     * **/
    private static MatOfPoint2f convertCornerSet(Mat cornerSet) {
        Point first = new Point(cornerSet.get(0, 0)[0], cornerSet.get(0, 1)[0]);
        Point second = new Point(cornerSet.get(1, 0)[0], cornerSet.get(1, 1)[0]);
        Point third = new Point(cornerSet.get(2, 0)[0], cornerSet.get(2, 1)[0]);
        Point fourth = new Point(cornerSet.get(3, 0)[0], cornerSet.get(3, 1)[0]);
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
        RobotLog.i(String.format(Locale.UK, "Detection: ID %d, corners [(%f, %f), (%f, %f), (%f, %f), (%f, %f)]", id, cornerSet.get(0, 0)[0], cornerSet.get(0, 1)[0], cornerSet.get(1, 0)[0], cornerSet.get(1, 1)[0], cornerSet.get(2, 0)[0], cornerSet.get(2, 1)[0], cornerSet.get(3, 0)[0], cornerSet.get(3, 1)[0]));

        MatOfPoint3f objectPoints = CameraHandler.aprilTagWorldContours[id - 11];
        MatOfPoint2f imageCorners = CameraHandler.convertCornerSet(cornerSet);

        Mat rvec = new Mat(), tvec = new Mat();
        boolean out = Calib3d.solvePnP(objectPoints, imageCorners, CameraHandler.cameraMatrix, CameraHandler.distCoeffs, rvec, tvec);

        if (!out) RobotLog.w("CameraHandler::getLocationFromDetection: solvePnP returned false. Results from here on out may be completely random...");
        else RobotLog.w("CameraHandler::getLocationFromDetection: solvePnP returned true. I honestly don't know which is a good thing and which is a bad thing...");

        //how to interpret rvec and tvec?

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
