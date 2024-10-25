package org.firstinspires.ftc.teamcode.autonomous;

//Enables a hacky fix for the null camera problem.
#define USE_REFLECTION

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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCamera;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSession;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.opencv.core.Mat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

import edu.umich.eecs.april.apriltag.ApriltagDetection;
import edu.umich.eecs.april.apriltag.ApriltagNative;

/**
 * Static functions related to the camera.
 * **/
public class CameraHandler {
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
     * @return The created camera. Possibly not necessary, but good to have nonetheless.
     * **/
    public static Camera createCamera(HardwareMap map, int xSize, int ySize, CameraCaptureSession.CaptureCallback frameCallback) throws CameraException {
        WebcamName name = map.get(WebcamName.class, "webcam");
        Camera camera = ClassFactory.getInstance().getCameraManager().requestPermissionAndOpenCamera(new Deadline(5000, TimeUnit.MILLISECONDS), name, null);
        if (camera instanceof DelegatingCamera) RobotLog.i("Camera is delegating! Crap!");
        else RobotLog.i("Camera is not delegating! Maybe we can use that as an exploit...");
        CameraCaptureRequest request = camera.createCaptureRequest(20, new Size(xSize, ySize), 30);
        RobotLog.i("About to create camera capture session");
        CameraCaptureSession session = camera.createCaptureSession(Continuation.createTrivial(
                new CameraCaptureSession.StateCallback(){
                    @Override
                    public void onConfigured(@NonNull CameraCaptureSession session) {
                        try {
                            RobotLog.i("onConfigured()...");
                            RobotLog.i("Camera = " + session.getCamera().getCameraName());
                            int[] androidFormats = session.getCamera().getCameraName().getCameraCharacteristics().getAndroidFormats();
                            for (int androidFormat : androidFormats) RobotLog.i(String.format(Locale.UK, "%d", androidFormat));
                            #if USE_REFLECTION
                            //here comes the unsafe stuff!
                            if (session instanceof DelegatingCaptureSession) {
                                try {
                                    RobotLog.i("Here goes nothing!");
                                    Field cameraField = DelegatingCaptureSession.class.getDeclaredField("camera");
                                    cameraField.setAccessible(true);
                                    Camera sessionCamera = (Camera)cameraField.get((DelegatingCaptureSession)session);
                                    RobotLog.i("camera is: " + sessionCamera);
                                    if (camera.getCameraName().isWebcam()) {
                                        RobotLog.i("base camera name is webcam, attempting to force it into delegating camera...");
                                        Method setCameraMethod = DelegatingCamera.class.getDeclaredMethod("changeDelegatedCamera", Camera.class);
                                        setCameraMethod.setAccessible(true);
                                        setCameraMethod.invoke(sessionCamera, camera);
                                        RobotLog.i("set delegated camera of " + sessionCamera + " to " + camera);
                                    } else {
                                        RobotLog.i("base camera is not webcam...");
                                    }
                                } catch (NoSuchFieldException e) {
                                    RobotLog.w("session is an instance of DelegatingCaptureSession, but does not contain field camera!\n" + e);
                                } catch (IllegalAccessException e) {
                                    RobotLog.w("illegal access!\n" + e);
                                } catch (NoSuchMethodException e) {
                                    RobotLog.w("changeDelegatedCamera does not exist!\n" + e);
                                } catch (InvocationTargetException e) {
                                    RobotLog.w("invocation target something-or-othered!\n" + e);
                                }
                            }
                            #endif //USE_REFLECTION
                            session.startCapture(request, Continuation.createTrivial(frameCallback), Continuation.createTrivial(new CameraCaptureSession.StatusCallback(){
                                @Override
                                public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {

                                }
                            }));
                            RobotLog.i("...onConfigured()");
                        } catch (CameraException e) {
                            throw new RuntimeException(e);
                        }
                    }

                    @Override
                    public void onClosed(@NonNull CameraCaptureSession session) {

                    }
                }
        ));
        return camera;
    }

    /**
     * Tries to determine the location of the robot from the given tag. Currently unable to utilise multiple tags.
     * @param tag The tag.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    private static FieldPos getLocationFromDetection(ApriltagDetection tag) {
        RobotLog.d(String.format(Locale.UK, "Detection: ID %d, centre (%f, %f), corners [(%f, %f), (%f, %f), (%f, %f), (%f, %f)]", tag.id, tag.c[0], tag.c[1], tag.p[0], tag.p[1], tag.p[2], tag.p[3], tag.p[4], tag.p[5], tag.p[6], tag.p[7]));
        return null;
    }

    /**
     * Calculates the robot's position on the field from a camera frame.
     * @param frame The frame from the camera. Must be in YUV/YCbCr format.
     * @return The location of the robot on the field, or null if it could not be determined.
     * **/
    @Nullable
    public static FieldPos getLocationOnBoard(Mat frame) {
        //idk if these configs are right but let's pray...
        ApriltagNative.apriltag_init("36h11", 0, 8, 0, 4);
        int length = (int)(frame.elemSize() * frame.total());
        byte[] buffer = new byte[length];
        frame.get(0, 0, buffer);
        ArrayList<ApriltagDetection> tags = ApriltagNative.apriltag_detect_yuv(buffer, frame.cols(), frame.rows());
        for (ApriltagDetection tag : tags) {
            FieldPos position = getLocationFromDetection(tag);
            if (position != null) return position;
        }
        return null;
    }
}
