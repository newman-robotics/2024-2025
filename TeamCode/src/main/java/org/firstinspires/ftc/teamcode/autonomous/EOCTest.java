package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Autonomous(name="EOCTest")
public class EOCTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(this.hardwareMap.get(WebcamName.class, "webcam"));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720);
            }

            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Camera error with code: " + errorCode);
            }
        });
        this.waitForStart();
    }
}
