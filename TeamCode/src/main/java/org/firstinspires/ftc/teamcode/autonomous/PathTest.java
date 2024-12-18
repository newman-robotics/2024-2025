package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

@Autonomous
public class PathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //set this up in the future
        AutoUtil.Drivetrain drivetrain = null;
        GoBildaPinpointDriver odometry = null;

        Path path = new Path.Builder(drivetrain, odometry)
                .andThen(new CameraHandler.FieldPos(36, 72, 0))
                .andThen(new CameraHandler.FieldPos(36, 108, 180))
                .build();
    }
}
