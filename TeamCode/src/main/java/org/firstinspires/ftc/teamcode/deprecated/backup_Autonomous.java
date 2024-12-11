package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.deprecated.Autonomous_parts;

@Disabled
@Autonomous(name="Backup_nonecammera", group="Test Robot")

public class backup_Autonomous extends LinearOpMode {

     Autonomous_parts powerClaw = new Autonomous_parts(this);

     private ElapsedTime     runtime = new ElapsedTime();


     static final double     FORWARD_SPEED = 0.5;
     static final double     TURN_SPEED    = 0.5;
     static final double     SIDE_SPEED    = 0.5;

     @Override
     public void runOpMode() {

         powerClaw.init();

         telemetry.addData("Status", "Ready to run");
         telemetry.update();

         waitForStart();

         powerClaw.driveRobot(FORWARD_SPEED, 0, 0);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 2.3)) {
             telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
             telemetry.update();
         }


         powerClaw.driveRobot(0, 0, TURN_SPEED);
         powerClaw.driveRobot(0, 0, -TURN_SPEED);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 0.75)) {
             telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
             telemetry.update();
         }


         powerClaw.driveRobot(FORWARD_SPEED, 0, 0);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 2.5)) {
             telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
             telemetry.update();
         }


         powerClaw.driveRobot(0, SIDE_SPEED, 0);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 2.5)) {
             telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
             telemetry.update();
         }

          runtime.reset();
          while (opModeIsActive() && (runtime.seconds() < 1.0)) {
              telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
              telemetry.update();
          }

         powerClaw.driveRobot(0, 0, 0);

         telemetry.addData("Path", "Complete");
         telemetry.update();
         sleep(1000);
     }
 }
