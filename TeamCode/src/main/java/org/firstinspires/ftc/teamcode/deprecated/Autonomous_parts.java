package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous_parts {

     private LinearOpMode myOpMode = null;
     private DcMotor leftFrontDrive = null;
     private DcMotor leftBackDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightBackDrive = null;


     public Autonomous_parts(LinearOpMode opmode) {
         myOpMode = opmode;
     }

     public void init(){
         leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "drivefl");
         leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "drivebl");
         rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "drivefr");
         rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "drivebr");

         leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
         leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
         rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
         rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

         myOpMode.telemetry.addData(">", "Hardware Initialized: Bot Ready!");
         myOpMode.telemetry.update();
     }
//axial is trun, lateral is forward and back, yaw is side to side.
     public void driveRobot(double axial, double lateral, double yaw) {
         double leftFrontPower  = axial + lateral + yaw;
         double rightFrontPower = axial - lateral - yaw;
         double leftBackPower   = axial - lateral + yaw;
         double rightBackPower  = axial + lateral - yaw;
         double max;

         max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
         max = Math.max(max, Math.abs(leftBackPower));
         max = Math.max(max, Math.abs(rightBackPower));

         if (max > 1.0) {
             leftFrontPower  /= max;
             rightFrontPower /= max;
             leftBackPower   /= max;
             rightBackPower  /= max;
         }


         setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
     }

     public void setDrivePower(double leftFrontWheel, double rightFrontWheel, double leftBackWheel, double rightBackWheel) {
         leftFrontDrive.setPower(leftFrontWheel);
         rightFrontDrive.setPower(rightFrontWheel);
         leftBackDrive.setPower(leftBackWheel);
         rightBackDrive.setPower(rightBackWheel);
     }
 }

