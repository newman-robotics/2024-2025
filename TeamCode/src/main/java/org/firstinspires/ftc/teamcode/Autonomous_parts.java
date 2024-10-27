package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Autonomous_parts {

     private LinearOpMode myOpMode = null;
     private DcMotor leftFrontDrive = null;
     private DcMotor leftBackDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightBackDrive = null;

      public static final double MID_SERVO       =  0.5 ;
      public static final double HAND_SPEED      =  0.02 ;
      public static final double ARM_UP_POWER    =  0.45 ;
      public static final double ARM_DOWN_POWER  = -0.45 ;

     public static final double INCREMENT   = 0.05;
     public static final int    CYCLE_MS    =   50;
     public static final double MAX_POS     =  0.5;
     public static final double MIN_POS     =  0.0;


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

