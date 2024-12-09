package org.firstinspires.ftc.teamcode.obj;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class Huynh_BotHardware_test {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)

    // Define 4 motors for the drive
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // Define 2 motors for the arm
    private DcMotor leftArmMotor = null;
    private DcMotor righArmMotor = null;
    // Define 2 servos for the claw
    public Servo  wrist = null;
    public CRServo intake = null;
    public Servo claw = null;
    // Define motor for the actuator
    public DcMotor actuator = null;
    // Define motor for the elbow
    public DcMotor elbow = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    // public static final double MID_SERVO       =  0.5 ;
    // public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    // public static final double ARM_UP_POWER    =  0.45 ;
    // public static final double ARM_DOWN_POWER  = -0.45 ;

    public static final double INCREMENT   = 0.05;     // amount to slew servo each CYCLE_MS cycle
    public static final int    CYCLE_MS    =   50;     // period of each cycle
    public static final double MAX_POS     =  0.5;     // Maximum rotational position
    public static final double MIN_POS     =  0.0;     // Minimum rotational position


    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Huynh_BotHardware_test (LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        // leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        // rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        // armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //I changed the "left_front_drive" to the correct names for paw paw
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotor.class, "drivefl");// port 2
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotor.class, "drivebl"); // port 3
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "drivefr"); // port 1
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "drivebr"); // port 0
        actuator = myOpMode.hardwareMap.get(DcMotor.class, "actuator"); //port 0 on extension hub
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist"); //port 4 on extension hub
        intake = myOpMode.hardwareMap.get(CRServo.class, "intake"); // port 3 on extension hub
        claw = myOpMode.hardwareMap.get(Servo.class, "claw"); //port 5 on extension hub
        elbow = myOpMode.hardwareMap.get(DcMotor.class, "elbow"); //port 2 on extension hub

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(DcMotor.Direction.FORWARD);


        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        wrist.setPosition(0);
        // leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        // rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        // leftHand.setPosition(MID_SERVO);
        // rightHand.setPosition(MID_SERVO);


        myOpMode.telemetry.addData(">", "Hardware Initialized: Bot Ready!");
        myOpMode.telemetry.update();
    }

    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine drive and turn for blended motion.
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        // double leftFrontPower  = -axial - lateral + yaw;
        // double rightFrontPower = axial + lateral - yaw;
        // double leftBackPower   = axial - lateral + yaw;
        // double rightBackPower  = -axial + lateral - yaw;

        // double leftFrontPower  = axial + lateral + yaw;
        //     double rightFrontPower = axial - lateral - yaw;
        //     double leftBackPower   = axial - lateral + yaw;
        //     double rightBackPower  = axial + lateral - yaw;
        double leftFrontPower = -axial + lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower = axial + lateral + yaw;
        double rightBackPower = -axial + lateral - yaw;

        // Scale the values so neither exceed +/- 1.0
        double max;
        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Use existing function to drive both wheels.
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontWheel, double rightFrontWheel,
                              double leftBackWheel, double rightBackWheel) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontWheel);
        rightFrontDrive.setPower(rightFrontWheel);
        leftBackDrive.setPower(leftBackWheel);
        rightBackDrive.setPower(rightBackWheel);
    }

    // functions that control the wrist movement
    public void wristUp(){
        wrist.setPosition(0);
    }

    public void wristDown(){
        wrist.setPosition(0.75);
    }

    //function that controls the elbow


    // public void setClaw() {
    //     double  position = MIN_POS;
    //     // Set the servo to the position;
    //     leftClaw.setPosition(position + 0.0); // adding to left -> open claw
    //     rightClaw.setPosition(position + 0.2); // adding to right -> close claw

    // }
    // public void openClaw() {
    //     double  position = MIN_POS;
    //     // Set the servo to the position;
    //     leftClaw.setPosition(position + 0.2); // adding to left -> open claw
    //     rightClaw.setPosition(position + 0.0); // adding to right -> close claw
    // }



    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    // public void setArmPower(double power) {
    //     armMotor.setPower(power);
    // }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    // public void setHandPositions(double offset) {
    //     offset = Range.clip(offset, -0.5, 0.5);
    //     leftHand.setPosition(MID_SERVO + offset);
    //     rightHand.setPosition(MID_SERVO - offset);
    // }
}
