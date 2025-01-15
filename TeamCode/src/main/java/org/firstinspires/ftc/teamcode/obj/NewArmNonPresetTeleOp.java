package org.firstinspires.ftc.teamcode.obj;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;


@TeleOp(name="STele")
public class NewArmNonPresetTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private long startTime = 0;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    // Declare a motor for the actuator
    private DcMotor actuator = null;
    // Declare a motor for the elbow
    private DcMotor elbow = null;
    // Declare a servo for the intake and wrist
    private DcMotor elevation = null;
    private Servo claw = null;

    public double DistanceToTime (double speed, double distance){
        double time = distance/speed; //time in seconds, speed in cm/s
        // axial: 0.2speed = 9.45in/s, 0.5speed = 36in/s
        // lateral: 0.5speed = 22in/s
        return time;
    }

    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "drivefl");// port 2
        leftBackDrive  = hardwareMap.get(DcMotor.class, "drivebl"); // port 3
        rightFrontDrive = hardwareMap.get(DcMotor.class, "drivefr"); // port 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "drivebr"); // port 0
        actuator = hardwareMap.get(DcMotor.class, "actuator"); //motor port 0 on extension hub
        elbow = hardwareMap.get(DcMotor.class, "elbow"); //motor port 2 on extension hub
        claw = hardwareMap.get(Servo.class, "claw"); //port 5 on extension hub
        //elevation = hardwareMap.get(CRServo.class, "elevation"); // port 4 on extension hub
        // replace elevation CRServo with elevation DC motor in extension hub port 3
        elevation = hardwareMap.get(DcMotor.class, "elevation"); //motor port 3 on extension hub

        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setPower(0.5);
        elbow.setTargetPosition(0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        elevation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // elevation.setPower(1);
        // elevation.setTargetPosition(0);
        // elevation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int elbowCurrentPosition = elbow.getCurrentPosition();
        int elevationCurrentPosition = elevation.getCurrentPosition();
        int gameStart = 0;

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");// port 2
        // leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); // port 3
        // rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); // port 1
        // rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); // port 0
        // actuator = hardwareMap.get(DcMotor.class, "actuator"); //port 0 on extension hub

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        actuator.setDirection(DcMotor.Direction.FORWARD);

        elevation.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        this.startTime = System.currentTimeMillis();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            if (gameStart == 0){
                claw.setPosition(0);
                gameStart = 1;
            }

            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadLeft = gamepad1.dpad_left;
            float leftTrigger = gamepad1.left_trigger;
            float rightTrigger = gamepad1.right_trigger;
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean buttonA = gamepad1.a;
            boolean buttonB = gamepad1.b;
            boolean buttonX = gamepad1.x;
            boolean buttonY = gamepad1.y;

            double leftFrontPower = -axial - lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = -axial - lateral - yaw;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // if the left trigger (LT) is hold, reduce speed
            if (Math.abs(gamepad1.left_trigger) > 0){
                leftFrontDrive.setPower(leftFrontPower/5);
                rightFrontDrive.setPower(rightFrontPower/5);
                leftBackDrive.setPower(leftBackPower/5);
                rightBackDrive.setPower(rightBackPower/5);
            }
            else{
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);
            }

            elbow.setTargetPosition(elbow.getCurrentPosition() + (int)AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_TRIGGER), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_TRIGGER)) * 150);
            elbowCurrentPosition = elbow.getCurrentPosition();

            elevation.setPower(AutoUtil.ternaryXOR(rightBumper, leftBumper) / 2.);
            elevationCurrentPosition = elevation.getCurrentPosition();

            if (buttonA) claw.setPosition(1);
            else claw.setPosition(0);

            actuator.setPower(AutoUtil.ternaryXOR(dpadUp, dpadDown));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + (System.currentTimeMillis() - this.startTime));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Left Trigger", gamepad1.left_trigger);
            telemetry.addData("Right Trigger", gamepad1.right_trigger);
            telemetry.addData("Elbow Position:", elbowCurrentPosition);
            telemetry.addData("Elevation Position:", elevationCurrentPosition);
            telemetry.addData("Button A", buttonA);
            telemetry.addData("Button B", buttonB);
            telemetry.addData("lf drive pos", leftFrontDrive.getCurrentPosition());
            telemetry.addData("rf drive pos", rightFrontDrive.getCurrentPosition());
            telemetry.addData("lb drive pos", leftBackDrive.getCurrentPosition());
            telemetry.addData("rb drive pos", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
    }}
