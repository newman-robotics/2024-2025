package org.firstinspires.ftc.teamcode;

//Whether to use the arm
#define USE_ARM

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="New_Code_Juju")
public class NotCopiedDrive extends LinearOpMode {
    /**
     * Represents the powers of the motors.
     * **/
    public class MotorPowers {
        private double frontLeft;
        private double frontRight;
        private double backLeft;
        private double backRight;

        /**
         * Default constructor: zero-initialises everything.
         * **/
        public MotorPowers() {
            this(0.f, 0.f, 0.f, 0.f);
        }

        /**
         * Creates a new MotorPowers from the given powers.
         * @param frontLeft The power of the front left motor.
         * @param frontRight The power of the front right motor.
         * @param backLeft The power of the back left motor.
         * @param backRight The power of the back right motor.
         * **/
        public MotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        public double getFrontLeft() {return this.frontLeft;}
        public double getFrontRight() {return this.frontRight;}
        public double getBackLeft() {return this.backLeft;}
        public double getBackRight() {return this.backRight;}

        public void setFrontLeft(double value) {this.frontLeft = value;}
        public void setFrontRight(double value) {this.frontRight = value;}
        public void setBackLeft(double value) {this.backLeft = value;}
        public void setBackRight(double value) {this.backRight = value;}

        /**
         * Multiplies every power by the given factor.
         * @param factor The multiplier to be applied to every power.
         * **/
        public void scale(double factor) {
            this.frontLeft *= factor;
            this.frontRight *= factor;
            this.backLeft *= factor;
            this.backRight *= factor;
        }

        /**
         * Clamps every value to the range [-1.0, 1.0].
         * Not actually sure if this is necessary at all, or if the motors do this automatically.
         * **/
        public void clamp() {
            if (this.frontLeft > 1.0f) this.frontLeft = 1.0f;
            if (this.frontRight > 1.0f) this.frontRight = 1.0f;
            if (this.backLeft > 1.0f) this.backLeft = 1.0f;
            if (this.backRight > 1.0f) this.backRight = 1.0f;

            if (this.frontLeft < -1.0f) this.frontLeft = -1.0f;
            if (this.frontRight < -1.0f) this.frontRight = -1.0f;
            if (this.backLeft < -1.0f) this.backLeft = -1.0f;
            if (this.backRight < -1.0f) this.backRight = -1.0f;
        }

        /**
         * Applies these powers to the robot's motors.
         * **/
        public void apply() {
            NotCopiedDrive.this.frontLeft.setPower(this.frontLeft);
            NotCopiedDrive.this.frontRight.setPower(this.frontRight);
            NotCopiedDrive.this.backLeft.setPower(this.backLeft);
            NotCopiedDrive.this.backRight.setPower(this.backRight);
        }
    }

    #if USE_ARM
    /**
     * Represents powers of the arm.
     * **/
    public class ArmPowers {
        private double elevation;
        private double angle;

        /**
         * Default constructor: zero-initialises everything.
         * **/
        public ArmPowers() {
            this(0.0f);
        }

        /**
         * Sets the elevation to the given value without the arm angle changing.
         * @param elevation The power with which to move the arm. Positive means up, negative means down.
         * **/
        public ArmPowers(double elevation) {
            this(elevation, NotCopiedDrive.this.armAngle.getPosition());
        }

        /**
         * Creates a new ArmPowers from the given powers.
         * @param elevation The power with which to move the arm. Positive means up, negative means down.
         * @param angle The angle to move the arm to. Must be in the range [0.0, 1.0].
         * **/
        public ArmPowers(double elevation, double angle) {
            this.elevation = elevation;
            this.angle = angle;
        }

        public double getElevation() {return this.elevation;}
        public double getAngle() {return this.angle;}

        public void setElevation(double value) {this.elevation = value;}
        public void setAngle(double value) {this.angle = value;}

        /**
         * Clamps both the elevation and angle to their respective ranges.
         * **/
        public void clamp() {
            if (this.elevation > 1.0f) this.elevation = 1.0f;
            if (this.angle > 1.0f) this.angle = 1.0f;

            if (this.elevation < -1.0f) this.elevation = -1.0f;
            if (this.angle < 0.0f) this.angle = 0.0f;
        }

        /**
         * Applies these powers to the robot's motors.
         * **/
        public void apply() {
            NotCopiedDrive.this.armAngle.setPosition(this.angle);
            //Zero doesn't do anything, so neither branch triggers on zero.
            if (this.elevation > 0.0f) NotCopiedDrive.this.armUp.setPower(this.elevation);
            else if (this.elevation < 0.0f) NotCopiedDrive.this.armDown.setPower(-this.elevation);
        }
    }
    #endif

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    #if USE_ARM
    public CRServo armUp;
    public CRServo armDown;
    public Servo armAngle;
    #endif

    /**
     * Initialises the robot's hardware. Dead simple.
     * **/
    public void initHardware() {
        this.frontLeft = this.hardwareMap.get(DcMotor.class, "drivefl");
        this.backLeft = this.hardwareMap.get(DcMotor.class, "drivebl");
        this.frontRight = this.hardwareMap.get(DcMotor.class, "drivefr");
        this.backRight = this.hardwareMap.get(DcMotor.class, "drivebr");

        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        #if USE_ARM
        this.armUp = this.hardwareMap.get(CRServo.class, "armup");
        this.armDown = this.hardwareMap.get(CRServo.class, "armdown");
        this.armAngle = this.hardwareMap.get(Servo.class, "armangle");
        #endif
    }

    /**
     * Returns the motor powers calculated from the gamepad.
     * @return The motor powers, which can be directly applied to the motors with one function call.
     * **/
    public MotorPowers getMotorPowers() {
        double axial = -this.gamepad1.left_stick_x;
        double lateral = this.gamepad1.left_stick_y;
        double yaw = this.gamepad1.right_stick_x;
        boolean slow = this.gamepad1.a;

        MotorPowers ret = new MotorPowers(
                axial + lateral + yaw,
                axial - lateral - yaw,
                axial - lateral + yaw,
                axial + lateral - yaw
        );
        if (slow) ret.scale(0.25f);
        ret.clamp();
        return ret;
    }

    #if USE_ARM
    /**
     * Returns the arm powers calculated from the gamepad.
     * @return The arm powers, which can be directly applied to the motors with one function call.
     * **/
    public ArmPowers getArmPowers() {
        boolean up = this.gamepad1.dpad_up;
        boolean down = this.gamepad1.dpad_down;
        boolean left = this.gamepad1.dpad_left;
        boolean right = this.gamepad1.dpad_right;

        double elevation;
        //Either neither is being pressed or both are being pressed. Either way, we don't want anything to happen.
        if (up == down) elevation = 0.0f;
        //Only up is being pressed: move it up.
        else if (up) elevation = 1.0f;
        //Only down is being pressed: move it down.
        else elevation = -1.0f;

        double angle = this.armAngle.getPosition();
        //See above for why this is required.
        if (left != right) {
            if (left) angle += 0.01f;
            else angle -= 0.01f;
        }

        ArmPowers ret = new ArmPowers(elevation, angle);
        ret.clamp();
        return ret;
    }
    #endif

    //NO PASTING
    @Override
    public void runOpMode() throws InterruptedException {
        this.initHardware();
        this.waitForStart();

        while (this.opModeIsActive()) {
            MotorPowers drivePowers = this.getMotorPowers();
            drivePowers.apply();

            #if USE_ARM
            ArmPowers armPowers = this.getArmPowers();
            armPowers.apply();
            #endif
        }

        //Brakes the robot.
        new MotorPowers().apply();
        #if USE_ARM
        new ArmPowers().apply();
        #endif
    }
}
