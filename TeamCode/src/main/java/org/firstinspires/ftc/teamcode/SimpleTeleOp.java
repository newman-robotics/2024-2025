package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;
import org.firstinspires.ftc.teamcode.external.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="Simple Tele Op")
public class SimpleTeleOp extends LinearOpMode {
    private AutoUtil.Drivetrain drivetrain;
    //private DcMotor actuator;
    private DcMotor elbow;
    private DcMotor linearSlide;
    private Servo claw;

    private final AutoUtil.ToggleSwitch clawState = new AutoUtil.ToggleSwitch();
    private final AutoUtil.ToggleSwitch slow = new AutoUtil.ToggleSwitch();

    private GoBildaPinpointDriver odometry;

    private void initHardware() {
        this.drivetrain = AutoUtil.Drivetrain.initAndGet(this.hardwareMap);

        //this.actuator = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_VERTICAL_MOTOR_NAME);
        this.elbow = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_ELBOW_MOTOR_NAME);
        this.linearSlide = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_LINEAR_SLIDE_NAME);
        this.claw = this.hardwareMap.get(Servo.class, GlobalConstants.CLAW_MOTOR_NAME);

        this.elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.elbow.setTargetPosition(20);

        this.odometry = this.drivetrain.odometry;
    }

    private void report() {
        AutoUtil.ChainTelemetry.assertAndGet()
                //.add("Runtime", System.currentTimeMillis() - this.initTime)
                .add("Elbow ticks", this.elbow.getCurrentPosition())
                .add("Elbow target", this.elbow.getTargetPosition())
                .add("ODOMETRY")
                .add("X", DistanceUnit.INCH.fromMm(this.odometry.getPosX()))
                .add("Y", DistanceUnit.INCH.fromMm(this.odometry.getPosY()))
                .add("Theta", DistanceUnit.INCH.fromMm(this.odometry.getHeading()))
                .update();
    }

    private void tick() {
        this.drivetrain.setFromGamepad();

        int elbowInput = (int)AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ELBOW_UP), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ELBOW_DOWN));
        double linearSlideInput = AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_LINEAR_SLIDE_UP), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_LINEAR_SLIDE_DOWN));

        this.clawState.update(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_CLAW));
        this.slow.update(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.SLOW));

        if (this.slow.getState()) {
            linearSlideInput *= GlobalConstants.SLOW_FACTOR;

            elbowInput *= (int)(GlobalConstants.ARM_ELBOW_TICK_MODIFIER * GlobalConstants.SLOW_FACTOR);
        } else elbowInput *= GlobalConstants.ARM_ELBOW_TICK_MODIFIER;

        elbowInput += AutoUtil.clamp(this.elbow.getTargetPosition(), GlobalConstants.ELBOW_TICK_LOWER_BOUND, GlobalConstants.ELBOW_TICK_UPPER_BOUND);

        //this.actuator.setPower(actuator);
        RobotLog.i(String.format(Locale.UK, "Elbow input: %d", elbowInput));
        this.elbow.setTargetPosition(elbowInput);
        this.linearSlide.setPower(linearSlideInput / 1.5);
        this.claw.setPosition(this.clawState.getState() ? 1. : 0.);

        this.report();
    }

    public void runOpMode() {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        //I think it's equivalent to `while (this.opModeInInit());`
        this.waitForStart();

        this.initHardware();

        //while (this.opModeInInit()) this.report();

        while (this.opModeIsActive()) this.tick();

        this.drivetrain.setPowers(0, 0, 0, 0);
    }
}
