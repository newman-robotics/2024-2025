package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

@TeleOp(name="Simple Tele Op")
public class SimpleTeleOp extends LinearOpMode {
    private AutoUtil.Drivetrain drivetrain;
    private DcMotor actuator;
    private DcMotor elbow;
    private DcMotor linearSlide;
    private Servo claw;

    private void initHardware() {
        this.drivetrain = AutoUtil.Drivetrain.initAndGet(this.hardwareMap);

        this.actuator = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_VERTICAL_MOTOR_NAME);
        this.elbow = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_ELBOW_MOTOR_NAME);
        this.linearSlide = this.hardwareMap.get(DcMotor.class, GlobalConstants.ARM_LINEAR_SLIDE_NAME);
        this.claw = this.hardwareMap.get(Servo.class, GlobalConstants.CLAW_MOTOR_NAME);

        this.elbow.setPower(1.);
        this.elbow.setTargetPosition(0);
        this.elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void tick() {
        this.drivetrain.setFromGamepad();

        double actuator = AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ACTUATOR_UP), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ACTUATOR_DOWN));
        int elbow = (int)AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ELBOW_UP), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_ELBOW_DOWN));
        double linearSlide = AutoUtil.ternaryXOR(AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_LINEAR_SLIDE_UP), AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_LINEAR_SLIDE_DOWN));
        boolean claw = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.INPUT_CLAW_OPEN);

        if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.SLOW)) {
            actuator *= GlobalConstants.SLOW_FACTOR;
            linearSlide *= GlobalConstants.SLOW_FACTOR;

            elbow *= (GlobalConstants.ARM_ELBOW_TICK_MODIFIER * GlobalConstants.SLOW_FACTOR);
        } else elbow *= GlobalConstants.ARM_ELBOW_TICK_MODIFIER;

        this.actuator.setPower(actuator);
        this.elbow.setTargetPosition(this.elbow.getTargetPosition() + elbow);
        this.linearSlide.setPower(linearSlide);
        this.claw.setPosition(claw ? 1. : 0.);
    }

    public void runOpMode() {
        AutoUtil.setOpMode(this);
        AutoUtil.ChainTelemetry.init(this.telemetry);

        this.initHardware();
        this.waitForStart();

        while (this.opModeIsActive()) this.tick();

        this.drivetrain.setPowers(0, 0, 0, 0);
    }
}
