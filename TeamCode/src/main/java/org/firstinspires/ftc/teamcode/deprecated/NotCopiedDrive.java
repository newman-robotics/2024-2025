package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="OPTIMAL CONTROLS")
public class NotCopiedDrive extends LinearOpMode {
    public DeprecatedUtil.Hardware hardware;
    public void updateDrivetrain() {
        double slow = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.SLOW) ? GlobalConstants.SLOW_FACTOR : 1;

        double axial = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.AXIAL) * slow;
        double lateral = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.LATERAL) * slow;
        double yaw = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.YAW) * slow;

        this.hardware.setDrivetrainPowers(
                axial + lateral + yaw,
                axial - lateral - yaw,
                axial - lateral + yaw,
                axial + lateral - yaw
        );
    }

    public void updateArmClaw() {
        double ropeTightener = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_UPPER_MODIFIER);
        double motorElbow = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_ELBOW_MODIFIER);
        double servoClaw = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.CLAW_MODIFIER);
        double servoClawDown = -AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_LOWER_MODIFIER);

        double stick = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_CLAW_INPUT);

        boolean armLower = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_LOWER_MODIFIER);
        boolean armUpper = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_UPPER_MODIFIER);
        boolean armElbow = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_ELBOW_MODIFIER);
        boolean claw = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.CLAW_MODIFIER);
        boolean xvar = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.XVARTESTNEWCODE);

        //if multiple are true, don't do anything
        if (!AutoUtil.notMoreThanOne(armLower, armUpper, armElbow, claw)) {
            armLower = false;
            armUpper = false;
            armElbow = false;
            claw = false;
        }


        if (xvar) {
            this.hardware.setArmPowers(armLower ? stick : 0, armUpper ? stick : 0, armElbow ? (int) (stick * GlobalConstants.ARM_ELBOW_TICK_MODIFIER) : 0);
            this.hardware.setClawPowers(claw ? stick * GlobalConstants.CLAW_WRIST_POSITION_MODIFIER : 0);
        } else {
            this.hardware.setArmPowers2(ropeTightener, (int) motorElbow);
            this.hardware.setClawPowers(servoClawDown + servoClaw);
        }
    }
    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);
        this.hardware = DeprecatedUtil.Hardware.init(this.hardwareMap);

        this.waitForStart();

        while (this.opModeIsActive()) {
            this.updateDrivetrain();
            this.updateArmClaw();
        }

        this.hardware.zeroOut();
    }
}

