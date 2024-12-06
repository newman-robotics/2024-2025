package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.FieldWorld;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="Modus Operandi")
public class NotCopiedDrive extends LinearOpMode {
    public AutoUtil.Hardware hardware;

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
        double stick = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_CLAW_INPUT);

        boolean armLower = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_LOWER_MODIFIER);
        boolean armUpper = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_UPPER_MODIFIER);
        boolean armElbow = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_ELBOW_MODIFIER);
        boolean claw = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.CLAW_MODIFIER);

        //if multiple are true, don't do anything
        if (!AutoUtil.notMoreThanOne(armLower, armUpper, armElbow, claw)) {
            armLower = false;
            armUpper = false;
            armElbow = false;
            claw = false;
        }
        this.hardware.setArmPowers(armLower ? stick : 0, armUpper ? stick : 0, armElbow ? (int)(stick * GlobalConstants.ARM_ELBOW_TICK_MODIFIER) : 0);
        this.hardware.setClawPowers(claw ? stick * GlobalConstants.CLAW_WRIST_POSITION_MODIFIER : 0);
    }

    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);
        this.hardware = AutoUtil.Hardware.init(this.hardwareMap);

        this.waitForStart();

        while (this.opModeIsActive()) {
            this.updateDrivetrain();
            this.updateArmClaw();
        }

        this.hardware.zeroOut();
    }
}

