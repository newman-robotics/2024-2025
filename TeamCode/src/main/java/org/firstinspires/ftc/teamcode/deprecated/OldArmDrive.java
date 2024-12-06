package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.deprecated.DeprecatedUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

//Whatever the name is will appear in the driver hub select display
@TeleOp(name="OldArmDrive")
public class OldArmDrive extends LinearOpMode {
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
        double stick = AutoUtil.parseGamepadInputAsDouble(GlobalConstants.ARM_CLAW_INPUT);

        boolean armElevation = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.OLD_ARM_ELEVATION_MODIFIER);
        boolean armElbow = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.OLD_ARM_ELBOW_MODIFIER);
        boolean clawWrist = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.OLD_CLAW_WRIST_MODIFIER);
        boolean clawIntake = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.OLD_CLAW_INTAKE_MODIFIER);

        //if multiple are true, don't do anything
        if ((armElevation && armElbow) || (armElevation && clawWrist) || (armElevation && clawIntake) || (armElbow && clawWrist) || (armElbow && clawIntake) || (clawWrist && clawIntake)) {
            armElevation = false;
            armElbow = false;
            clawWrist = false;
            clawIntake = false;
        }
        //X ? Y : Z ==== if X is true then do Y, but if not do Z
        this.hardware.setArmPowers(armElevation ? stick : 0, armElbow ? (int)(stick * GlobalConstants.ARM_ELBOW_TICK_MODIFIER) : 0);
        this.hardware.setClawPowers(clawWrist ? stick * GlobalConstants.CLAW_WRIST_POSITION_MODIFIER : 0, clawIntake ? stick : 0);
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

