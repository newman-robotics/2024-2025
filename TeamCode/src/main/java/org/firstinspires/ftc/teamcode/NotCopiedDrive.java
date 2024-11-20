package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;
import org.firstinspires.ftc.teamcode.arm2;

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

        boolean armElevation = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_ELEVATION_MODIFIER);
        boolean armElbow = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.ARM_ELBOW_MODIFIER);
        boolean clawWrist = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.CLAW_WRIST_MODIFIER);
        boolean clawIntake = AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.CLAW_INTAKE_MODIFIER);

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
        try {this.hardware = AutoUtil.Hardware.get();} catch (IllegalAccessException e) {throw new RuntimeException(e);}

        this.waitForStart();

        while (this.opModeIsActive()) {
            this.updateDrivetrain();
            this.updateArmClaw();
        }

        this.hardware.zeroOut();
    }
}

