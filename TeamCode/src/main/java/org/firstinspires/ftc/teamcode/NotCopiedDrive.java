package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

import java.security.acl.NotOwnerException;

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

        this.hardware.setArmPowers(armElevation ? stick : 0, armElbow ? stick : 0);
        this.hardware.setClawPowers(clawWrist ? stick : 0, clawIntake ? stick : 0);
    }

    @Override
    public void runOpMode() {
        AutoUtil.setOpMode(this);
        this.hardware = AutoUtil.Hardware.init(this.hardwareMap);

        this.waitForStart();

        this.hardware.setClawWristAbsolutePosition(0);

        while (this.opModeIsActive()) {
            this.updateDrivetrain();
            this.updateArmClaw();
        }

        this.hardware.zeroOut();
    }
}
