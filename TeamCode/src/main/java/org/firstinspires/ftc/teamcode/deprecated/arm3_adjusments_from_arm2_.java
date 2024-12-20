package org.firstinspires.ftc.teamcode.deprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

@Disabled
@TeleOp(name = "arm3")
public class arm3_adjusments_from_arm2_ extends LinearOpMode {

    @Override
    public void runOpMode () throws InterruptedException {
        AutoUtil.setOpMode(this);

        CRServo top_rope_tightener = hardwareMap.get(CRServo.class, GlobalConstants.ARM3_TOP_SERVO_TIGHTENER_NAME);
        Servo bottom_claw = hardwareMap.get(Servo.class, GlobalConstants.ARM3_BOTTOM_SERVO_CLAW_NAME);
        DcMotor top_base = hardwareMap.get(DcMotor.class, GlobalConstants.ARM3_BASE_NAME);


        this.waitForStart();

        while(this.opModeIsActive()){
            boolean tighten = gamepad1.dpad_right;
            boolean untighten = gamepad1.dpad_left;
            boolean arm_up = gamepad1.dpad_up;
            boolean arm_down = gamepad1.dpad_down;
            boolean claw_close = gamepad1.right_bumper;
            boolean claw_open = gamepad1.left_bumper;

            if (tighten) top_rope_tightener.setPower(top_rope_tightener.getPower() + 0.1);
            else if (untighten) top_rope_tightener.setPower(top_rope_tightener.getPower() - 0.1);
            else if (arm_up) top_base.setPower(top_base.getPower() + 0.1);
            else if (arm_down) top_base.setPower(top_base.getPower() - 0.1);
            else if (claw_close) bottom_claw.setPosition(bottom_claw.getPosition() + 0.1);
            else if (claw_open) bottom_claw.setPosition(bottom_claw.getPosition() - 0.1);
        }

    }

}
