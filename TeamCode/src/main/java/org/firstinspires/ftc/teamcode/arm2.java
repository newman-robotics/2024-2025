package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.autonomous.AutoUtil;
import org.firstinspires.ftc.teamcode.autonomous.GlobalConstants;

@teleop(name = "arm2")
public class arm2 extends LinearOpMode {


    private CRServo rope_tightener = null;

    private CRServo rope_tightener2 = null;
    private CRServo upy_downly = null;

    private CRServo grippy = null;




    @Override
    public void runOpMode() throws InterruptedException {
        AutoUtil.setOpMode(this);

        //Servo 1:  rope_tightener   == intake
        //Servo 2:  rope_tightener2  == wrist
        //Servo 3:` upy_downly       == arm_p1
        //Servo 4:  grippy           == Rope_arm
        rope_tightener = hardwareMap.get(CRServo.class, "intake");
        rope_tightener = hardwareMap.get(CRServo.class, "wrist");
        upy_downly = hardwareMap.get(CRServo.class, "arm_p1");
        grippy = hardwareMap.get(CRServo.class, "Rope_arm)");


        this.waitForStart();



        while (this.opModeIsActive()) {
            boolean want_to_lift = gamepad1.right_bumper;
            boolean want_to_drop_down = gamepad1.left_bumper;


            boolean hand_closes = gamepad1.b;

            float fason = gamepad1.left_trigger;
            float fason2 = gamepad1.right_trigger;



            // setter we will see what happens
            upy_downly.setDirection(DcMotorSimple.Direction.FORWARD);

            grippy.setDirection(DcMotorSimple.Direction.FORWARD);

            rope_tightener.setDirection(DcMotorSimple.Direction.FORWARD);


            double input_detector_for_upydowny = AutoUtil.ternaryXOR(
                    AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_TRIGGER)

                    ,AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_TRIGGER)

            );

            upy_downly.setPower(input_detector_for_upydowny / 2.);



            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.LEFT_BUMPER)) {
                grippy.setPower(-.5);
            }
            else{
                grippy.setPower(.5);
            }

            if (AutoUtil.parseGamepadInputAsBoolean(GlobalConstants.GamepadInput.RIGHT_BUMPER)){
                rope_tightener.setPower(.5);
            }
            else {
                rope_tightener.setPower(-.5);
            }
        }









    }
}