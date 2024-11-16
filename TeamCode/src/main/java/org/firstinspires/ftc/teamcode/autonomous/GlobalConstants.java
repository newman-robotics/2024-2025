package org.firstinspires.ftc.teamcode.autonomous;

public class GlobalConstants {
    public enum GamepadInput {
        LEFT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_X,
        RIGHT_STICK_Y,

        BUTTON_A,
        BUTTON_B,
        BUTTON_X,
        BUTTON_Y,

        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,

        LEFT_TRIGGER,
        RIGHT_TRIGGER,

        LEFT_BUMPER,
        RIGHT_BUMPER,

        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON,
    }

    public static final String FRONT_LEFT_MOTOR_NAME = "drivefl";
    public static final String FRONT_RIGHT_MOTOR_NAME = "drivefr";
    public static final String BACK_LEFT_MOTOR_NAME = "drivebl";
    public static final String BACK_RIGHT_MOTOR_NAME = "drivebr";

    public static final String ARM_VERTICAL_MOTOR_NAME = "actuator";
    public static final String ARM_ELBOW_MOTOR_NAME = "elbow";

    public static final String CLAW_WRIST_MOTOR_NAME = "wrist";
    public static final String CLAW_INTAKE_MOTOR_NAME = "intake";

    public static final String WEBCAM_NAME = "webcam";

    public static final double GAMEPAD_THRESHOLD = 0.7;
    public static final double SLOW_FACTOR = 0.2;

    public static final GamepadInput AXIAL = GamepadInput.LEFT_STICK_X;
    public static final GamepadInput LATERAL = GamepadInput.LEFT_STICK_Y;
    public static final GamepadInput YAW = GamepadInput.RIGHT_STICK_X;
    public static final GamepadInput SLOW = GamepadInput.BUTTON_A;

    public static final GamepadInput ARM_CLAW_INPUT = GamepadInput.RIGHT_STICK_Y;
    public static final GamepadInput ARM_ELEVATION_MODIFIER = GamepadInput.LEFT_BUMPER;
    public static final GamepadInput ARM_ELBOW_MODIFIER = GamepadInput.RIGHT_BUMPER;
    public static final GamepadInput CLAW_INTAKE_MODIFIER = GamepadInput.LEFT_TRIGGER;
    public static final GamepadInput CLAW_WRIST_MODIFIER = GamepadInput.RIGHT_TRIGGER;

    public static final double ARM_ELBOW_TICK_MODIFIER = 100.0;
    public static final double ARM_ELBOW_SPEED = 0.8;

    public static final boolean CLAW_IS_INSTALLED = true;

    //Forward
    //Sideways
    //Backwards

    public static final long MOTION_MILLIS_TILE_FULL_SPEED = 1000;
    public static final double MOTION_SPEED = 1.0;
    public static final long MOTION_MILLIS_TILE = (long)Math.floor(MOTION_MILLIS_TILE_FULL_SPEED / MOTION_SPEED);
}
