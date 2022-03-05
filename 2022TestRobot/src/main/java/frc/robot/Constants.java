package frc.robot;

public final class Constants {
    /*
     * The class that holds constant values
     * All names should be all caps with _ between to denote a static (unchangeable)
     * value
     * If it is a general constant (not a port/id), put between this comment and the
     * next class
     * If it is a port/id, put it into its class based on type (CAN, DIO, AIO, etc)
     */
    public static final double QUICK_TURN_DEADBAND = 0.05;
    public static final double QUICK_TURN_MULTIPLIER = 0.75;
    public static final double ARCADE_DRIVE_DEADBAND = 0.025;

    public static final double AUTO_DRIVE_SPEED = 0.25;

    public static final double FALCON_MAX_RPM = 6000;

    public final class PID {
        public static final double DRIVE_PROPORTIONAL_PRACTICE     = 0.017;
        public static final double DRIVE_INTEGRAL_PRACTICE         = 0.000;
        public static final double DRIVE_DERIVATIVE_PRACTICE       = 0.001;
        public static final double DRIVE_TOLERANCE_PRACTICE        =   1.0;

        public static final double ROTATE_PROPORTIONAL_PRACTICE    = 0.0090;
        public static final double ROTATE_INTEGRAL_PRACTICE        = 0.0005;
        public static final double ROTATE_DERIVATIVE_PRACTICE      = 0.0030;
        public static final double ROTATE_TOLERANCE_PRACTICE       =    1.0;

        public static final double DRIVE_PROPORTIONAL_COMPETITION  = 0.017;
        public static final double DRIVE_INTEGRAL_COMPETITION      = 0.000;
        public static final double DRIVE_DERIVATIVE_COMPETITION    = 0.001;
        public static final double DRIVE_TOLERANCE_COMPETITION     =   1.0;

        public static final double ROTATE_PROPORTIONAL_COMPETITION = 0.0090;
        public static final double ROTATE_INTEGRAL_COMPETITION     = 0.0005;
        public static final double ROTATE_DERIVATIVE_COMPETITION   = 0.0030;
        public static final double ROTATE_TOLERANCE_COMPETITION    =    1.0;
    }


    public final class ButtonMappings {
        /*
         * The class that holds the button mappings for an X-box controller
         * Reference the all-caps name in code to use the buttons
         * You should not need to add anything to this class, so don't touch please!
         */
        public static final int A_BUTTON = 1;
        public static final int B_BUTTON = 2;
        public static final int X_BUTTON = 3;
        public static final int Y_BUTTON = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int BACK_BUTTON = 7;
        public static final int START_BUTTON = 8;
        public static final int LEFT_JOY_BUTTON = 9;
        public static final int RIGHT_JOY_BUTTON = 10;
    }
}
