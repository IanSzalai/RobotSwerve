
package frc.robot;

public final class RobotMap {

    public static final class map_Controllers {

        public static final int DRIVER = 0;
    }

    public static final class map_Drivetrain {

        // CAN IDS 
        public static final int FRONT_LEFT_DRIVE = 0; // left front drive
        public static final int FRONT_LEFT_TURN = 1;  // left front steer

        public static final int FRONT_RIGHT_DRIVE = 2; // right front drive
        public static final int FRONT_RIGHT_TURN = 3; // right front steer

        public static final int BACK_LEFT_DRIVE = 4; // left back drive
        public static final int BACK_LEFT_TURN = 5; // left back steer

        public static final int BACK_RIGHT_DRIVE = 6; // right back drive
        public static final int BACK_RIGHT_TURN = 7; // right back steer

        // ANALOG INPUT PORTS
        public static final int FRONT_LEFT_TURN_ENCODER = 0; // left front
        public static final int FRONT_RIGHT_TURN_ENCODER = 1; // right front
        public static final int BACK_LEFT_TURN_ENCODER = 2; // left back
        public static final int BACK_RIGHT_TURN_ENCODER = 3; // right back
    }

}
