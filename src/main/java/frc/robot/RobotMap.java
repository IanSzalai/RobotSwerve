
package frc.robot;

public final class RobotMap {

    public static final class map_Controllers {

        public static final int DRIVER = 0;
    }

    public static final class map_Drivetrain {

        // CAN IDS 
        public static final int FRONT_LEFT_TURN = 1;  // left front steer
        public static final int FRONT_LEFT_DRIVE = 2; // left front drive

        public static final int BACK_LEFT_TURN = 3; // left rear steer
        public static final int BACK_LEFT_DRIVE = 4; // left rear drive

        public static final int BACK_RIGHT_TURN = 5; // right rear steer
        public static final int BACK_RIGHT_DRIVE = 6; // right rear drive

        public static final int FRONT_RIGHT_TURN = 7; // right front steer
        public static final int FRONT_RIGHT_DRIVE = 8; // right front drive

        // ANALOG INPUT PORTS
        public static final int FRONT_LEFT_TURN_ENCODER = 0; // left front
        public static final int BACK_LEFT_TURN_ENCODER = 1; // left rear
        public static final int BACK_RIGHT_TURN_ENCODER = 2; // right rear
        public static final int FRONT_RIGHT_TURN_ENCODER = 3; // right front
    }

}
