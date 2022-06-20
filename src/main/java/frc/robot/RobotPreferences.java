package frc.robot;

import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.math.util.Units;

public final class RobotPreferences {

    public static final class pref_SwerveModule {
        public static final SN_DoublePreference wheelRadius = new SN_DoublePreference(
                "wheelRadius", Units.inchesToMeters(1.5));
        public static final SN_DoublePreference encoderCountsPerRotation = new SN_DoublePreference(
                "encoderCountsPerRotation", 4096);
        public static final SN_DoublePreference maxAngularVelocity = new SN_DoublePreference(
                "maxAngularVelocity", Math.PI);
        public static final SN_DoublePreference maxAngularAcceleration = new SN_DoublePreference(
                "maxAngularAcceleration", 2 * Math.PI);

        public static final SN_DoublePreference drivePIDControllerP = new SN_DoublePreference("drivePIDControllerP", 1);
        public static final SN_DoublePreference drivePIDControllerI = new SN_DoublePreference("drivePIDControllerI", 0);
        public static final SN_DoublePreference drivePIDControllerD = new SN_DoublePreference("drivePIDControllerD", 0);

        public static final SN_DoublePreference turnPIDControllerP = new SN_DoublePreference("turnPIDControllerP", 1);
        public static final SN_DoublePreference turnPIDControllerI = new SN_DoublePreference("turnPIDControllerI", 0);
        public static final SN_DoublePreference turnPIDControllerD = new SN_DoublePreference("turnPIDControllerD", 0);

        public static final SN_DoublePreference driveFeedforwardS = new SN_DoublePreference("driveFeedforwardS", 1);
        public static final SN_DoublePreference driveFeedforwardV = new SN_DoublePreference("driveFeedforwardV", 3);

        public static final SN_DoublePreference turnFeedforwardS = new SN_DoublePreference("turnFeedforwardS", 1);
        public static final SN_DoublePreference turnFeedforwardV = new SN_DoublePreference("turnFeedforwardV", 0.5);
        
        public static final SN_DoublePreference driveSpeedMultiplier = new SN_DoublePreference("driveSpeedMultiplier", 0.1);
        public static final SN_DoublePreference turnSpeedMultiplier = new SN_DoublePreference("turnSpeedMultiplier", 0.05);
}

    public static final class pref_Drivetrain {

        public static final SN_DoublePreference maxSpeed = new SN_DoublePreference("maxSpeed", Units.feetToMeters(20));
        public static final SN_DoublePreference maxAngularSpeed = new SN_DoublePreference("maxAngularSpeed", Math.PI);

        // figure out a better way to do this at some point
        public static final SN_DoublePreference moduleDistOut = new SN_DoublePreference(
                "frontLeftLocation", Units.inchesToMeters(30));

    }

}
