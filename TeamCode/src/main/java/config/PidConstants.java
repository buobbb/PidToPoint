package config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PidConstants {

    public static double kP = 0.025;
    public static double kI = 0.0001;
    public static double kD = 0.002;

    public static double kPHeading = 2;
    public static double kIHeading = 0.0001;
    public static double kDHeading = 0.002;

}
