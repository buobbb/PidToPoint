package pidToPoint;

import com.qualcomm.robotcore.util.ElapsedTime;

import config.PidConstants;

public class PID {

    double kP = PidConstants.kP;
    double kI = PidConstants.kI;
    double kD = PidConstants.kD;

    double target = 0;
    double encoderPosition = 0;
    double error = 0;
    double out = 0;

    double derivative = 0;
    double integralSum = 0;

    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public boolean isAtTarget(double currentCoordinate) {
        return Math.abs(currentCoordinate - target) <= 0.1;
    }

    public double calculate(double currentCoordinate) {
        encoderPosition = currentCoordinate;

        error = target - encoderPosition;

        derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();

        out = kP * error + kI * integralSum + kD * derivative;

        lastError = error;

        timer.reset();

        return out;
    }
}
