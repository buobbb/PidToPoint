package pidToPoint;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    double kP = 0.025;
    double kI = 0.0001;
    double kD = 0.002;

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
