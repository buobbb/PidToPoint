package pidToPoint;

import com.qualcomm.robotcore.util.ElapsedTime;

import config.PidConstants;

public class HeadingPID {

    double kP = PidConstants.kPHeading;
    double kI = PidConstants.kIHeading;
    double kD = PidConstants.kDHeading;

    double target = 0;
    double encoderHeading = 0;
    double error = 0;
    double out = 0;

    double derivative = 0;
    double integralSum = 0;

    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public boolean isAtTarget(double currentHeading) {
        return Math.abs(currentHeading - target) <= 0.1;
    }

    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public double calculate(double currentHeading){
        encoderHeading = currentHeading;

        error = Math.toDegrees(angleWrap(Math.toRadians(target - currentHeading)));

        derivative = (error - lastError) / timer.seconds();

        integralSum += error * timer.seconds();

        out = kP * error + kI * integralSum + kD * derivative;

        lastError = error;

        timer.reset();

        return out;
    }

}

