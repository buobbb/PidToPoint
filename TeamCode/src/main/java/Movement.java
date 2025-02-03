import static java.lang.Math.signum;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Movement {
    Localizer localizer;

    double integral;
    double lastErrorHeading , lastErrorDrive;
    double hP = 3, hI = 0, hD = 0.02;
    double dP = 0.1, dD = 0.03;
    double sP = 0.08, sD = 0.001;

    double targetX, targetY, targetH;

    DcMotorEx fr, fl, br, bl;

    double speed = 1;

    Vector2d driveVector, rotatedVector;

    public Movement(HardwareMap map, Localizer localizer){
        fl = map.get(DcMotorEx.class, "m1");
        bl = map.get(DcMotorEx.class, "m0");
        fr = map.get(DcMotorEx.class, "m2");
        br = map.get(DcMotorEx.class, "m3");

        this.localizer = localizer;

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);
        fr.setDirection(DcMotorEx.Direction.FORWARD);
        br.setDirection(DcMotorEx.Direction.FORWARD);

        fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTarget(double x, double y, double h){
        targetX = x;
        targetY = y;
        targetH = h;
        driveVector = new Vector2d(targetX - localizer.getPose().getX(), targetY - localizer.getPose().getY());
        rotatedVector = driveVector.rotateBy(localizer.getPose().getHeading());
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public boolean isAtTarget(double tolerance, double toleranceH){
        return Math.abs(localizer.getPose().getX() - targetX) < tolerance && Math.abs(localizer.getPose().getY() - targetY) < tolerance && Math.abs(localizer.getPose().getHeading() - targetH) < toleranceH;
    }

    public double pidHeading(double target, double kP, double kI, double kD, double current) {
        double error = target - current;
        integral += error;
        double derivative = error - lastErrorHeading;

        if(error > Math.PI){
            error -= Math.PI * 2;
        }
        else if(error < -Math.PI){
            error += Math.PI * 2;
        }

        double correction = (error * kP) + (integral * kI) + (derivative * kD);
        lastErrorHeading = error;

        return correction;
    }

    public double pfdDrive(double kP, double kD, double kF, double error){
        double derivative = error - lastErrorDrive;

        double correction = (error * kP) + (derivative * kD);

        correction += signum(error) * kF;
        lastErrorDrive = error;

        return correction;
    }

    public void update(){
        localizer.update();

        double inputTurn = pidHeading(targetH, hP, hI, hD, localizer.getPose().getHeading());
        double driveCorrection = pfdDrive(dP, dD, 0, rotatedVector.getX());
        double strafeCorrection = pfdDrive(sP, sD, 0, rotatedVector.getY());

        fr.setPower((driveCorrection - strafeCorrection - inputTurn) * speed);
        fl.setPower((driveCorrection + strafeCorrection + inputTurn) * speed);
        br.setPower((driveCorrection + strafeCorrection - inputTurn) * speed);
        bl.setPower((driveCorrection - strafeCorrection + inputTurn) * speed);
    }

}
