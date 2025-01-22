package pidToPoint;

import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import config.Motors;

public class MecanumDrive {

    DcMotorEx leftFrontMotor;
    DcMotorEx rightFrontMotor;
    DcMotorEx leftRearMotor;
    DcMotorEx rightRearMotor;

    Localizer localizer;

    PID xControl = new PID();
    PID yControl = new PID();
    HeadingPID headingControl = new HeadingPID();

    public double currentX = localizer.getPose().getX();
    public double currentY = localizer.getPose().getY();

    double startX = 0, startY = 0, startHeading = 0;

    double x = startX;
    double y = startY;
    double heading = startHeading;

    public double speed = 0.5;

    public MecanumDrive(HardwareMap hardwareMap) {

        leftFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.leftFrontMotor);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, Motors.rightFrontMotor);
        leftRearMotor = hardwareMap.get(DcMotorEx.class, Motors.leftRearMotor);
        rightRearMotor = hardwareMap.get(DcMotorEx.class, Motors.rightRearMotor);

        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void setStartPose(Pose2D pose){
        startX = pose.getX(DistanceUnit.CM);
        startY = pose.getY(DistanceUnit.CM);
        startHeading = pose.getHeading(AngleUnit.DEGREES);
    }

    public void setTargetPose(Pose2D pose){
        xControl.target = pose.getX(DistanceUnit.CM);
        yControl.target = pose.getY(DistanceUnit.CM);
        headingControl.target = pose.getHeading(AngleUnit.DEGREES);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    public boolean isAtTarget(){
        return xControl.isAtTarget(localizer.getPose().getX()) && yControl.isAtTarget(localizer.getPose().getY()) && headingControl.isAtTarget(localizer.getPose().getHeading());
    }

    public Pose2D getCurrentPose(){
        return new Pose2D(DistanceUnit.CM, localizer.getPose().getX(), localizer.getPose().getY(), AngleUnit.DEGREES, localizer.getPose().getHeading());
    }

    public Pose2D getTargetPose(){
        return new Pose2D(DistanceUnit.CM, xControl.target, yControl.target, AngleUnit.DEGREES, headingControl.target);
    }

    public double errorX(){
        return getTargetPose().getX(DistanceUnit.CM) - getCurrentPose().getX(DistanceUnit.CM);
    }

    public double errorY(){
        return getTargetPose().getY(DistanceUnit.CM) - getCurrentPose().getY(DistanceUnit.CM);
    }

    public void update(){
        //pt calculat pid
        x = xControl.calculate(localizer.getPose().getX());
        y = yControl.calculate(localizer.getPose().getY());
        heading = headingControl.calculate(localizer.getPose().getHeading());

        //pt field centric
        double xRotated = x * Math.cos(Math.toRadians(localizer.getPose().getHeading())) - y * Math.sin(Math.toRadians(localizer.getPose().getHeading()));
        double yRotated = x * Math.sin(Math.toRadians(localizer.getPose().getHeading())) + y * Math.cos(Math.toRadians(localizer.getPose().getHeading()));

        leftFrontMotor.setPower((xRotated + yRotated + heading) * speed);
        leftRearMotor.setPower((xRotated - yRotated + heading) * speed);
        rightFrontMotor.setPower((xRotated - yRotated - heading) * speed);
        rightRearMotor.setPower((xRotated + yRotated - heading) * speed);

        //se opreste cand ajunge la target
        if(isAtTarget()){
            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);
        }
    }


}
