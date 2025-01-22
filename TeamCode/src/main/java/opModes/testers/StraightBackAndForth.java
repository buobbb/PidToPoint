package opModes.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.Localizer;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pidToPoint.MecanumDrive;

@Config
@Autonomous
public class StraightBackAndForth extends LinearOpMode {

    MecanumDrive drive;
    Pose2D startPose, endPose;
    Telemetry telemetry;

    public static double DISTANCE = 50;

    public static boolean sameSpeed = true;
    public static double normalSpeed = 0.6;
    public static double forwardSpeed = 0.7;
    public static double backwardSpeed = 0.4;

    public enum STATES{
        FORWARD,
        MOVING,
        BACKWARD
    }
    STATES CS = STATES.FORWARD, NS = STATES.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        endPose = new Pose2D(DistanceUnit.CM, DISTANCE, 0, AngleUnit.DEGREES, 0);

        drive.setStartPose(startPose);
        if(sameSpeed) drive.setSpeed(normalSpeed);

        waitForStart();

        while(opModeIsActive() && !gamepad1.a) {

            switch (CS){
                case FORWARD:
                    if(!sameSpeed) drive.setSpeed(forwardSpeed);
                    drive.setTargetPose(endPose);
                    CS = STATES.MOVING;
                    NS = STATES.BACKWARD;
                    break;

                case MOVING:
                    if(drive.isAtTarget()){
                        CS = NS;
                    }
                    break;

                case BACKWARD:
                    if(!sameSpeed) drive.setSpeed(normalSpeed);
                    drive.setTargetPose(startPose);
                    CS = STATES.MOVING;
                    NS = STATES.FORWARD;
                    break;
            }
        }
        telemetry.addData("X error", drive.errorX());
        telemetry.addData("Y error", drive.errorY());

        drive.update();
        telemetry.update();
    }
}
