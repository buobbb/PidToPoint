package opModes.testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pidToPoint.MecanumDrive;

@Config
public class StraightBackAndForth extends LinearOpMode {

    MecanumDrive drive;
    Pose2D startPose, endPose;

    public static double DISTANCE = 50;

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

        startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        endPose = new Pose2D(DistanceUnit.CM, DISTANCE, 0, AngleUnit.DEGREES, 0);

        drive.setStartPose(startPose);
        drive.setSpeed(0.6);

        waitForStart();

        while(opModeIsActive() && !gamepad1.a) {
            switch (CS){
                case FORWARD:
                    drive.followTrajectory(endPose);
                    CS = STATES.MOVING;
                    NS = STATES.BACKWARD;
                    break;

                case MOVING:
                    if(drive.isAtTarget()){
                        CS = NS;
                    }
                    break;

                case BACKWARD:
                    drive.followTrajectory(startPose);
                    CS = STATES.MOVING;
                    NS = STATES.FORWARD;
                    break;
            }
        }
    }
}
