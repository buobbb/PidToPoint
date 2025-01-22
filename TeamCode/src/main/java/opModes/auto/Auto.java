package opModes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

/*



* o autonomie ce merge pana la high chamber, apoi la human, apoi la basket
* aceasta formeaza un triunghi si poate fi rulata in continuu
*
*
*
*
*
* */

@Config
@Autonomous
public class Auto extends LinearOpMode {

    MecanumDrive drive;
    Pose2D startPose, chamberPose, humanPose, basketPose;
    Telemetry telemetry;

    public static double speed = 0.6;

    public enum STATES{
        CHAMBER,
        HUMAN,
        BASKET,
        MOVING
    }
    STATES CS = STATES.CHAMBER, NS = STATES.CHAMBER;

    public static double chamberX = 50, chamberY = 0, chamberHeading = 0;
    public static double humanX = 10, humanY = -50, humanHeading = 90;
    public static double basketX = 10, basketY = 50, basketHeading = 90;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        chamberPose = new Pose2D(DistanceUnit.CM, chamberX, chamberY, AngleUnit.DEGREES, chamberHeading);
        humanPose = new Pose2D(DistanceUnit.CM, humanX, humanY, AngleUnit.DEGREES, humanHeading);
        basketPose = new Pose2D(DistanceUnit.CM, basketX, basketY, AngleUnit.DEGREES, basketHeading);

        drive.setStartPose(startPose);
        drive.setSpeed(speed);

        waitForStart();

        while(opModeIsActive() && !gamepad1.a) {

            switch (CS){
                case CHAMBER:
                    drive.setTargetPose(chamberPose);
                    CS = STATES.MOVING;
                    NS = STATES.HUMAN;
                    break;

                case HUMAN:
                    drive.setTargetPose(humanPose);
                    CS = STATES.MOVING;
                    NS = STATES.BASKET;
                    break;

                case BASKET:
                    drive.setTargetPose(basketPose);
                    CS = STATES.MOVING;
                    NS = STATES.CHAMBER;
                    break;

                case MOVING:
                    if(drive.isAtTarget()){
                        CS = NS;
                    }
                    break;
            }

            telemetry.addData("Current State", CS);
            telemetry.addData("Next State", NS);
            telemetry.addData("X error", drive.errorX());
            telemetry.addData("Y error", drive.errorY());

            drive.update();
            telemetry.update();
        }
    }
}
