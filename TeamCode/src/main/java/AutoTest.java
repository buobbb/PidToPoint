import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
@Config
public class AutoTest extends LinearOpMode {

    Movement drive;
    Localizer localizer;
    public static double x = 40, y = 0, h = 0;
    public static double speed = 0.3;

    public enum STATES{
        IDLE,
        FORWARD,
        MOVING,
        BACKWARD
    }
    STATES CS = STATES.IDLE, NS = STATES.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Movement(hardwareMap, localizer);

        waitForStart();

        while(opModeIsActive()){
            switch (CS){
                case IDLE:
                    CS = STATES.FORWARD;
                    break;

                case FORWARD:
                    drive.setTarget(x, y, h);
                    drive.setSpeed(speed);
                    CS = STATES.MOVING;
                    NS = STATES.BACKWARD;
                    break;

                case MOVING:
                    if(drive.isAtTarget(0.5, 0.5)){
                        CS = NS;
                    }
                    break;

                case BACKWARD:
                    drive.setTarget(0, 0, 0);
                    drive.setSpeed(speed);
                    CS = STATES.MOVING;
                    NS = STATES.FORWARD;
                    break;
            }

            drive.update();
        }
    }
}
