package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.fieldtracking.DirectionDistance;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;
import org.firstinspires.ftc.teamcode.prototype.Drive;
import org.firstinspires.ftc.teamcode.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.sensors.EvaluateColorSensor;
import org.firstinspires.ftc.teamcode.sensors.eColorState;

/**
 * Created by ROUS on 3/11/2017.
 */
@Autonomous(name="Blue.2.Beacon", group="AutoOp")
//@Disabled
public class Blue2Beacon extends LinearOpMode{

    public static final String TAG = "Blue.2.Beacon";

    com.qualcomm.robotcore.hardware.ColorSensor sensorRGB;

    @Override
    public void runOpMode() throws InterruptedException {

        /**Initialize the color sensor first... we'll need it for the */
        ColorSensor colorSensor = new ColorSensor();
        colorSensor.initializeForOpMode(this, hardwareMap);

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        /** Initialize DriveCommands*/
        sensorRGB = hardwareMap.colorSensor.get("color");
        DriveCommands drive = new DriveCommands();
        drive.initializeForOpMode( this, hardwareMap, Field.BLUE_POSITION4, 135.0);
        telemetry.addData("SCTrack",drive.scTrack.formatAsString());

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        ////////////////////////////////////////////////////////////////////////////////////////
        // driving starts here...
        //

        /**
         * calculate drive along x-axis 18 inches forward
         */
        //DirectionDistance move1 = new DirectionDistance( 135, 46.5);
        //Vector2d ShootPos = scTrack.CalculatePosition(move1); // calculates relative to current position
        drive.DriveStraight(0.5, 46.5, 10.0);
        //drive.EncoderDrive(0.3, move1, 6.0);
        telemetry.addData("SCTrack",drive.scTrack.formatAsString());
        telemetry.update();

        // calculate the angle we need to turn based on target direction - current direction
        double newdirection = 180.0-Math.toDegrees(drive.scTrack.direction);
        drive.BreakTurnLeft(0.2, newdirection, 5.0);
        telemetry.addData("SCTrack",drive.scTrack.formatAsString());
        telemetry.update();

        // calculate the drive to line distance plus 1/2 the separation between the beacons
        DirectionDistance lineDrive = Field.BLUE_WHEELS_LINE_POS
                .added( Field.BEACON_BUTTON_WIDTH, 0.0)
                .subtract( drive.scTrack.coordinate)
                .asDirectionDistance();
        // the drive command will stop when it finds the line so it should be about
        // 3 inches short of the requested distance.
        boolean bFoundWheelsWhiteLine = drive.OdsDriveStraightToWhiteLine(0.1, 24, 5.0);
        telemetry.addData("SCTrack",drive.scTrack.formatAsString());
        telemetry.addData("ODS", bFoundWheelsWhiteLine ? "true" : "false" );
        telemetry.update();

        if ( bFoundWheelsWhiteLine ){

            sleep(1000);
            idle();
            sleep(1000);
            telemetry.addData("EVAL", EvaluateColorSensor.FormatAsString(EvaluateColorSensor.Evaluate(sensorRGB)));
            telemetry.update();
            Boolean Blue = EvaluateColorSensor.EvaluateColor(sensorRGB, eColorState.blue);
            Boolean Red = EvaluateColorSensor.EvaluateColor(sensorRGB, eColorState.red);
            drive.BeaconEvalBlue(Blue, Red, .3, .9);

        }

        // calculate the drive to line distance plus 1/2 the separation between the beacons
        //drive to lego
        DirectionDistance legoLineApproach = Field.BLUE_LEGOS_LINE_POS
                .added( Field.BEACON_BUTTON_WIDTH, 0.0)
                .subtract( drive.scTrack.coordinate)
                .asDirectionDistance();
        drive.DriveStraight(0.3, legoLineApproach.distIn, 5.0);
        // the drive command will stop when it finds the line so it should be about
        // 3 inches short of the requested distance.
        boolean bFoundLegoWhiteLine = drive.OdsDriveStraightToWhiteLine(0.1, 12, 5.0);
        telemetry.addData("SCTrack",drive.scTrack.formatAsString());
        telemetry.addData("ODS", bFoundLegoWhiteLine ? "true" : "false" );
        telemetry.update();

        if ( bFoundLegoWhiteLine ){

            sleep(1000);
            idle();
            sleep(1000);
            telemetry.addData("EVAL", EvaluateColorSensor.FormatAsString(EvaluateColorSensor.Evaluate(sensorRGB)));
            telemetry.update();
            Boolean Blue = EvaluateColorSensor.EvaluateColor(sensorRGB, eColorState.blue);
            Boolean Red = EvaluateColorSensor.EvaluateColor(sensorRGB, eColorState.red);
            drive.BeaconEvalBlue(Blue, Red, .3, .9);

        }

        /** if there's any time left loop */
        while (opModeIsActive()) {
            // just updating telemetry from sensors
            idle();
            sleep(1000);
        }

    }
}
