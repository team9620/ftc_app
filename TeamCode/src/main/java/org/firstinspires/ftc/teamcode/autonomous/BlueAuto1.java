package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.fieldtracking.DirectionDistance;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.Tracker;
import org.firstinspires.ftc.teamcode.fieldtracking.Util;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;
import org.firstinspires.ftc.teamcode.fieldtracking.VuforiaTarget;
import org.firstinspires.ftc.teamcode.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.ODSSensor;

/**
 * Created by ROUS on 3/11/2017.
 */

@Autonomous
//@Disabled
public class BlueAuto1 extends LinearOpMode{

    public static final String TAG = "AutoOp : Blue.Auto1";

    public static final String TAG_INIT = "INIT";

    // simple field position tracker
    SimpleCoordinateTracker scTrack = null;
    // tick count tracker.
    TickCountTracker tcTrack = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        scTrack = new SimpleCoordinateTracker();
        scTrack.setPositionAndDirectionDeg(Field.BLUE_POSITION4, 135.0);

        /**Initialize TickCountTracker*/
        tcTrack = new TickCountTracker();
        tcTrack.initialize(scTrack, 0,0);

        /** Initialize DriveCommands*/
        DriveCommands drive = new DriveCommands();
        drive.initializeForOpMode( this, hardwareMap, tcTrack, scTrack );

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
        drive.DriveStraight(0.2, 46.5, 5.0);
        //drive.EncoderDrive(0.2, move1, 5.0);

        double newdirection=180.0-Math.toDegrees(scTrack.direction);
        drive.BreakTurnLeft(0.2, newdirection, 5.0);


        /** if there's any time left loop */
        while (opModeIsActive()) {
            // just updating telemetry from sensors
            idle();
            sleep(30000);
        }

    }
}
