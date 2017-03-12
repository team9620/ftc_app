package org.firstinspires.ftc.teamcode.prototype;

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
 * Example Autonomous Op Mode.
 *
 * starts at Blue Center against the wall.
 * drives to an assumed shooting position, calculates direction and distance
 * for optimal shot, drives to location, and shoots two balls
 * it then continues on the the wall to the Wheels target for a button push,
 * Then on to the other wall to the legos target for same.
 * finally it backs up to hit the ball and park on the ramp.
 */
@Autonomous(name="AutoOp : Blue.Center.Shoot2.Wheels.Legos", group="Autonomous")
//@Disabled
public class AutoOpBlueCtrShoot2WheelsLegos extends LinearOpMode {

    public static final String TAG = "AutoOp : Blue.Center.Shoot2.Wheels.Legos";

    public static final String TAG_INIT = "INIT";

    // vuforia field position tracker
    Tracker vfTrack = null;
    // simple field position tracker
    SimpleCoordinateTracker scTrack = null;
    // tick count tracker.
    TickCountTracker tcTrack = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /**Initialize the gyro*/
        Gyro gyro = new Gyro();
        gyro.initializeForOpMode(this, hardwareMap);

        /**Initialize the color sensor*/
        ColorSensor color = new ColorSensor();
        color.initializeForOpMode(this, hardwareMap);

        /**Initialize the color sensor*/
        ODSSensor ods = new ODSSensor();
        ods.initializeForOpMode(this, hardwareMap);

        /**
         * field and bot dimensions in mm - recommended because vuforia trackables are in mm
         * so using mm simplifies the math.  Use Convert for convenience so you can specify in uint of choice.
         *
         * NOTE: this diagram needs to be updated once we identify proper locations,
         * as do the rotation angles and translation distances below.
         *
         *
         *
         *                            -X
         *     red             gears          tools
         *         +--------------O----+---------O---------+
         *         ||  /               |                  /|
         *         ||/                 |                /  |
         *         |                   |             /     |
         *  +----+ |                   |          /        O legos
         *  |red | |                   |       /           |
         *  |    | |                   |    /              |
         *  |    | |                +--+--+                |
         *  | -Y | +----------------+  +  +----------------+   Y
         *  |    | |                +--+--+                |
         *  |    | |              /    |                   O wheels
         *  |    | |           /       |                   |
         *  +----+ |        /          |                   |
         *         |     /             |                   |
         *         |  /                |                 /||
         *         |/                  |               /  ||
         *         +-------------------+-------------------+
         *                  +---------------------+           blue
         *                  |          X   blue   |
         *                  +---------------------+
         *
         *
         *
         */
        /** Initialize vuforia*/
        vfTrack = new Tracker();
        vfTrack.intializefunction(telemetry, Tracker.PhoneOnBot);

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        scTrack = new SimpleCoordinateTracker();
        scTrack.setPositionAndDirectionDeg(Field.BLUE_POSITION2, 180.0);

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

        /** Start vuforia tracking the data sets we care about. */
        vfTrack.beginTracking(); // may need to delay this until close to target...

        ////////////////////////////////////////////////////////////////////////////////////////
        // driving starts here...
        //

        /**
         * calculate drive along x-axis 18 inches forward
         */
        DirectionDistance move1 = new DirectionDistance( -180, 18.0);
        Vector2d ShootPos = scTrack.CalculatePosition(move1); // calculates relative to current position
        drive.DriveStraight(0.1, 18.0, 5.0);

        /**
         * This is a simple calc to figure out the path to the center vortex
         * from current robot position.
         *
         * It calculates the optimized angle to turn and
         * the optimal distance to drive forward/backward to shoot.
         *
         * calculate turn and drive to optimal shoot location.
         * */
        // when subtracting vectors it is always head-tail.
        DirectionDistance observeBlueCenterVortex = Vector2d.Subtract( Field.BLUE_CENTER_VORTEX_XY, tcTrack.coordinate).asDirectionDistance();
        //Turn to face the vortex and drive to shoot position
        drive.EncoderDrive( 0.1, observeBlueCenterVortex, 10.0 );

        //////////////////////////////////////////
        // shoot code here
        //////////////////////////////////////////

        /**
         * calculate turn and drive to beacon approach.
         * */
        Vector2d faceWheelsOffsetPos = new Vector2d( Field.BLUE_WHEELS_BEACON_XY ).add( -10, -10 );
        DirectionDistance driveToWheels = Vector2d.Subtract( faceWheelsOffsetPos, tcTrack.coordinate).asDirectionDistance(); // head-tail
        //Turn and drive to next location for approach to wheels target
        drive.EncoderDrive( 0.1, driveToWheels, 10.0 );

        /**update tracking from vuforia observation*/
        VuforiaTarget obs = vfTrack.updateAndGetCurrentObservation();
        if ( null != obs ) {
            scTrack.setPositionAndDirection(obs);
            tcTrack.initialize(scTrack, drive.leftMotor.getCurrentPosition(), drive.rightMotor.getCurrentPosition());
        }

        // calculate appropriate turn to be facing parallel to the wall in the -x direction
        double breakLeftDeltaDeg = 180.0 - Math.toDegrees(scTrack.direction);
        drive.BreakTurnLeft(0.1, breakLeftDeltaDeg, 10.0);

        // calculate distance to line from our current x location
        double xWheelsLineDist = Field.BLUE_WHEELS_BEACON_XY.x - scTrack.coordinate.x;
        drive.DriveStraight(0.1, xWheelsLineDist, 10.0);

        // this is where we would use osd to look for the line as we drive.  That means a special
        // osd based drive command needs to be written

        // check color
        if ( color.getColor() && !color.isBlue() ){
            // optional move forward to other side of beacon
            double xBeaconOtherSide = (Field.BLUE_WHEELS_BEACON_XY.x+(4.5/2.0)) - scTrack.coordinate.x;
            drive.DriveStraight(0.1, xBeaconOtherSide, 10.0);
        }
        // safety check check color
        if ( color.getColor() && color.isBlue() ){
            // press button 3x with 1/2 return and 250ms sleep between
        }

        //Calculate next move - this is a shortcut because it is an example
        // You need ot stop short, loook for the line, then do similar calcs as above
        DirectionDistance legosBeaconApproach = Vector2d
                .Subtract(Field.BLUE_LEGOS_BEACON_XY, scTrack.coordinate)
                .add( -3.0, 0.0 )
                .asDirectionDistance();
        //Turn and drive to next location for approach to wheels target
        drive.EncoderDrive( 0.1, legosBeaconApproach, 10.0 );

        // repeat the line detection and color check- optional- drive- check press

        // calculate reverse break turn to back into ball

        // calculate reverse direction distance to drive to knock off the ball and park on the center

        // driving ends here.
        ////////////////////////////////////////////////////////////////////////////////////////

        /** if there's any time left loop */
        while (opModeIsActive()) {
            // just updating telemetry from sensors
            gyro.addTelemetryData(this);
            color.addTelemetryData(this);
            ods.addTelemetryData(this);
            telemetry.update();
            idle();
            sleep(50);
        }
    }
}
