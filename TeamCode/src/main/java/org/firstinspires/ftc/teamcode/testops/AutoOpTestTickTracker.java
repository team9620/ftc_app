package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;

/**
 * ROUS
 */
@Autonomous(name = "Auto : Tick Count Tracker", group = "Test")
@Disabled
public class AutoOpTestTickTracker extends LinearOpMode {

    public static final String TAG = "Test: Tick Count Tracker - RC only OpMode";

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private SimpleCoordinateTracker fieldPos = null;
    private TickCountTracker tcTrack = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /**
         *                            -X
         *     red             gears          tools
         *         +--------------O(-72,0)-------O---------+
         *         ||  /               |                  /|
         *         ||/                 |                /  |
         *         |                   |             /     |
         *  +----+ |                   |          /     FP O legos
         *  |red | |                   |       /         . |
         *  |    | |                   |    /            . |
         *  |    | |                +--+--+              x |
         *  | -Y (0,-72)------------+ 0,0 +-------------.(0,72) Y
         *  |    | |                +--+--+            .   |
         *  |    | |              /    |            x      O wheels
         *  |    | |           /       |       .           |
         *  +----+ |        /            .                 |
         *         |     /         x   |                   |
         *         |  /            .   |                 /||
         *         |/             IP   |               /  ||
         *         +----------------(72,0)-----------------+
         *                  +---------------------+           blue
         *                  |          X   blue   |
         *                  +---------------------+
         *
         *
         *
         */

        /** Initialize field _position and orientation.*/
        fieldPos = new SimpleCoordinateTracker();
        fieldPos.setPositionAndDirectionDeg(72.0, 0.0, 180.0);
        fieldPos.shiftPositionByXY(-10, -8.0);
        tcTrack = new TickCountTracker();
        int left = 0;
        int right = 0;
        tcTrack.initializeRad(fieldPos.coordinate.x, fieldPos.coordinate.y, fieldPos.direction, left,right);
        int step = (int)TickCountTracker.COUNTS_PER_INCH/10;
        // show bot position on DS phone
        telemetry.addData("Bot", "%s", tcTrack.formatAsString() );

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start driving");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();
        int turnTicks = 0;
        int rightTarget = 0;
        int leftTarget = 0;

        // to drive 12 inches
        int targetTicks = (int)(12.0 * TickCountTracker.COUNTS_PER_INCH);
        do {
            left += targetTicks/25;
            right += targetTicks/25;
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while ((left + right)/2 < targetTicks );

        fieldPos.moveOnCurentHeading(12.0); // drive forward 12 inches
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----moveOnCurrentHeading(12.0) complete-------------------------------");

        turnTicks = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH/2.0)*Math.toRadians(17.5));
        rightTarget = right - turnTicks;
        leftTarget = left + turnTicks;
        do
        {
            left += turnTicks/25;
            right -= turnTicks/25;
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while (right > rightTarget &&
                left < leftTarget );

        fieldPos.turnRelativeDeg(-17.5);// pivot turn right 17.5 deg
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(-17.5) right complete-------------------------------");

        turnTicks = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH/2.0)*Math.toRadians(36.5));
        rightTarget = right - turnTicks;
        leftTarget = left + turnTicks;
        do
        {
            left += turnTicks/25;
            right -= turnTicks/25;
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while (right > rightTarget &&
                left < leftTarget );

        fieldPos.turnRelativeDeg(-36.5);// pivot turn right 36.5 deg
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(-36.5) right complete-------------------------------");

        turnTicks = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH/2.0)*Math.toRadians(90.0));
        rightTarget = right + turnTicks;
        leftTarget = left - turnTicks;
        do
        {
            left -= turnTicks/25;
            right += turnTicks/25;
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while (right < rightTarget &&
                left > leftTarget );

        fieldPos.turnRelativeDeg(90);// pivot turn left 90 deg
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(90) left complete-------------------------------");

        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(360.0) Arc turn right start 2x wheel width--------");
        left = 0;
        right = 0;
        tcTrack.initializeDeg(0,0,0,left,right);
        int turnTicks1 = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH*1.0)*Math.toRadians(360.0));
        int turnTicks2 = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH*2.0)*Math.toRadians(360.0));
        leftTarget = left + turnTicks2;
        rightTarget = right + turnTicks1;
        do
        {
            left += (turnTicks2/360);
            right += (turnTicks1/360);
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while (right < rightTarget &&
                left < leftTarget );

        fieldPos.moveArcTurnRightDeg(TickCountTracker.ROBOT_WHEEL_WIDTH, 360);// arc turn right 227 deg
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(360) Arc turn right end 2x wheel width--------");

        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(360) Arc turn Left start -.25x wheel width--------");
        left = 0;
        right = 0;
        tcTrack.initializeDeg(0,0,0,left,right);
        turnTicks1 = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH*-0.25)*Math.toRadians(360.0));
        turnTicks2 = (int)(TickCountTracker.COUNTS_PER_INCH * (TickCountTracker.ROBOT_WHEEL_WIDTH*0.75)*Math.toRadians(360.0));
        leftTarget = left + turnTicks1;
        rightTarget = right + turnTicks2;
        do
        {
            left += (turnTicks1/360);
            right += (turnTicks2/360);
            tcTrack.updateTicks(left, right);
            // show bot position on DS phone
            telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
            telemetry.update();
            sleep(50);
            idle();
        } while (right < rightTarget &&
                left > leftTarget ); // > because left wheel is moving backwards

        fieldPos.moveArcTurnLeftDeg(TickCountTracker.ROBOT_WHEEL_WIDTH, 360);// arc turn right 360 deg
        // record info to the robot log
        RobotLog.ii("TickCountTracker", "-----turnRelativeDeg(360) Arc turn left end -.25x wheel width--------");

        // show bot position on DS phone
        telemetry.addData("Bot", "%s", tcTrack.formatAsString() );
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
