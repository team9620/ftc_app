package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;
import org.firstinspires.ftc.teamcode.sensors.ODSSensor;

/**
 *
 */
@TeleOp(name = "Test: MR ODS Drive to line", group = "SensorTest")
//@Disabled
public class AutoOpTestODSLineDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        /**Initialize the color sensor*/
        ODSSensor ods = new ODSSensor();
        ods.initializeForOpMode(this, hardwareMap);

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        SimpleCoordinateTracker scTrack = new SimpleCoordinateTracker();
        scTrack.setPositionAndDirectionDeg( new Vector2d(Field.BLUE_WHEELS_BEACON_XY).add( 6.0, 10.0 ), 180.0);

        /**Initialize TickCountTracker*/
        TickCountTracker tcTrack = new TickCountTracker();
        tcTrack.initialize(scTrack, 0,0);

        /** Initialize DriveCommands*/
        DriveCommands drive = new DriveCommands();
        drive.initializeForOpMode( this, hardwareMap, tcTrack, scTrack );

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        boolean bFoundWhiteLine = false;

        // Determine new target position, and pass to motor controller
        int newLeftTarget = drive.leftMotor.getCurrentPosition() + (int) (12.0 * drive.COUNTS_PER_INCH);
        int newRightTarget = drive.rightMotor.getCurrentPosition() + (int) (12.0 * drive.COUNTS_PER_INCH);
        drive.leftMotor.setTargetPosition(newLeftTarget);
        drive.rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        drive.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive() && drive.leftMotor.isBusy() || drive.rightMotor.isBusy() ) {

            drive.leftMotor.setPower(0.1);
            drive.rightMotor.setPower(0.1);
            drive.tcTrack.updateTicks(drive.leftMotor.getCurrentPosition(), drive.rightMotor.getCurrentPosition());

            if ( !bFoundWhiteLine && ods.getLinearReading() > 0.6 ) {
                bFoundWhiteLine = true;
                telemetry.addData("Line", "FoundLine @ %s", drive.tcTrack.coordinate.formatAsString());
            } else if ( bFoundWhiteLine && ods.getLinearReading() < 0.6 ) {
                bFoundWhiteLine = false;
                telemetry.addData("Line", "LostLine @ %s", drive.tcTrack.coordinate.formatAsString());
            } else if ( ods.getLinearReading() < 0.6 ) {
                telemetry.addData("Line", "OnLine @ %s", drive.tcTrack.coordinate.formatAsString());
            }

            // send the info back to driver station using telemetry function.
            ods.addTelemetryData(this);
            telemetry.addData("Linear",    ods.getLinearReading());
            telemetry.update();
        }
        drive.leftMotor.setPower(0.0);
        drive.rightMotor.setPower(0.0);
        drive.tcTrack.updateTicks(drive.leftMotor.getCurrentPosition(), drive.rightMotor.getCurrentPosition());

        // Stop all motion;
        drive.leftMotor.setPower(0);
        drive.rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        drive.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.opMode.sleep(100);   // optional pause after each move

        while (opModeIsActive()){
            idle();
        }

    }


}
