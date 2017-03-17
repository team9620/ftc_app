package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;
import org.firstinspires.ftc.teamcode.sensors.ColorSensor;
import org.firstinspires.ftc.teamcode.sensors.ODSSensor;

/**
 * ROUS
 */
@Autonomous(name = "Test DriveCommands", group = "Test")
//@Disabled
public class AutoOpTestDriveCommands extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        /**Initialize the color sensor first... we'll need it for the */
        ColorSensor colorSensor = new ColorSensor();
        colorSensor.initializeForOpMode(this, hardwareMap);

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        /** Initialize DriveCommands*/
        DriveCommands drive = new DriveCommands();
        drive.initializeForOpMode( this, hardwareMap,  new Vector2d(Field.BLUE_WHEELS_BEACON_XY).add( 6.0, 10.0 ), 180.0 );

        /**Initialize the color sensor*/
        ODSSensor ods = new ODSSensor();
        ods.initializeForOpMode(this, hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        sleep(1000);
        try {
            drive.DriveStraight(0.1, -5.3, 5.0);
        } catch(InterruptedException e){ }

        try {
            drive.DriveStraight(0.1, 5.3, 5.0);
        } catch(InterruptedException e){ }

        boolean bFoundWheelsWhiteLine = drive.OdsDriveStraightToWhiteLine(0.05, 16.0, 5.0);
        if ( bFoundWheelsWhiteLine ){
            telemetry.addData("OSD", "Found Line @ %s", drive.tcTrack.formatAsString());
            telemetry.update();
        }

        while (opModeIsActive()){

            colorSensor.getColor();
            colorSensor.addTelemetryData(this);
            telemetry.update();

            idle();
        }

    }


}
