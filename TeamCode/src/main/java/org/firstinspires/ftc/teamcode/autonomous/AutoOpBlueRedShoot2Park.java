package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.fieldtracking.DirectionDistance;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.prototype.Drive;
import org.firstinspires.ftc.teamcode.sensors.ColorSensor;

/**
 * Created by ROUS on 3/17/2017.
 */

@Autonomous(name="Blue.Left.Park.Shoot.2", group="AutoOp")
//@Disabled
public class AutoOpBlueRedShoot2Park extends LinearOpMode {
    public static final String TAG = "Blue.Left.Park.Shoot.2";

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
        drive.initializeForOpMode(this, hardwareMap, Field.BLUE_POSITION1, 180.0);
        telemetry.addData("SCTrack", drive.scTrack.formatAsString());

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        /*
        *Driving starts here
         */
        //driving 6 inches forward so the turn is parallel to the blue line
        drive.DriveStraight(0.5, 32, 10);

        //time to shoot
        drive.Shoot(drive.FLIPPER_UP, drive.FLIPPER_DOWN);

        //drive to park on square
        drive.DriveStraight(0.5, 36, 10);

        /** if there's any time left loop */
        while (opModeIsActive()) {
            // just updating telemetry from sensors
            idle();
            sleep(1000);
        }
    }
}
