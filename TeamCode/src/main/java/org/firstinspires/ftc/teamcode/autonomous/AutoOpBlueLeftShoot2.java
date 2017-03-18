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

@Autonomous(name="Blue.Left.Shoot.2", group="AutoOp")
//@Disabled
public class AutoOpBlueLeftShoot2 extends LinearOpMode {
    public static final String TAG = "Blue.Left.Shoot.2";
    //static public double FLIPPER_UP = 0.3;
    //static public double FLIPPER_DOWN = 0.9;

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
        drive.DriveStraight(0.4, 6, 5);

        //calculate the turn and drive distance to the blue center vortex
        DirectionDistance faceCenterVortex = Field.BLUE_CENTER_VORTEX_XY.
                subtracted(drive.scTrack.coordinate).asDirectionDistance();
        faceCenterVortex.distIn -= 36;
        drive.EncoderDrive(0.4, faceCenterVortex, 10);

        //time to shoot
        drive.Shoot(drive.FLIPPER_UP, drive.FLIPPER_DOWN);

        //calculate a drive to the ball and park on the center
        faceCenterVortex = Field.BLUE_CENTER_VORTEX_XY.
                subtracted(drive.scTrack.coordinate).asDirectionDistance();
        drive.EncoderDrive(0.4, faceCenterVortex, 10);

        /** if there's any time left loop */
        while (opModeIsActive()) {
            // just updating telemetry from sensors
            idle();
            sleep(1000);
        }
    }
}
