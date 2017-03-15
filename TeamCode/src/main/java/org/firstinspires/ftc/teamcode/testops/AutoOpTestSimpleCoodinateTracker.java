package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.fieldtracking.DirectionDistance;
import org.firstinspires.ftc.teamcode.fieldtracking.Field;
import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TurnCalc;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;

/**
 * Created by ROUS on 3/14/2017.
 */
@Autonomous(name = "Simple Coordinate Tracker", group = "Test")
//@Disabled
public class AutoOpTestSimpleCoodinateTracker extends LinearOpMode {

    public static final String TAG = "Test: SimpleCoordinateTracker - RC only OpMode";

    @Override
    public void runOpMode() throws InterruptedException {

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start driving");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        SimpleCoordinateTracker calc = new SimpleCoordinateTracker();
        calc.setPositionAndDirectionDeg(Field.BLUE_POSITION4, 135.0);
        Vector2d expected = Field.BLUE_POSITION4.added( DirectionDistance.CreateVector2dDeg(135,46.5) );

        calc.moveOnCurentHeading(46.5);
        Vector2d scDiff = expected.subtracted( calc.coordinate );
        double ad = 135.0 - Math.toDegrees(calc.direction);
        double dd = scDiff.asDirectionDistance().distIn;
        telemetry.addData("Diff","Dist: %.04f Ang: %.04f", dd, ad);
        telemetry.addData("Calc",calc.formatAsString());
        telemetry.addData("Expected", "%s %.04fd", expected.formatAsString(), 135.0);
        telemetry.update();
        sleep(5000);

        // calculate the angle we need to turn based on target direction - current direction
        double turnAngle = 180.0-Math.toDegrees(calc.direction);
        calc.moveArcTurnLeftDeg(TurnCalc.WheelSpace/2.0,turnAngle);
        Vector2d dir = Vector2d.UnitVectorDeg(135.0);
        Vector2d center = dir.PerpendicularCCW().multiplied(TurnCalc.WheelSpace/2.0);
        Vector2d radial = dir.PerpendicularCW()
                .multiplied(TurnCalc.WheelSpace/2.0)
                .rotatedByCCWAngleDeg(turnAngle);
        expected.add(center).add(radial);
        scDiff = expected.subtracted( calc.coordinate );
        ad = 180.0 - Math.toDegrees(calc.direction);
        dd = scDiff.asDirectionDistance().distIn;
        telemetry.addData("Diff","Dist: %.04f Ang: %.04f", dd, ad);
        telemetry.addData("Calc",calc.formatAsString());
        telemetry.addData("Expected", "%s %.04fd", expected.formatAsString(), 135.0);
        telemetry.update();
        sleep(5000);

        // calculate the drive to line distance plus 1/2 the separation between the beacons
        Vector2d wheelsLineApproach = Field.BLUE_WHEELS_LINE_POS
                .added( Field.BEACON_BUTTON_WIDTH, 0.0)
                .subtract( calc.coordinate );

        calc.setDirectionDeg(180);
        calc.moveOnCurentHeading(wheelsLineApproach.x);
        expected.add( DirectionDistance.CreateVector2dDeg(180,wheelsLineApproach.x) );
        scDiff = expected.subtracted( calc.coordinate );
        ad = 180.0 - Math.toDegrees(calc.direction);
        dd = scDiff.asDirectionDistance().distIn;
        telemetry.addData("Diff","Dist: %.04f Ang: %.04f", dd, ad);
        telemetry.addData("Calc",calc.formatAsString());
        telemetry.addData("Expected", "%s %.04fd", expected.formatAsString(), 135.0);
        telemetry.update();
        sleep(5000);

        while (opModeIsActive()) {
            idle();
            sleep(500);
        }
    }
}