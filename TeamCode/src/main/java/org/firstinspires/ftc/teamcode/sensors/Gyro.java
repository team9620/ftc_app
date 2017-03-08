package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ROUS on 3/3/2017.
 * Initializes and interacts with the gyro sensor
 */
public class Gyro {

    public ModernRoboticsI2cGyro gyro = null;                                          // Additional Gyro device

    /**Initialize ColorSensor for Op Mode.*/
    public void initializeForOpMode(LinearOpMode opModeIn, HardwareMap hwMap) throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        opModeIn.telemetry.addData(">", "Calibrating Gyro");
        opModeIn.telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!opModeIn.isStopRequested() && gyro.isCalibrating()) {
            opModeIn.sleep(50);
            opModeIn.idle();
        }

        opModeIn.telemetry.addData(">", "Gyro is Calibrated.");    //
        opModeIn.telemetry.update();
    }

    double getCurrentDeflectionDeg() {
        return (double) gyro.getIntegratedZValue();
    }

    double getCurrentDeflectionRad() {
        return Math.toRadians(getCurrentDeflectionDeg());
    }

    void resetToZero(){
        gyro.resetZAxisIntegrator();
    }

    void addTelemetryData( LinearOpMode op ) {
        op.telemetry.addData(">", "Gyro Heading = %d", gyro.getIntegratedZValue());
    }

}
