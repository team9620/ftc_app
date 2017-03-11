package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ROUS on 3/3/2017.
 * Initializes and interacts with the gyro sensor
 */
public class Gyro {

    public static final String TAG = "Gyro";
    public ModernRoboticsI2cGyro gyro = null;                                          // Additional Gyro device

    /**Initialize ColorSensor for Op Mode.*/
    public void initializeForOpMode(LinearOpMode op, HardwareMap hwMap) throws InterruptedException {

        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        op.telemetry.addData(TAG, "Calibrating Gyro");
        op.telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (!op.isStopRequested() && gyro.isCalibrating()) {
            op.sleep(50);
            op.idle();
        }

        op.telemetry.addData(TAG, "Gyro is Calibrated.");
        addTelemetryData(op);
        op.telemetry.update();
    }

    public double getCurrentDeflectionDeg() {
        return (double) gyro.getIntegratedZValue();
    }

    public double getCurrentDeflectionRad() {
        return Math.toRadians(getCurrentDeflectionDeg());
    }

    public void resetToZero(){
        gyro.resetZAxisIntegrator();
    }

    public void addTelemetryData( LinearOpMode op ) {
        op.telemetry.addData(TAG, "Gyro Heading = %d", getCurrentDeflectionDeg());
    }

}
