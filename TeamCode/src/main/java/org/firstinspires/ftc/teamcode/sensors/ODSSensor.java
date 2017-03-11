package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by ROUS on 3/10/2017.
 * Initializes and interacts with the color sensor
 */
public class ODSSensor {

    //Instance of OpticalDistanceSensor
    public static final String TAG = "ODS";
    public OpticalDistanceSensor ods = null;

    //Raw value is between 0 and 1
    public double odsReadingRaw;

    // odsReadingRaw to the power of (-0.5)
    public double odsReadingLinear;

    /**Initialize ColorSensor for Op Mode.*/
    public void initializeForOpMode(LinearOpMode op, HardwareMap hwMap) throws InterruptedException {

        op.telemetry.addData(TAG, "Initializing ODS Sensor");
        op.telemetry.update();

        //identify the port of the ODS and motors in the configuration file
        ods = hwMap.opticalDistanceSensor.get("ods");

        odsReadingRaw = ods.getRawLightDetected() ;
        odsReadingLinear = Math.pow(odsReadingRaw, -0.5);

        op.telemetry.addData(TAG, "ODS initialized.");
        addTelemetryData(op);
        op.telemetry.update();
    }

    public double getLinearReading() {
        odsReadingRaw = ods.getRawLightDetected() ;
        odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
        return odsReadingLinear;
    }

    public void addTelemetryData( LinearOpMode op ) {
        op.telemetry.addData(TAG, "Raw: %04f",    ods.getRawLightDetected());
        op.telemetry.addData(TAG, "Normal: %.04f", ods.getLightDetected());
    }

}
