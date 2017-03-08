package org.firstinspires.ftc.teamcode.sensors;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ROUS on 3/3/2017.
 * Initializes and interacts with the color sensor
 */
public class ColorSensor extends EvaluateColorSensor {
    static final long Pause_Time = 25;
    static final long Pause_Time_int= 250;
    static final long Pause_Time_Telemetry= 2500;

    public com.qualcomm.robotcore.hardware.ColorSensor sensorRGB = null;
    public DeviceInterfaceModule cdim = null;
    static final private int LED_CHANNEL = 5;
    public View relativeLayout = null;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    static public final float emptyValues[] = {0F, 0F, 0F};
    public float hsvValues[] = emptyValues;
    public eColorState eColor = eColorState.unknown;
    public float values[] = emptyValues;

    // values is a reference to the hsvValues array.
    public boolean bLedOn = true;

    /**Initialize ColorSensor for Op Mode.*/
    public void initializeForOpMode(LinearOpMode op, HardwareMap hw ) throws InterruptedException {

        op.telemetry.addData(">", "Initializing Color Sensor");
        op.telemetry.update();

        resetColor();

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.

        relativeLayout = ((Activity) hw.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);

        // bLedOn represents the state of the LED.


        // get a reference to our DeviceInterfaceModule object.
        cdim = hw.deviceInterfaceModule.get("dim");

        op.sleep(Pause_Time_int);

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        op.sleep(Pause_Time_int);

        // get a reference to our ColorSensor object.
        sensorRGB = hw.colorSensor.get("color");

        op.sleep(Pause_Time_int);

        // turn the LED on in the beginning, just so user will know that the sensor is active.

        cdim.setDigitalChannelState(LED_CHANNEL, bLedOn);

        op.sleep(Pause_Time_int);

        // wait for the start button to be pressed.
        // send the info back to driver station using telemetry function.

        op.telemetry.addData(">", "Color Sensor is Active");
        op.telemetry.addData("LED", bLedOn ? "On" : "Off");
        op.telemetry.addData("Clear", sensorRGB.alpha());
        op.telemetry.addData("Red  ", sensorRGB.red());
        op.telemetry.addData("Green", sensorRGB.green());
        op.telemetry.addData("Blue ", sensorRGB.blue());
        op.telemetry.addData("Hue", hsvValues[0]);

        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        op.telemetry.update();

        op. sleep(Pause_Time_Telemetry);
    }

    void resetColor() {
        hsvValues = emptyValues;
        values = emptyValues;
        eColor = eColorState.unknown;
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
    }

    boolean getColor() {
        // update previous state variable.
        // convert the RGB values to HSV values.
        hsvValues = GetHSVFromSensor( sensorRGB );
        eColor = EvaluateColorSensor.EvaluateHSV( hsvValues );
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });

        return eColorState.unknown != eColor;
    }

    boolean isBlue() { return eColorState.blue == eColor; }
    boolean isRed() { return eColorState.red == eColor; }
    boolean isColor( eColorState checkColor ) { return checkColor == eColor; }

    void addTelemetryData( LinearOpMode op ) {
        op.telemetry.addData(">", "Color Sensor is Active");
        op.telemetry.addData("LED", bLedOn ? "On" : "Off");
        op.telemetry.addData("Clear", sensorRGB.alpha());
        op.telemetry.addData("Red  ", sensorRGB.red());
        op.telemetry.addData("Green", sensorRGB.green());
        op.telemetry.addData("Blue ", sensorRGB.blue());
        op.telemetry.addData("Hue", hsvValues[0]);
    }
}
