package org.firstinspires.ftc.teamcode.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.sensors.eColorState;

/**
 *
 */
public class EvaluateColorSensor {

    static private int colorThreshold = 32; /** color comparrison value in range of 1 to 128 */

    static public String FormatAsString( eColorState e ) {
        switch( e )
        {
            case red : { return "[Red]"; }
            case green : { return "[Green]"; }
            case blue : { return "[Blue]"; }
            default : { return "[Unknown]"; }
        }
    }

    static boolean FuzzyEqual( float v1, float v2, float tol ) {
        float delta = Math.abs(v2 - v1);
        return ( delta <= tol );
    }

    static public float[] GetHSVFromSensor( ColorSensor sensorRGB ) {
        /**  is an array that will hold 0.0 - 1.0 values of red, green blue. */
        float hsvValues[] = {0f,0f,0f};
        /** convert the RGB values to HSV values. */
        Color.RGBToHSV((sensorRGB.red() * 255) / 800, (sensorRGB.green() * 255) / 800, (sensorRGB.blue() * 255) / 800, hsvValues);
        return hsvValues;
    }

    /** evaluate color channels and return simple color assessment */
    static public eColorState Evaluate(ColorSensor sensorRGB ) {
        return EvaluateHSV( GetHSVFromSensor( sensorRGB ));
    }
    /** evaluate color channels and return simple color assessment */
    static public eColorState EvaluateHSV( final float[] hsvValues ) {
        /**
         * For blue color a hue range from 221° to 240°
         * For red color a hue range from 355° to 10°
         * For green color a hue range from 81° to 140°
         */
        if ( hsvValues[1] < 0.05 ) /** asuming satuation value < 5% is effectively white/gray */
        {
            return eColorState.unknown;
        } else {
            if ( FuzzyEqual( hsvValues[0], 5.0f, 8.0f ) || FuzzyEqual( hsvValues[0], 355.0f, 5.0f )) {
                return eColorState.red;
            } else {
                if ( FuzzyEqual( hsvValues[0], 230.5f, 9.5f) ) {
                        return eColorState.blue;
                } else {
                    if ( FuzzyEqual( hsvValues[0], 110.5f, 29.5f) ) {
                        return eColorState.green;
                    } else {
                        return eColorState.unknown;
                    }
                }
            }
        }
    }
    /** evaluate for specific color channel dominance */
    static public boolean EvaluateColorHSV( final float[] hsvValues, eColorState checkColor )
    {
        return ( checkColor == EvaluateHSV( hsvValues ) );
    }

    /** evaluate for specific color channel dominance */
    static public boolean  EvaluateColor( ColorSensor sensorRGB, eColorState checkColor )
    {
        return ( checkColor == Evaluate( sensorRGB ) );
    }
}
