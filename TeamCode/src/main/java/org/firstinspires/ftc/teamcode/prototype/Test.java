package org.firstinspires.ftc.teamcode.prototype;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivetrain.DriveCommands;
import org.firstinspires.ftc.teamcode.drivetrain.ROUSAutoHardware_WithServos;
import org.firstinspires.ftc.teamcode.sensors.EvaluateColorSensor;
import org.firstinspires.ftc.teamcode.sensors.eColorState;

/**
 * Created by Connor on 2/9/2017.
 */
@Autonomous(name="!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", group="Pushbot")
@Disabled
public class Test extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("test", "test");
        telemetry.update();

    }

}
