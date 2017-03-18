package org.firstinspires.ftc.teamcode.testops;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.ROUSAutoHardware_WithServos;

/**
 * Created by ROUS on 3/17/2017.
 */

@TeleOp(name = "Test.Flipper.Servo", group = "Test")
//@Disabled
public class TeleOpTestFlipperServo extends LinearOpMode {
    // Use a Pushbot's hardware
    ModernRoboticsI2cGyro gyro = null;
    //ColorSensor sensorRGB;
    @Override
    public void runOpMode() throws InterruptedException {

        ROUSAutoHardware_WithServos robot = new ROUSAutoHardware_WithServos();
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

         //double ServoFlipper = 0.8; //equals bottom
        // double ServoButtonPusher = 0.0; //equals in position
        // boolean AButtonPreviousState = false;
        double UP = .3;
        double DOWN = .9;
        double IN = .3;
        double OUT = .9;
        // double Aim;


    /* Initialize the hardware variables.
     * The init() method of the hardware class does all the work here
     */
        robot.init(hardwareMap);
        robot.button.setPosition(IN);
        robot.servo.setPosition(DOWN);
        sleep(1000);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        /** Wait for the game to start (driver presses PLAY) */
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /**
             * Controller 2  mapping starts here
             * */

            //Button Press

            if (gamepad2.left_bumper) {
                robot.servo.setPosition(UP);
                telemetry.addData("flipper", "UP");
            } else {
                robot.servo.setPosition(DOWN);
                telemetry.addData("flipper", "DOWN");
            }
            telemetry.update();
            idle();
            sleep(200);
        }
    }

}



