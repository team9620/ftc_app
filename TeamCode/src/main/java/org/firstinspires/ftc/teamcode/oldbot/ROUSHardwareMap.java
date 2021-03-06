package org.firstinspires.ftc.teamcode.oldbot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a ROUS robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left drive motor:         "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Motor channel:  Intake motor:             "intake"
 * Servo channel:  Left side Servo:          "left servo"
 * Servo channel:  Right side Servo:         "right servo"
 *
 * Note: the configuration of the servos is such that:
 *   As the _____ side servo approaches 1, it moves towards the pressing position (facing away from the robot  |-->).
 *   As the _____ side servo approaches 0, it moves towards the pressing position (facing away from the robot  |-->).
 */
public class ROUSHardwareMap
{
    /* Public OpMode members. */
    public DcMotor  leftMotor    = null;
    public DcMotor  rightMotor   = null;
    public DcMotor  Intake       = null;
    public Servo    PressR  = null;
    public Servo    PressL   = null;

    public final static double rHOME = 1;
    public final static double lHOME = 1;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ROUSHardwareMap() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left motor");
        rightMotor  = hwMap.dcMotor.get("right motor");
        Intake      = hwMap.dcMotor.get("intake");
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftMotor.setDirection(DcMotor.Direction.REVERSE);
        //shootMotorR.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run with encoders.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //claw = hwMap.servo.get("claw");      (Servo example)
        PressR = hwMap.servo.get("right servo");
        PressL = hwMap.servo.get("left servo");

         //PressR.setPosition(rHOME);
         //PressL.setPosition(lHOME);



    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
