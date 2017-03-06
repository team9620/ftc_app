package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TurnCalc;
import org.firstinspires.ftc.teamcode.fieldtracking.Vector2d;


public class DriveCommands {

    public static final String TAG = "EncoderDrive";
    public final static String tag_left_wheel = "left motor";
    public final static String tag_right_wheel = "right motor";
    public final static String tag_left_shooter= "left shooter";
    public final static String tag_right_shooter= "right shooter";
    public final static String tag_gyro = "gyro";
    public final static String tag_ODS = "ods";
    public final static String tag_color = "color";
    public final static String tag_Flipper= "flip";
    public final static String tag_Button= "press";
    public final static String tag_Intake= "intake";

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor leftshooter = null;
    public DcMotor rightshooter = null;
    public DcMotor Intake     = null;
    public ModernRoboticsI2cGyro gyro = null;                    // Additional Gyro device
    public ColorSensor sensorRGB;
    OpticalDistanceSensor ODS;
    public Servo servo = null;
    public Servo button=null;
    static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = .625;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double Pi = 3.141592653f;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Pi);
    static final double HEADING_THRESHOLD = 10;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = .075;     // Larger is more responsive, but also less stable
    static final long Pause_Time = 50;
    double odsReadingRaw;
    double odsReadingRaw2;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;
    static double odsReadingLinear2;

    public LinearOpMode opMode   = null;
    public TickCountTracker tcTrack = null;
    public SimpleCoordinateTracker scTracker = null;

    /**
     * Initialize EncoderDrive for Op Mode.
     */
    public void initializeForOpMode(LinearOpMode opModeIn, HardwareMap hwMap, TickCountTracker tcTrackIn, SimpleCoordinateTracker scTrackerIn) throws InterruptedException {

        /**
         * Send telemetry message to signify robot waiting;
         */
        opMode.telemetry.addData("Status", "Resetting Encoders");
        opMode.telemetry.update();

        // Define and Initialize Motors
        opMode = opModeIn;
        tcTrack = tcTrackIn;
        scTracker = scTrackerIn;

        leftMotor       = hwMap.dcMotor.get(tag_left_wheel);
        rightMotor      = hwMap.dcMotor.get(tag_right_wheel);
        leftshooter     = hwMap.dcMotor.get(tag_left_shooter);
        rightshooter    = hwMap.dcMotor.get(tag_right_shooter);
        Intake          = hwMap.dcMotor.get(tag_Intake);
        gyro            = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get(tag_gyro);
        ODS             = hwMap.opticalDistanceSensor.get(tag_ODS);
        sensorRGB       = hwMap.colorSensor.get(tag_color);
        servo           = hwMap.servo.get(tag_Flipper);
        button          = hwMap.servo.get(tag_Button);
        odsReadingRaw   = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
        odsReadingLinear= Math.pow(odsReadingRaw, -0.5);


        // set initial direction
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run with encoders.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior to brake mode
        RobotLog.ii(TAG, "ZP Behavior before - Left:=%s Right:=%s"
                , ZPMToString(leftMotor.getZeroPowerBehavior())
                , ZPMToString((rightMotor.getZeroPowerBehavior())));
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.ii(TAG, "ZP Behavior after - Left:=%s Right:=%s"
                , ZPMToString(leftMotor.getZeroPowerBehavior())
                , ZPMToString((rightMotor.getZeroPowerBehavior())));


        /** Send telemetry message to indicate initial encoder pos*/
        int encoderLeft = leftMotor.getCurrentPosition();
        int encoderRight = rightMotor.getCurrentPosition();
        opMode.telemetry.addData("Encoders","@ L: %7d R:%7d", encoderLeft, encoderRight );
        opMode.telemetry.update();
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElapsedTime resetTimer = new ElapsedTime();
        resetTimer.reset();
        do{
            opMode.idle();
            encoderLeft = leftMotor.getCurrentPosition();
            encoderRight = rightMotor.getCurrentPosition();
            opMode.telemetry.addData("Encoders","@ L: %7d R:%7d", encoderLeft, encoderRight );
            opMode.telemetry.update();
            // keep looping while we have time remaining. and encoders aren't reset
        } while( (resetTimer.time() < 2.0) && (0!=encoderLeft || 0!=encoderRight) );
        // Send telemetry message to indicate successful Encoder reset
        if (0==encoderLeft && 0==encoderRight){
            opMode.telemetry.addData("Encoders","RESET@ L:%7d R:%7d", encoderLeft, encoderRight );
        } else {
            opMode.telemetry.addData("Encoders","FAIL!@ L:%7d R:%7d", encoderLeft, encoderRight );
        }
        opMode.telemetry.update();

        /**initialize tracker once encoders have been properly reset*/
        tcTrack.initialize(scTracker, encoderLeft, encoderRight );
    }

    protected void DriveCore(double speed,
                      double leftInches, double rightInches,
                      double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                //opMode.telemetry.addData("Clear", sensorRGB.alpha());
                // opMode.telemetry.addData("Red  ", sensorRGB.red());
                // opMode. telemetry.addData("Green", sensorRGB.green());
                //opMode. telemetry.addData("Blue ", sensorRGB.blue());
                opMode.telemetry.update();

                /**update tick tracker with current positions*/
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // Allow time for other processes to run.
                opMode.idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // this should be the last thing we do before return
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
        }
    }

    /**Dont use this method, use DirveStraight or one of the Turn methods*/
    public void Drive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {
            DriveCore(speed, leftInches, rightInches, timeoutS);
            // CAN'T quite calculate error here because we don't know clear intent form dist
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void DriveStraight(double speed,
                             double distInches,
                             double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {
            DriveCore(speed, distInches, distInches, timeoutS);
            scTracker.moveOnCurentHeading(distInches);
            RobotLog.ii("DriveStraight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void TurnRight(double speed,
                          float turndeg,
                          double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            double dist = TurnCalc.Turn(turndeg);
            DriveCore(speed, dist, -dist, timeoutS);

            scTracker.turnRelativeDeg(turndeg);
            RobotLog.ii("TurnRight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void BreakTurnRight(double speed,
                              float turndeg,
                              double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            DcMotor.ZeroPowerBehavior zpmOld = leftMotor.getZeroPowerBehavior();
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double dist = TurnCalc.BreakTurnDeg(turndeg);
            DriveCore(speed, dist, 0.0, timeoutS);

            leftMotor.setZeroPowerBehavior(zpmOld);

            scTracker.moveArcTurnRight(TurnCalc.WheelSpace, turndeg);
            RobotLog.ii("BreakTurnRight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void TurnLeft(double speed,
                          float turndeg,
                          double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            double dist = TurnCalc.Turn(turndeg);
            DriveCore(speed, -dist, dist, timeoutS);

            scTracker.turnRelativeDeg(turndeg);
            RobotLog.ii("TurnLeft", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void BreakTurnLeft(double speed,
                         float turndeg,
                         double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            DcMotor.ZeroPowerBehavior zpmOld = leftMotor.getZeroPowerBehavior();
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double dist = TurnCalc.BreakTurnDeg(turndeg);
            DriveCore(speed, 0.0, dist, timeoutS);

            leftMotor.setZeroPowerBehavior(zpmOld);

            scTracker.moveArcTurnLeft(TurnCalc.WheelSpace, turndeg);
            RobotLog.ii("BreakTurnLeft", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    // this is replaced by the new TurnRight().  You should delete this method.
    public void OldTurnRight(double speed,
                          float turndeg,
                          double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (TurnCalc.Turn(turndeg) * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (TurnCalc.Turn(turndeg) * COUNTS_PER_INCH * -1f);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());

                opMode.telemetry.update();
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());


                // Allow time for other processes to run.
                opMode.idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scTracker.turnRelativeDeg(turndeg);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            RobotLog.ii("OldTurnRight", "Error: %s, DA: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    // this is replaced by the new TurnLeft().  You should delete this method.
    public void OldTurnLeft(double speed,
                         float turndeg,
                         double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (TurnCalc.Turn(turndeg) * COUNTS_PER_INCH * -1f);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (TurnCalc.Turn(turndeg) * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.update();
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // Allow time for other processes to run.
                opMode.idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            scTracker.moveOnCurentHeading(turndeg);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            RobotLog.ii("OldTurnLeft", "Error: %s, DA: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed,
                          double distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, .5);
            leftMotor.setPower(TurnCalc.OffL(gyro.getIntegratedZValue(), angle));
            rightMotor.setPower(TurnCalc.OffR(gyro.getIntegratedZValue(), angle));

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > .3) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(TurnCalc.OffL(gyro.getIntegratedZValue(), angle));
                rightMotor.setPower(TurnCalc.OffR(gyro.getIntegratedZValue(), angle));

                // Display drive status for the driver.
                opMode.telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                opMode.telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.update();
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            scTracker.moveOnCurentHeading(distance);
            RobotLog.ii("DriveStraight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn( double speed, double angle) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // keep looping while we have time remaining.
            // keep looping while we are still active, and not on heading.
            while (opMode.opModeIsActive() && !onHeading(speed, angle)) {
                // Update telemetry & Allow time for other processes to run.
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.update();
            }
            tcTrack.updateTicks(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            scTracker.setDirectionDeg(angle);
            RobotLog.ii("DriveStraight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate, scTracker.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad - scTracker.direction));
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }


    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param //holdTime Length of time (in seconds) to hold the specified heading.
     */
    // we need to talk about changing this back to the
    // original sample form.
    boolean onHeading(double speed, double angle) {

        boolean onTarget = false;
        // determine turn power based on +/- error

        if (angle == gyro.getIntegratedZValue()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            scTracker.setPositionAndDirection(tcTrack);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            opMode.sleep(500);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            onTarget = true;
        } else {
            if (angle == (gyro.getIntegratedZValue()+1)) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                scTracker.setPositionAndDirection(tcTrack);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // reset tracking to last known coordinate and direction just in case vuforia provided new info
                tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                opMode.sleep(500);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                onTarget = true;
            }
            if (angle == (gyro.getIntegratedZValue()-1)) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                scTracker.setPositionAndDirection(tcTrack);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // reset tracking to last known coordinate and direction just in case vuforia provided new info
                tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                opMode.sleep(500);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                onTarget = true;
            }
            if (gyro.getIntegratedZValue() - angle >= 2) {
                rightMotor.setPower(-speed);
                leftMotor.setPower(speed);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

            } else {
                if (gyro.getIntegratedZValue() - angle <= -2) {

                    rightMotor.setPower(speed);
                    leftMotor.setPower(-speed);
                    tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                }
            }

        }
        tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
        scTracker.setPositionAndDirection(tcTrack);
        opMode.sleep(Pause_Time);
        return onTarget;

    }

    public void OdsDrive(double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(TurnCalc.WallDistL(odsReadingLinear));
            rightMotor.setPower(TurnCalc.WallDistR(odsReadingLinear));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                odsReadingRaw = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
                odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.addData("0 ODS Raw", odsReadingRaw);
                opMode.telemetry.addData("1 ODS linear", odsReadingLinear);

                //opMode.telemetry.addData("Clear", sensorRGB.alpha());
                // opMode.telemetry.addData("Red  ", sensorRGB.red());
                // opMode. telemetry.addData("Green", sensorRGB.green());
                //opMode. telemetry.addData("Blue ", sensorRGB.blue());
                opMode.telemetry.update();
                leftMotor.setPower(TurnCalc.WallDistL(odsReadingLinear));
                rightMotor.setPower(TurnCalc.WallDistR(odsReadingLinear));
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // Allow time for other processes to run.
                opMode.idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            scTracker.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);

        }
    }
    public void OdsDriveRev(double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTracker, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (dist * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(TurnCalc.WallDistR(odsReadingLinear));
            rightMotor.setPower(TurnCalc.WallDistL(odsReadingLinear));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                odsReadingRaw = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
                odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.addData("0 ODS Raw", odsReadingRaw);
                opMode.telemetry.addData("1 ODS linear", odsReadingLinear);

                //opMode.telemetry.addData("Clear", sensorRGB.alpha());
                // opMode.telemetry.addData("Red  ", sensorRGB.red());
                // opMode. telemetry.addData("Green", sensorRGB.green());
                //opMode. telemetry.addData("Blue ", sensorRGB.blue());
                opMode.telemetry.update();
                leftMotor.setPower(TurnCalc.WallDistL(-odsReadingLinear));
                rightMotor.setPower(TurnCalc.WallDistR(-odsReadingLinear));
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // Allow time for other processes to run.
                opMode.idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            scTracker.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);
        }
    }

    public void Shoot(double up, double down) {
        leftshooter.setPower(-1);
        rightshooter.setPower(1);
        opMode.sleep(1000);
        servo.setPosition(up);
        opMode.telemetry.addData(">", "Servo is in Up Position");
        opMode.telemetry.update();
        opMode.sleep(1000);
        servo.setPosition(down);
        opMode.telemetry.addData(">", "Servo is in Down Position");
        opMode. telemetry.update();
        Intake.setPower(-1);
        opMode. sleep(5000);
        Intake.setPower(0);
        opMode. sleep(2000);
        servo.setPosition(up);
        opMode. telemetry.addData(">", "Servo is in Up Position");
        opMode.telemetry.update();
        opMode. sleep(1000);
        servo.setPosition(down);
        leftshooter.setPower(0);
        rightshooter.setPower(0);
        opMode. telemetry.addData(">", "Servo is in Down Position");
        opMode.telemetry.update();
        opMode.idle();
        opMode.sleep(Pause_Time);

    }
    public void BeaconEvalBlue(boolean Blue,boolean Red,double IN, double Out){
        if (Blue) {
            opMode.sleep(1000);
            OdsDrive(5.3, 100);
            opMode.telemetry.addData("Color", "BLUE");
            opMode.telemetry.addData("Red  ", sensorRGB.red());
            opMode.telemetry.addData("Blue  ", sensorRGB.blue());
            opMode.telemetry.update();
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.stop();

//
                      } else if (Red) {
            opMode.sleep(1000);
            OdsDrive(5.3, 100);
            opMode.telemetry.addData("Color", "RED");
            opMode.telemetry.addData("Red  ", sensorRGB.red());
            opMode.telemetry.addData("Blue  ", sensorRGB.blue());
            opMode.telemetry.update();
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.sleep(125);
            button.setPosition(Out);
            opMode.sleep(500);
            button.setPosition(IN);
            opMode.stop();

                 } else {
            odsReadingRaw2 = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear2 = Math.pow(odsReadingRaw2, -0.5);
            rightMotor.setPower(TurnCalc.WallDistR(odsReadingLinear2));
            leftMotor.setPower(TurnCalc.WallDistL(odsReadingLinear2));
            opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            opMode.telemetry.addData("Clear", sensorRGB.alpha());
            opMode.telemetry.addData("Red  ", sensorRGB.red());
            opMode.telemetry.addData("Red  ", sensorRGB.red());
            opMode.telemetry.addData("1 ODS linear", odsReadingLinear2);
            opMode.telemetry.update();




             }



    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }


    /**
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */


    static final String ZPMToString( DcMotor.ZeroPowerBehavior zpm ){
        String value = "UNKNOWN";
        switch( zpm ){
            case BRAKE:{
                value = "BREAK";
                break;
            }
            case FLOAT:{
                value = "FLOAT";
                break;
            }
            case UNKNOWN: // UNKNOWN fall through to default
            default:
            {
                break;
            }
        }
        return value;
    }
}