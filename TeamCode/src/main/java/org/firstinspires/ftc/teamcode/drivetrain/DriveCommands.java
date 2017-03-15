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

import org.firstinspires.ftc.teamcode.fieldtracking.DirectionDistance;
import org.firstinspires.ftc.teamcode.fieldtracking.SimpleCoordinateTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TickCountTracker;
import org.firstinspires.ftc.teamcode.fieldtracking.TurnCalc;
import org.firstinspires.ftc.teamcode.fieldtracking.Util;
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
    public static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: TETRIX Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = .625;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double Pi = 3.141592653f;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Pi);
    public static final double HEADING_THRESHOLD = 10;      // As tight as we can make it with an integer gyro
    public static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    public static final double P_DRIVE_COEFF = .075;     // Larger is more responsive, but also less stable
    public static final long Pause_Time = 50;
    double odsReadingRaw;
    double odsReadingRaw2;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;
    static double odsReadingLinear2;

    public LinearOpMode opMode   = null;
    public TickCountTracker tcTrack = null;
    public SimpleCoordinateTracker scTrack = null;

    /**
     * Initialize EncoderDrive for Op Mode.
     */
    public void initializeForOpMode(LinearOpMode opModeIn, HardwareMap hwMap, final Vector2d coordinate, double ccwAngleDeg) throws InterruptedException {

        /**this must be first*/
        opMode = opModeIn;

        /**Initialize SimpleCoordinateTracker with Blue Center position*/
        opMode.telemetry.addData("Status", "Initializing Field Coordinates");
        opMode.telemetry.update();
        scTrack = new SimpleCoordinateTracker();
        scTrack.setPositionAndDirectionDeg(coordinate, ccwAngleDeg);

        /**Send telemetry message to indicating getting HW form HW Map*/
        opMode.telemetry.addData("Status", "Checking HW Map");
        opMode.telemetry.update();

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
        odsReadingRaw   = ODS.getRawLightDetected();  //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
        odsReadingLinear= Math.pow(odsReadingRaw, -0.5);

        /**Send telemetry message initializing drive system*/
        opMode.telemetry.addData("Status", "Init Drive System");
        opMode.telemetry.update();

        // set initial direction
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run with encoders.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /** Set zero power behavior to brake mode */
        RobotLog.ii(TAG, "ZP Behavior before - Left:=%s Right:=%s"
                , ZPMToString(leftMotor.getZeroPowerBehavior())
                , ZPMToString((rightMotor.getZeroPowerBehavior())));
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotLog.ii(TAG, "ZP Behavior after - Left:=%s Right:=%s"
                , ZPMToString(leftMotor.getZeroPowerBehavior())
                , ZPMToString((rightMotor.getZeroPowerBehavior())));

       /** Send telemetry message to as we reset encoders*/
        opMode.telemetry.addData("Status", "Resetting Encoders");
        opMode.telemetry.update();
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

        /**Initialize TickCountTracker with SimpleCoordinateTracker and current motor ticks*/
        opMode.telemetry.addData("Status", "Init TickCountTracker");
        opMode.telemetry.update();
        tcTrack = new TickCountTracker();
        tcTrack.initialize(scTrack, encoderLeft, encoderRight );
    }

    protected void DriveCore(double speed,
                      double leftInches, double rightInches,
                      double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            /**update tick tracker with current positions*/
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
            scTrack.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    /**
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void EncoderDriveCore( double speed,
                                  double leftInches, double rightInches,
                                  double timeoutS) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive() && 0.0 != leftInches || 0.0 != rightInches ) {

            // Determine initial and target positions
            int initialLeftPos = leftMotor.getCurrentPosition();
            int initialRightPos = rightMotor.getCurrentPosition();
            int targetLeftPos = initialLeftPos + (int)(leftInches * COUNTS_PER_INCH);
            int targetRightPos = initialRightPos + (int)(rightInches * COUNTS_PER_INCH);

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // calculate the needed speed differential between the two sides
            // such that each side should arrive at the desired location at the same time
            // by biasing the speed to one side or the other.
            int initialLeftDelta = Math.abs( initialLeftPos-targetLeftPos );
            int initialRightDelta = Math.abs( initialRightPos-targetRightPos );
            int maxDelta = Math.max( initialLeftDelta, initialRightDelta );

            boolean bBreakLeftMode = (initialLeftDelta == 0);
            boolean bBreakRightMode = (initialRightDelta == 0);

            // one or both speeds will be <= speed without exceeding speed.
            final double leftSpeed = speed * ((double)initialLeftDelta/(double)maxDelta);
            final double rightSpeed = speed * ((double)initialRightDelta/(double)maxDelta);
            double powerRamp = 0.1; // initial speed 10% of target
            // calculate new speed based on power ramp
            double newLeftSpeed = powerRamp * leftSpeed ;
            double newRightSpeed = powerRamp * rightSpeed ;

            // Display it for the drive
            opMode.telemetry.addData("Speed", "Start IS=%.04f, PR=%.04f", speed, powerRamp);
            opMode.telemetry.addData("LS", "TS=%.04f CS=%.04f", leftSpeed, leftSpeed);
            opMode.telemetry.addData("RS", "TS=%.04f CS=%.04f", rightSpeed, rightSpeed);
            opMode.telemetry.addData("Tics", "LTD=%7d RTD=%7d", initialLeftDelta, initialRightDelta);
            opMode.telemetry.addData("Tics", "LCD=%7d RCD=%7d", initialLeftDelta, initialRightDelta);
            opMode.telemetry.update();

            // reset the timeout time and start motion.
            runtime.reset();

            // Set initial motor speed based on 0.1 min power ramp value (10%)
            if ( !bBreakLeftMode ) {
                // pass target positions on to the motor controllers
                leftMotor.setTargetPosition(targetLeftPos);
                // Turn On RUN_TO_POSITION
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set initial speed
                leftMotor.setPower(Math.abs(newLeftSpeed));
            }
            if ( !bBreakRightMode ) {
                // pass target positions on to the motor controllers
                rightMotor.setTargetPosition(targetRightPos);
                // Turn On RUN_TO_POSITION
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set initial speed
                rightMotor.setPower(Math.abs(newRightSpeed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            while ( opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() || rightMotor.isBusy()) ) {

                // calculate the current delta from target position
                int curLeftDelta = Math.abs( leftMotor.getCurrentPosition()-targetLeftPos );
                int curRightDelta = Math.abs( rightMotor.getCurrentPosition()-targetRightPos );

                // calculate progress ratio in the range of 1.0 to 0.0 indicating how much of the distance is left to traverse
                // check denominator to and assign value of 1.0 to avoid division by zero.
                double leftProgressRatio = Range.clip((initialLeftDelta != 0 ?  ((double)curLeftDelta/(double)initialLeftDelta) : 0.0 ),0.0,1.0);
                double rightProgressRatio = Range.clip((initialRightDelta != 0 ?  ((double)curRightDelta/(double)initialRightDelta) : 0.0 ),0.0,1.0);

                // if one side or the other is at it's target location it's power will go to zero
                // the side that is lagging behind the other will represent the max ratio.
                double maxProgressRatio = Math.max( leftProgressRatio, rightProgressRatio );
                if ( maxProgressRatio != 0.0 ) {

                    // powerRamp is a simple multiplier for the final speed calc to reduce speed
                    // by linear ramp percentage from 10%-100% and then 100% back down to 10%
                    // of the requested speed.
                    opMode.telemetry.addData("MPR", "CV=%.04f", maxProgressRatio);
                    if ( maxProgressRatio < 0.25 ){ /**ramp power up from 10% to 100%*/
                        powerRamp = 0.1 + 0.9 * (maxProgressRatio/0.25);
                    } else {
                        if ( maxProgressRatio < 0.75 ){ /**progress between 25% and 75% - hold powerRamp at 100%*/
                            powerRamp = 1.0;
                        } else { /**ramp power down from 100% to 10%*/
                            powerRamp = 1.0 - 0.9 * ((maxProgressRatio-0.75)/0.25);
                        }
                    }
                    // calculate current speed based on power ramp
                    newLeftSpeed = powerRamp *leftSpeed ;
                    newRightSpeed = powerRamp * rightSpeed ;

                    // update motor speed based on latest position.
                    if ( !bBreakLeftMode ) {
                        leftMotor.setPower(Range.clip(Math.abs(newLeftSpeed), 0.0, 1.0));
                    }
                    if ( !bBreakRightMode ) {
                        rightMotor.setPower(Range.clip(Math.abs(newRightSpeed), 0.0, 1.0));
                    }

                }
                else
                {
                    opMode.telemetry.addData("MPR", "CV=%.04f", maxProgressRatio);
                }

                // Display it for the drive
                opMode.telemetry.addData("Speed", "IS=%.04f, PR=%.04f", speed, powerRamp);
                opMode.telemetry.addData("LS", "TS=%.04f CS=%.04f", leftSpeed, newLeftSpeed);
                opMode.telemetry.addData("RS", "TS=%.04f CS=%.04f", rightSpeed, newRightSpeed);
                opMode.telemetry.addData("Tics", "LTD=%7d RTD=%7d", initialLeftDelta, initialRightDelta);
                opMode.telemetry.addData("Tics", "LCD=%7d RCD=%7d", curLeftDelta, curRightDelta);
                opMode.telemetry.update();

                /**update tick tracker with current positions*/
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // we may want to replace this idel() with sleep(50 or 100);
                opMode.idle(); // give the system a moment as we wait for things to progress
                opMode.sleep(25);
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            opMode.sleep(100);   // optional pause after each move
            opMode.idle();
        }
        else {
            opMode.telemetry.addData("EDrive","No ActionTaken");
        }
    }

    public void DriveStraight(double speed,
                             double distInches,
                             double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            EncoderDriveCore(speed, distInches, distInches, timeoutS);
            scTrack.moveOnCurentHeading(distInches);

            double dd = Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).asDirectionDistance().distIn;
            double ad = Math.toDegrees(Util.OptomizeAngleNegPi_PosPi(tcTrack.dirRad-scTrack.direction));
            String diffText = String.format( "DD: %.04f\", AD: %.04fd",dd, ad );
            opMode.telemetry.addData("CMD", "DriveStraight %.04f\"",distInches);
            opMode.telemetry.addData("Diff", diffText );
            RobotLog.ii("DriveStraight", diffText );

            scTrack.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void EncoderDrive(double speed,
                      final DirectionDistance dd,
                      double timeoutS) throws InterruptedException {
        /**
         * calculation to get the relative turn needed to make the
         * robot align with the center of the blue vortex
         * */
        double deltaRad = dd.dirRad - scTrack.direction;
        double turnAngle = Math.toDegrees( Util.OptomizeAngleNegPi_PosPi( deltaRad ));
        //Turn to direction
        if ( !Util.FuzzyZero(turnAngle, 0.5) ){
            TurnRelative( speed, Math.toDegrees(turnAngle), timeoutS );
        }
        //drive on heading
        if (!Util.FuzzyZero(dd.distIn, 0.1) ){
            DriveStraight( speed, dd.distIn, timeoutS );
        }
    }


    public void EncoderTurnToDirectionDegrees(double speed,
                          double dirDeg,
                          double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            /**
             * calculation to get the relative turn needed to make the
             * robot align with the center of the blue vortex
             * */
            double deltaRad = Util.OptomizeAngleNegPi_PosPi(Math.toRadians(dirDeg) - scTrack.direction);
            double turnAngle = Math.toDegrees( deltaRad );
            /**Turn to face the vortex*/
            if ( !Util.FuzzyZero(turnAngle, 0.5) ){

                double dist = TurnCalc.Turn(turnAngle);
                EncoderDriveCore(speed, dist, -dist, timeoutS);
                scTrack.turnRelativeRad(turnAngle);

                double dd = Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).asDirectionDistance().distIn;
                double ad = Math.toDegrees(Util.OptomizeAngleNegPi_PosPi(tcTrack.dirRad-scTrack.direction));
                String diffText = String.format( "DD: %.04f\", AD: %.04fd",dd, ad );
                opMode.telemetry.addData("CMD", "EncoderTurnToDirectionDegrees %.04f\"",dirDeg);
                opMode.telemetry.addData("Diff", diffText );
                RobotLog.ii("EncoderTurnToDirectionDegrees", diffText );

                scTrack.setPositionAndDirection(tcTrack);

                opMode.sleep(Pause_Time);   // optional pause after each move
            }
        }
    }

    public void TurnRelative(double speed,
                            double turndeg,
                            double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            double dist = TurnCalc.Turn(turndeg);
            EncoderDriveCore(speed, dist, -dist, timeoutS);

            scTrack.turnRelativeDeg(turndeg);

            double dd = Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).asDirectionDistance().distIn;
            double ad = Math.toDegrees(Util.OptomizeAngleNegPi_PosPi(tcTrack.dirRad-scTrack.direction));
            String diffText = String.format( "DD: %.04f\", AD: %.04fd",dd, ad );
            opMode.telemetry.addData("CMD", "TurnRelative %.04fd",turndeg);
            opMode.telemetry.addData("Diff", diffText );
            RobotLog.ii("TurnRelative", diffText );

            scTrack.setPositionAndDirection(tcTrack);
            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }


    public void TurnRight(double speed,
                          float turndeg,
                          double timeoutS) throws InterruptedException {
        TurnRelative( speed, -(double)turndeg, timeoutS );
     }

    public void BreakTurnRight(double speed,
                               double turndeg,
                              double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            DcMotor.ZeroPowerBehavior zpmOld = leftMotor.getZeroPowerBehavior();
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double dist = TurnCalc.BreakTurnDeg(turndeg);
            EncoderDriveCore(speed, dist, 0.0, timeoutS);
            scTrack.moveArcTurnRightDeg(TurnCalc.WheelSpace, turndeg);

            leftMotor.setZeroPowerBehavior(zpmOld);

            double dd = Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).asDirectionDistance().distIn;
            double ad = Math.toDegrees(Util.OptomizeAngleNegPi_PosPi(tcTrack.dirRad-scTrack.direction));
            String diffText = String.format( "DD: %.04f\", AD: %.04fd",dd, ad );
            opMode.telemetry.addData("CMD", "BreakTurnRight %.04fd",turndeg);
            opMode.telemetry.addData("Diff", diffText );
            RobotLog.ii("BreakTurnRight", diffText );

            scTrack.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);   // optional pause after each move
        }
    }

    public void TurnLeft(double speed,
                          float turndeg,
                          double timeoutS) throws InterruptedException {
        TurnRelative( speed, (double)turndeg, timeoutS );
    }

    public void BreakTurnLeft(double speed,
                         double turndeg,
                         double timeoutS) throws InterruptedException {
        if (opMode.opModeIsActive()) {

            DcMotor.ZeroPowerBehavior zpmOld = leftMotor.getZeroPowerBehavior();
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            double dist = TurnCalc.BreakTurnDeg(turndeg);
            EncoderDriveCore(speed, 0.0, dist, timeoutS);
            scTrack.moveArcTurnLeftDeg(TurnCalc.WheelSpace, turndeg);

            leftMotor.setZeroPowerBehavior(zpmOld);

            double dd = Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).asDirectionDistance().distIn;
            double ad = Math.toDegrees(Util.OptomizeAngleNegPi_PosPi(tcTrack.dirRad-scTrack.direction));
            String diffText = String.format( "DD: %.04f\", AD: %.04fd",dd, ad );
            opMode.telemetry.addData("CMD", "BreakTurnLeft %.04fd",turndeg);
            opMode.telemetry.addData("Diff", diffText );
            RobotLog.ii("BreakTurnLeft", diffText );

            scTrack.setPositionAndDirection(tcTrack);

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
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            scTrack.turnRelativeDeg(turndeg);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

            RobotLog.ii("OldTurnRight", "Error: %s, DA: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTrack.direction));

            scTrack.setPositionAndDirection(tcTrack);
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
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            scTrack.moveOnCurentHeading(turndeg);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            /**
            RobotLog.ii("OldTurnLeft", "Error: %s, DA: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTrack.direction));
             */
            scTrack.setPositionAndDirection(tcTrack);
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
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            scTrack.moveOnCurentHeading(distance);
            /**
            RobotLog.ii("DriveStraight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate,scTrack.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad-scTrack.direction));
             */
            scTrack.setPositionAndDirection(tcTrack);
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
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // keep looping while we have time remaining.
            // keep looping while we are still active, and not on heading.
            while (opMode.opModeIsActive() && !onHeading(speed, angle)) {
                // Update telemetry & Allow time for other processes to run.
                opMode.telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
                opMode.telemetry.update();
            }
            tcTrack.updateTicks(leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            scTrack.setDirectionDeg(angle);
            RobotLog.ii("DriveStraight", "Error: %s, AE: %.04fd",
                    Vector2d.Subtract(tcTrack.coordinate, scTrack.coordinate).formatAsString(),
                    Math.toDegrees(tcTrack.dirRad - scTrack.direction));
            scTrack.setPositionAndDirection(tcTrack);
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
            scTrack.setPositionAndDirection(tcTrack);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
            opMode.sleep(500);
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            onTarget = true;
        } else {
            if (angle == (gyro.getIntegratedZValue()+1)) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                scTrack.setPositionAndDirection(tcTrack);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // reset tracking to last known coordinate and direction just in case vuforia provided new info
                tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                opMode.sleep(500);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                onTarget = true;
            }
            if (angle == (gyro.getIntegratedZValue()-1)) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
                scTrack.setPositionAndDirection(tcTrack);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // reset tracking to last known coordinate and direction just in case vuforia provided new info
                tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
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
        scTrack.setPositionAndDirection(tcTrack);
        opMode.sleep(Pause_Time);
        return onTarget;

    }

    public boolean OdsDriveStraightToWhiteLine( double speed, double maxDist, double timeoutS ){

        boolean bFoundWhiteLine = false;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine initial and target positions
            int initialLeftPos = leftMotor.getCurrentPosition();
            int initialRightPos = rightMotor.getCurrentPosition();
            int targetLeftPos = initialLeftPos + (int)(maxDist * COUNTS_PER_INCH);
            int targetRightPos = initialRightPos + (int)(maxDist * COUNTS_PER_INCH);

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

            // pass target positions on to the motor controllers
            leftMotor.setTargetPosition(targetLeftPos);
            rightMotor.setTargetPosition(targetRightPos);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            odsReadingRaw = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
            odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
            opMode.telemetry.addData("ODS", "Light: %.04f", odsReadingRaw);
            opMode.telemetry.update();

            // break mode will make sure we stop when we hit the line
            DcMotor.ZeroPowerBehavior zpmOldLeft = leftMotor.getZeroPowerBehavior();
            DcMotor.ZeroPowerBehavior zpmOldRight = rightMotor.getZeroPowerBehavior();
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while ( opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() || rightMotor.isBusy()) ) {

                /**update tick tracker with current positions*/
                tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());

                // Display light levels
                odsReadingRaw = ODS.getRawLightDetected();              //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
                odsReadingLinear = Math.pow(odsReadingRaw, -0.5);
                opMode.telemetry.addData("ODS", "Light: %.04f", odsReadingRaw);
                opMode.telemetry.update();

                if ( odsReadingRaw >= .8 ) // this is just a guess could be anywhere above 0.4?
                {
                    bFoundWhiteLine = true;
                    break;
                }
                // we may want to replace this idel() with sleep(50 or 100);
                opMode.idle(); // give the system a moment as we wait for things to progress
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            /**update tick tracker with current positions*/
            tcTrack.updateTicks(leftMotor.getCurrentPosition(),rightMotor.getCurrentPosition());
            scTrack.setPositionAndDirection(tcTrack);

            opMode.telemetry.addData("ODS", "Light: %.04f", odsReadingRaw);
            if ( bFoundWhiteLine ) {
                opMode.telemetry.addData("ODS", "Found White Line");
            }
            opMode.telemetry.update();

            opMode.sleep(Pause_Time);   // optional pause after each move
            opMode.idle();

            // safe to restore now...
            leftMotor.setZeroPowerBehavior(zpmOldLeft);
            rightMotor.setZeroPowerBehavior(zpmOldRight);
        }
        return bFoundWhiteLine;
    }

    // follow edge
    public void OdsDrive(double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            scTrack.setPositionAndDirection(tcTrack);

            opMode.sleep(Pause_Time);

        }
    }
    public void OdsDriveRev(double dist, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // reset tracking to last known coordinate and direction just in case vuforia provided new info
            tcTrack.initialize(scTrack, leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());

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
            scTrack.setPositionAndDirection(tcTrack);
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
            ///OdsDrive(5.3, 100);
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
            //opMode.stop();

//
                      } else if (Red) {
            opMode.sleep(1000);
            try {
                DriveStraight(0.5, 5.3, 5.0);
            } catch(InterruptedException e){

            }
            // (5.3, 100);
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
            ///opMode.stop();

                 } else {
            //odsReadingRaw2 = ODS.getRawLightDetected();                   //update raw value (This function now returns a value between 0 and 5 instead of 0 and 1 as seen in the video)
           // odsReadingLinear2 = Math.pow(odsReadingRaw2, -0.5);
           // rightMotor.setPower(TurnCalc.WallDistR(odsReadingLinear2));
            //leftMotor.setPower(TurnCalc.WallDistL(odsReadingLinear2));
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
