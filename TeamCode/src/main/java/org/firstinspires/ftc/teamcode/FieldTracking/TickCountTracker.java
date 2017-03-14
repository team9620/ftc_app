package org.firstinspires.ftc.teamcode.fieldtracking;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Created by ROUS on 3/2/2017.
 */
public class TickCountTracker {
    public static final double COUNTS_PER_MOTOR_REV = 1680;    // AndyMark NeveRest 60 Never Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = .625;     // Motor is geared 60: 1 but that is accounted for in ticks above - This is <1.0 if geared UP or > 1.0 if reduced beyond ticks above
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double PI = 3.141592653f;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    public static final double ROBOT_WHEEL_WIDTH = 14.5;        //Width in inches from left drive wheel center to right drive wheel center

    static final double   ADJACENT_SQUARED     = (ROBOT_WHEEL_WIDTH*ROBOT_WHEEL_WIDTH);  //Wheel width squared is used for adjacent calc of deflection

    protected int leftTicks;
    protected int rightTicks;
    public Vector2d coordinate;
    public double dirRad;

    public TickCountTracker() {
        this.leftTicks = 0;
        this.rightTicks = 0;
        this.dirRad = 0.0;
        this.coordinate = new Vector2d();
    }

    /**
     * Format position and direction as a string
     */
    public String formatAsString()
    {
        return String.format("%s-> %.08fd", this.coordinate.formatAsString(), Math.toDegrees(this.dirRad));
    }

    public void  initialize(final SimpleCoordinateTracker sctrack, int left, int right){
        this.leftTicks = left;
        this.rightTicks = right;
        this.dirRad = sctrack.direction;
        this.coordinate.set(sctrack.coordinate);
    }
    public void initializeRad(double xin, double yin, double ccwDirRad, int left, int right) {
        this.leftTicks = left;
        this.rightTicks = right;
        this.dirRad = ccwDirRad;
        this.coordinate.set(xin,yin);
    }

    public void initializeDeg(double xin, double yin, double ccwDirDeg, int left, int right) {
        initializeRad(xin, yin, Math.toRadians(ccwDirDeg), left, right);
    }

    public void updateTicks(int left, int right) {

        int dtLeft = left - this.leftTicks;
        int dtRight = right - this.rightTicks;

        RobotLog.ii("TickCountTracker", "updateTicks CT[%d,%d] DT[%d,%d]",left, right, dtLeft, dtRight);

        //calculate distance traveled by each wheel
        double distLeft = ((double)dtLeft)/ COUNTS_PER_INCH;
        double distRight = ((double)dtRight) / COUNTS_PER_INCH;

        SimpleCoordinateTracker calc = new SimpleCoordinateTracker().setPositionAndDirection(this);

        if ( dtLeft == dtRight ){ // linear travel

            //calculate the average distance moved(centroid or robot).
            double moveAvg = ((distLeft + distRight)/2.0);
            calc.moveOnCurentHeading(moveAvg);
            Vector2d deltaXY = DirectionDistance.CreateVector2dRad(calc.direction, moveAvg);
            RobotLog.ii("TickCountTracker", "\tLinearTravel %s %s", deltaXY.formatAsString(), deltaXY.asDirectionDistance().formatAsString());

        } else { // arc travel

            boolean bTurnSide = (Math.abs(distRight) > Math.abs(distLeft));
            double L1 = (bTurnSide ? distLeft : distRight);
            double L2 = (bTurnSide ? distRight : distLeft);
            double D = ROBOT_WHEEL_WIDTH;

            double R1 = (L1*D)/(L2-L1);
            double R2 = (L2*D)/(L2-L1);
            double RC = (R1+R2)/2.0; // locate center of robot rotation radius
            double thetaRad = L2/Math.abs(R2);


            RobotLog.ii("TickCountTracker", "\tTheta: %.08fd R1: %.08f, R2: %.08f, RC: %.08f", Math.toDegrees(thetaRad), R1, R2, RC );

            if ( bTurnSide ) { // true is turn left

                calc.moveArcTurnLeftRad( Math.abs(RC), thetaRad );

                // calculate new coordinate and new robot direction
                Vector2d deltaXY = Vector2d.Subtract(calc.coordinate, this.coordinate);
                double deltaAng = Util.OptomizeAngleNegPi_PosPi(calc.direction-this.dirRad);

                RobotLog.ii("TickCountTracker", "\tArcLeft Delta:%.08fd %s", Math.toDegrees(deltaAng), deltaXY.formatAsString());

            } else { // false is turn right

                calc.moveArcTurnRightRad( Math.abs(RC), thetaRad );

                // calculate new coordinate and new robot direction
                Vector2d deltaXY = Vector2d.Subtract(calc.coordinate, this.coordinate);
                double deltaAng = Util.OptomizeAngleNegPi_PosPi(calc.direction-this.dirRad);

                // update internal data
                RobotLog.ii("TickCountTracker", "\tArcRight Delta:%.08fd %s", Math.toDegrees(deltaAng), deltaXY.formatAsString());
            }
        }

        // update internal data
        this.coordinate.set( calc.coordinate );
        this.dirRad = calc.direction;

        RobotLog.ii("TickCountTracker", "\tNewPos %s", this.formatAsString());

        // update previous ticks count
        this.leftTicks = left;
        this.rightTicks = right;
    }

    public void Old_updateTicks(int left, int right) {

        //calculate distance traveled by each wheel
        double dLeft = ((double) (left - this.leftTicks)) / COUNTS_PER_INCH;
        double dRight = ((double) (right - this.rightTicks)) / COUNTS_PER_INCH;

        //calculate the abs diff in distance moved between the wheel (opposite)
        double opposite = Math.abs(dLeft - dRight);
        //calculate the average distance moved(centroid or robot).
        double moveAvg = ((dLeft + dRight)/2.0);
        //the wheel base is the adjacent of a right triangle
        //calculate the hypotenuse of the triangle a^2 + b^2 = c^2
        double hypotenuse = Math.sqrt(ADJACENT_SQUARED + opposite*opposite);

        //basic calc of deflection angel is theta = inv cosine (adjacent/hypotenuse);
        double deflection = Math.acos(ROBOT_WHEEL_WIDTH / hypotenuse);

        /**
         * sign the angle based on weather abs(left) > abs(right)
         * IE: did we 1) drift right (left > right)
         *            2) straight (left = right) (theta = 0)
         *            3) drift left ( right > left)
         */
        if (dLeft > dRight) {
            deflection *= -1.0;
        }

        /**
         * finally calculate dx dy from the updated average traveled distance and added deflection
         */
        this.dirRad += deflection;
        double dx = (Math.cos(this.dirRad) * moveAvg);
        double dy = (Math.sin(this.dirRad) * moveAvg);

        this.coordinate.add(dx, dy);

        //record into to the robot log
        /**
        RobotLog.ii("TickCountTracker", "Ticks[%d,&d] delta[%.04f,%.04f] DxDy[%.04f,%.04f] A,D[%.04fd,%.04f\" Dir[%.04f] Pos[%s]",
                left, right, dLeft, dRight, dx, dy, Math.toDegrees(deflection), moveAvg,
                Math.toDegrees(this.dirRad), coordinate.formatAsString());
         */

        this.leftTicks = left;
        this.rightTicks = right;
    }
}
