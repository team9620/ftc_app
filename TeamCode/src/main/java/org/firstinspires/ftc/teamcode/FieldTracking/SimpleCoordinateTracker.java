package org.firstinspires.ftc.teamcode.fieldtracking;

/**
 * Created by ROUS on 2/27/2017.
 */

/**
 * *Keeps track of current coordinate based on navigation input
 */
public class SimpleCoordinateTracker {
    /**
     * *public data members for
     */
    public double direction; /**robot orientation as ccw direction relative to x-axis*/
    public Vector2d coordinate;/**robot center cordinate on the field*/

    /**
     * *constructors create initialize the data members
     */

    /**default constructor intializes the object with x=0.0, y=0.0*/
    public SimpleCoordinateTracker() {
        this.direction = 0.0;
        this.coordinate = new Vector2d(0.0,0.0);
    }

    /**Format robot coordinate position and direction as a string*/
    public String formatAsString(){
        return String.format("Pos: %s Dir:%.04f)", this.direction, this.coordinate.formatAsString());
    }

    public SimpleCoordinateTracker setPosition( double x, double y ){this.coordinate = new Vector2d(x, y);
    return this;
    }

    public SimpleCoordinateTracker setPosition(final Vector2d vec){
        return setPosition(vec.x,vec.y);
    }

    public SimpleCoordinateTracker setDirectionRad(double ccwAngle){
        this.direction = ccwAngle;
        return this;
    }

    public SimpleCoordinateTracker setDirectionDeg(double ccwAngle){
        this.direction = Math.toRadians(ccwAngle);
        return this;
    }

    public SimpleCoordinateTracker setPositionAndDirectionDeg(double x, double y, double ccwAngle){
        return setDirectionDeg(ccwAngle).setPosition(x,y);
    }

    public SimpleCoordinateTracker setPositionAndDirectionDeg(final Vector2d vec, double ccwAngle){
        return setDirectionDeg(ccwAngle).setPosition(vec);
    }

    public SimpleCoordinateTracker setPositionAndDirectionRad(final Vector2d vec, double ccwAngle){
        return setDirectionRad(ccwAngle).setPosition(vec);
    }

    public SimpleCoordinateTracker setPositionAndDirection(final TickCountTracker tcTrack){
        return setDirectionRad(tcTrack.dirRad).setPosition(tcTrack.coordinate);
    }

    public SimpleCoordinateTracker setPositionAndDirection(final VuforiaTarget obs){
        return setDirectionRad(obs.getRobotDirRad()).setPosition(obs.getRobotPos());
    }

    public SimpleCoordinateTracker shiftPositionByXY(double x, double y){
        return shiftPositionByXY(new Vector2d(x, y));
    }

    public SimpleCoordinateTracker shiftPostionByDirectionDistance(final DirectionDistance dd){
        return shiftPositionByXY(dd.asVector2d());
    }

    public SimpleCoordinateTracker shiftPositionByXY(final Vector2d vec){
        this.coordinate.add(vec);
        return this;
    }

    public SimpleCoordinateTracker turnRelativeDeg(double ccwAngle){
        return turnRelativeRad( Math.toRadians(ccwAngle) );
    }

    public SimpleCoordinateTracker turnRelativeRad(double ccwAngle){
        this.direction += ccwAngle;
        return this;
    }

    public SimpleCoordinateTracker moveOnCurentHeading(double distance){
        DirectionDistance dd = new DirectionDistance(this.direction, distance);
        this.coordinate.add(dd.asVector2d());
        return this;
    }

    // helper function to calculate a point on a circle given the center, radius and
    // an x-axis relative ccw direction
    static public Vector2d CalculatePointOnCircle(final Vector2d arcCenter, double arcRadius, double xAxisRelativeCCWDirection ){
        Vector2d arcPoint = new Vector2d(arcCenter);
        arcPoint.x += ( Math.cos(xAxisRelativeCCWDirection) * Math.abs(arcRadius));
        arcPoint.y += ( Math.sin(xAxisRelativeCCWDirection) * Math.abs(arcRadius));
        return arcPoint;
    }

    static public Vector2d CalculateArcCenterLeft( final Vector2d startPoint, double arcRadius, double xAxisRelativeCCWStartDirection ){
        /**
         * to calculate the radial center of the arc, we need the starting point of the arc and
         * the tangent direction of the arc at that starting point.  From that we can calculate
         * the perpendicular direct, multiply by the arc radius adn add that value to the current
         * coordinate to get the center point.
         *
         * In 2D the perpendicular direction is the revers of direction components where x becomes y
         * and y becomes x using a unit vector so to simplify our calc we just swap sin for cos
         * in our direction calc before multiplying the distance
         *
         * Because we are offsetting left we need to multiply by a negative arc radius to
         * */
        Vector2d arcCenter = new Vector2d(startPoint);
        arcCenter.x += ( Math.sin(xAxisRelativeCCWStartDirection) * -Math.abs(arcRadius));
        arcCenter.y += ( Math.cos(xAxisRelativeCCWStartDirection) * -Math.abs(arcRadius));
        return arcCenter;
    }

    /**
     * For left arc turn, the radial center point is to the lef of our current coordinate,
     * and the radial direction is + 90 Degrees from our Current robot direction
     * We can then add the deflection angle +- will work to get both our
     * final robot direction and the new radial direction.
     *
     * */
    public void moveArcTurnLeft( double arcRadius, double deflectionAngleLeft ) {

        // remember previous values for logging
        double oldDir = this.direction;
        Vector2d oldPos = new Vector2d(this.coordinate);

        // convert deflection angle to radians
        double   radDeflectionAngle = Math.toRadians(deflectionAngleLeft);

        // calculate the arc radial center point on the field
        Vector2d arcCenter = CalculateArcCenterLeft(this.coordinate, arcRadius, this.direction );

        // calculate the radial direction from arc radial center to the current coordinate
        double   currentRadialDirection = this.direction - Util.HALF_PI;

        // calculate the new radial direction once deflection is applied.
        double   newRadialDirection = currentRadialDirection + radDeflectionAngle;

        // calculate new coordinate and new robot direction
        this.setPosition( CalculatePointOnCircle( arcCenter, arcRadius, newRadialDirection ) );

        // add radial deflection angle because it is signed consistently with CCW rotation
        this.setDirectionRad( Util.OptomizeAngleZero_TwoPi( this.direction + radDeflectionAngle ) );
    }

    /**
     * to calculate the radial center of the arc, we need the starting point of the arc and
     * the tangent direction of the arc at that starting point.  From that we can calculate
     * the perpendicular direct, multiply by the arc radius adn add that value to the current
     * coordinate to get the center point.
     *
     * In 2D the perpendicular direction is the revers of direction components where x becomes y
     * and y becomes x using a unit vector so to simplify our calc we just swap sin for cos
     * in our direction calc before multiplying the distance
     *
     * To offset right we multiply by positive arcRadius.
     * */
    static public Vector2d CalculateArcCenterRight( final Vector2d startPoint, double arcRadius, double xAxisRelativeCCWStartDirection ){
        Vector2d arcCenter = new Vector2d(startPoint);
        arcCenter.x += ( Math.sin(xAxisRelativeCCWStartDirection) * Math.abs(arcRadius));
        arcCenter.y += ( Math.cos(xAxisRelativeCCWStartDirection) * Math.abs(arcRadius));
        return arcCenter;
    }

    /**
     * For left arc turn, the radial center point is to the lef of our current coordinate,
     * and the radial direction is + 90 Degrees from our Current robot direction
     * We can then add the deflection angle +- will work to get both our
     * final robot direction and the new radial direction.
     *
     * */
    public void moveArcTurnRight( double arcRadius, double deflectionAngleRight ) {
        // convert deflection angle to radians
        double   radDeflectionAngle = Math.toRadians(deflectionAngleRight);

        // calculate the arc radial center point on the field
        Vector2d arcCenter = CalculateArcCenterRight(this.coordinate, arcRadius, this.direction );

        // calculate the radial direction from arc radial center to the current coordinate
        double   currentRadialDirection = this.direction + Util.HALF_PI;

        // calculate the new radial direction once deflection is applied.
        double   newRadialDirection = currentRadialDirection + radDeflectionAngle;

        // calculate new coordinate and new robot direction
        this.setPosition( CalculatePointOnCircle( arcCenter, arcRadius, newRadialDirection ) );

        // subtract radial deflection angle because it is signed for reverse CW rotation
        this.setDirectionRad( Util.OptomizeAngleZero_TwoPi( this.direction - radDeflectionAngle ) );
    }

    /**calculates new position from current without changing current position*/
    public Vector2d CalculatePosition( final DirectionDistance dd ){
        return dd.asVector2d().add(this.coordinate);
    }

    /**calculates new position from current without changing current position*/
    public Vector2d CalculatePosition( final Vector2d vec ){
        return new Vector2d(this.coordinate).add(vec);
    }
}
