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

    /**copy constructor intializes the object with a copy of other*/
    public SimpleCoordinateTracker(final SimpleCoordinateTracker other) {
        this.direction = other.direction;
        this.coordinate = new Vector2d(other.coordinate);
    }

    /**Format robot coordinate position and direction as a string*/
    public String formatAsString(){
        return String.format("%s -> %.08fd", this.coordinate.formatAsString(), Math.toDegrees(this.direction));
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
        this.coordinate.add(DirectionDistance.CreateVector2dRad( this.direction, distance));
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

     /**
     * For left arc turn, the radial center point is to the lef of our current coordinate,
     * and the radial direction is + 90 Degrees from our Current robot direction
     * We can then add the deflection angle +- will work to get both our
     * final robot direction and the new radial direction.
     *
     * */
    public void moveArcTurnLeftDeg( double arcRadius, double deflectionAngleLeft ) {
            moveArcTurnLeftRad(arcRadius, Math.toRadians(deflectionAngleLeft));
    }
    public void moveArcTurnLeftRad( double arcRadius, double deflectionAngleLeft ) {

        // convert deflection angle to ccw radians
        double   radCCWDeflection = deflectionAngleLeft;

        // calculate dx,dy by rotating the radial vector from 0,0
        Vector2d dir = Vector2d.UnitVectorRad(this.direction);
        Vector2d center = dir.PerpendicularCCW().multiplied(Math.abs(arcRadius));
        Vector2d radial =  dir.PerpendicularCW().multiplied(Math.abs(arcRadius));
        Vector2d newRadial = radial.rotatedByCCWAngleRad(radCCWDeflection);
        Vector2d dxdy = Vector2d.Add(center,newRadial);
        // new coordinate
        this.coordinate.add( dxdy );
        // new direction = cur direction + ccwDeflection angle
        this.direction = Util.OptomizeAngleZero_TwoPi( this.direction + radCCWDeflection );
    }

    /**
     * For right arc turn, the radial center point is to the right of our current coordinate,
     * and the radial direction is + 90 Degrees from our Current robot direction
     * We can then add the deflection angle +- will work to get both our
     * final robot direction and the new radial direction.
     *
     * */
    public void moveArcTurnRightDeg( double arcRadius, double deflectionAngleRight ) {
        moveArcTurnRightRad(arcRadius, Math.toRadians(deflectionAngleRight));
    }
    public void moveArcTurnRightRad( double arcRadius, double deflectionAngleRight ) {

        // convert deflection angle to ccw radians
        double   radCCWDeflection = -deflectionAngleRight;

        // calculate dx,dy by rotating the radial vector from 0,0
        Vector2d dir = Vector2d.UnitVectorRad(this.direction);
        Vector2d center = dir.PerpendicularCW().multiplied(Math.abs(arcRadius));
        Vector2d radial =  dir.PerpendicularCCW().multiplied(Math.abs(arcRadius));
        Vector2d newRadial = radial.rotatedByCCWAngleRad(radCCWDeflection);
        Vector2d dxdy = Vector2d.Add(center,newRadial);

        // new coordinate
        this.coordinate.add( dxdy );
        // new direction = cur direction + ccwDeflection angle
        this.direction = Util.OptomizeAngleZero_TwoPi( this.direction + radCCWDeflection );
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
