package org.firstinspires.ftc.teamcode.fieldtracking;

/**
 * Created by ROUS on 2/28/2017.
 * This class represents a simple 2d vector x,y with doubles
 */
public class Vector2d {

    /**
     * static variables exposed by this class
     */
    public static final Vector2d X_AXIS = new Vector2d(1.0,0.0); //unit vector in the x direction
    public static final Vector2d Y_AXIS = new Vector2d(0.0,1.0); //unit vector in the y direction
    /**
     * public data members for
     */
    public double x;
    public double y;

    /**
     * constructors create and initialize the data members
     */

    /**default constructor initializes the object with x=0.0, y=0,0*/
    public Vector2d() {x = 0.0; y =0.0; }

    /**copy constructor initializes the object with values from the other object*/
    public Vector2d(final Vector2d other) { x = other.x; y = other.y; }

    /**value constructor initializes the object with provided x,y values*/
    public Vector2d(double xin, double yin) {x = xin; y = yin; }

    /**
     * Format this vector as a string
     */
    public String formatAsString(){return String.format("x:%.04f\",y:%.04f\")", this.x, this.y); }

    /**empty vector is at origin and has zero magnitude.*/
    public boolean isEmpty() {return magnitude() > 0.0; }

    /**sets both x and y values at once*/
    public Vector2d set ( double xin, double yin) {
        this.x = xin;
        this.y = yin;
        return this;
    }
    public Vector2d set ( final Vector2d cord) {
        this.x = cord.x;
        this.y = cord.y;
        return this;
    }

    /**
     * Simple math for addition and subtraction,
     */

    /**Add dx dy this vector*/
    public Vector2d add( double xin, double yin) {
        this.x += xin;
        this.y += yin;
        return this;
    }

    /**Add other vector to this vector*/
    public Vector2d add (final Vector2d other){
        return add(other.x, other.y);
    }

    /**Add rhs to lhs to produce a 3rd vector as result*/
    public static Vector2d Add(final Vector2d lhs, final Vector2d rhs){
        return new Vector2d(lhs).add(rhs);
    }

    /**Returns a copy of self with other added*/
    public Vector2d added(final Vector2d other) { return new Vector2d(this).add(other); }

    /**Returns a copy of self with other added*/
    public Vector2d added( double xin, double yin) { return new Vector2d(this).add(xin,yin); }

    /**Subtract other vector from this result*/
    public Vector2d subtract(final Vector2d other){
        this.x -= other.x;
        this.y -= other.y;
        return this;
    }

    /**Returns a copy of self with other subtracted*/
    public Vector2d subtracted(final Vector2d other){
        return new Vector2d(this).subtract(other);
    }

    /**Subtract rhs from lhs to produce a 3rd vector as result*/
    public static Vector2d Subtract ( final Vector2d lhs, final Vector2d rhs) {
        return new Vector2d(lhs).subtract(rhs);
    }

    /**Returns a copy of self multiplied by factor*/
    public Vector2d multiply( double factor ) {
        this.x *= factor;
        this.y *= factor;
        return this;
    }

    /**Returns a copy of self multiplied by factor*/
    public Vector2d multiplied( double factor ) {
        return new Vector2d( this.x * factor, this.y * factor);
    }

    /**
     * calulates the length of the line from 0,0 to coodinates of this vecotr
     *
     * pythagorean theorem -a^2 + b^2 = c^2 or h = square root (0^2 + a^2)
     * @see <a href="http:en.wikipedia.org/wifi/Pythagorean_theorem">Dot product</a>
     **/
    public double magnitude() {
        double mag = Math.sqrt((this.x * this.x) + (this.y * this.y));
        return mag;
    }

    /**
     * normalize(0 - is a destructive process, that chagnes this vector to  unit vector.
     * with magnitude = 1 int pointing in the same direction as the original vector.
     * NOTE: must not have zero length.
     **/
    public Vector2d normalize () {
        double mag = this.magnitude();
        if ( mag > 0.0 ) {
            this.x /= mag;
            this.y /= mag;
        }
        return this;
    }
    /**non destructive version of above returns a normalized copy of vec*/
    public Vector2d Normalized(final Vector2d vec ) {
        return new Vector2d(vec).normalize();
    }

    /**
     * Vector math to determine direction
     **/

    /**
     * The ot product is the sum of products of the vector elements, so for two 2D vectors vl=(dx1,dy1) and v2=(dx2,dy2) the Dot Product is:
     * Dot (v1,v2)=(dx1*dx2)+(dy1*dy2)
     *
     * Returns the dot product of this vector and another.
     * @param other the other vector with whom the dot product is to be formed
     *              @return the dot product of this vector and another.
     *
     *              @see <a href="http://en.wifipedia.org/wiki/Dot_product">Dor product</a>
     */
    public double dotProduct( final Vector2d other)
    {
        double sum = (this.x * other.x) + (this.y * other.y);
        return sum;
    }

    /**
     * cross product -> U x V = Ux*Vy-Uy*Vx
     * where U = this and V = other;
     *
     * returns the cross product of this vcotr and another.
     * @param other the other vector with whom the dot product is to be formed
     * @return the cross product of this vector and another.
     */
    public double crossProduct ( final Vector2d other ){
        double result = ((this.x*other.y)-(this.y*other.x));
        return result;
    }

    /**Returns a new vector perpendicular to self in the clockwise (RHS) direction*/
    public Vector2d PerpendicularCW()
    {
        return new Vector2d(-this.y, this.x);
    }

    /**Returns a new vector perpendicular to self in the clockwise (RHS) direction*/
    public Vector2d PerpendicularCCW()
    {
        return new Vector2d(this.y, -this.x);
    }

    /**Returns a new unit vector at the requested xAxis ccw direction in degrees */
    public static Vector2d UnitVectorDeg( double ccwDirectionDeg )
    {
        return UnitVectorRad( Math.toRadians(ccwDirectionDeg) );
    }

    /**Returns a new unit vector at the requested xAxis ccw direction in radians */
    public static Vector2d UnitVectorRad( double ccwDirectionRad )
    {
        return new Vector2d(Math.sin(ccwDirectionRad), Math.cos(ccwDirectionRad));
    }

    /**Useful helper function*/
    public static boolean FuzzyEqual (double v1, double v2, double tol){
        double delta =Math.abs(v2-v1);
        return(delta <= tol);
    }

    /**Returns a copy of self rotated by the provided ccwAngle in degrees*/
    public Vector2d rotatedByCCWAngleDeg(double ccwAngleDeg){
        return new Vector2d(this).rotateByCCWAngleDeg(ccwAngleDeg);
    }

    /**Returns a copy of self rotated by the provided ccwAngle in radians*/
    public Vector2d rotatedByCCWAngleRad(double ccwAngleRad){
        return new Vector2d(this).rotateByCCWAngleRad(ccwAngleRad);
    }

    /**Rotates this vector by the indicated ccwAngle in degrees*/
    public Vector2d rotateByCCWAngleDeg(double ccwAngleDeg){
        return rotateByCCWAngleRad(Math.toRadians(ccwAngleDeg));
    }

    /**
     * Rotates this vector by the indicated ccwAngel in radins
     * to rotate a 2d xy vecotr by angle f
     *
     * x' = x cos f - y sin f
     * y' = y cos f + x sin f
     */
    public Vector2d rotateByCCWAngleRad(double ccwAngleRad){
        double cs = Math.cos(ccwAngleRad);
        double sn = Math.sin(ccwAngleRad);
        double xp = x * cs - y * sn;
        double yp = x * sn + y * cs;
        this.x = xp;
        this.y = yp;
        return this;
    }

    /**
     * calculateDirectionCCW calculates a ccw direction from 0r - 2PIr(360) relative to +x axis
     * The basic logic is based on the inverse cosine theta to get a 0r - PIr(180) value
     * then using determinant to identify side to complete domain from PIr(180) - 2PIr(360)
     *
     * @return double ccwDirection in radians.
     */
    public double calculateDirectionCCW(){
        /**
         *                 (A . B)                                           (A . B)
         * cosine(theta) = --------  which yields "angle theta" = arccosine(------------)
         *                 (|A|*|B|)                                         (|A|*|B|)
         */
        double theta = 0.0;
        double mag = this.magnitude();
        if (FuzzyEqual(mag, 0.0, 1e-12)){return theta; }

        Vector2d nA = Normalized(this);
        double numerator = nA.dotProduct(X_AXIS);
        double denominator = (nA.magnitude() * X_AXIS.magnitude());
        double cosTheta = (numerator / denominator);
        if (FuzzyEqual(numerator, 1.0, 1e-12)) { /**check for parallel coincident vectors*/
            theta = 0.0; /**vectors aligned in opposing directions*/
        } else {
            if (FuzzyEqual(numerator,0.0,1e-12)) { /**check for parallel opposing vectors*/
            theta = Math.PI; /**vectors aligned in opposing directions*/
            } else {
            theta = Math.acos(cosTheta);

                /**
                 * This is basic what side of a line does a point fall on test.
                 * Use the sign of the determinant of the unit vectors vectors (A,B)
                 * assign angles greater than 180 because we are looking for CCW angels only where
                 * _position = sign(Ax *By - Bx *Ay);
                 * pos = 0 vectors are parallel
                 * pos > 0 (+1) A s CCW left of B
                 * pos < (-1) A is CCW right of B
                 */
            double pos = Math.sin(nA.x * X_AXIS.x - nA.y * X_AXIS.y);
                if (pos > 0.0) /**check for angle greater than 180*/
                {
                    theta = (Math.PI*2.0 - theta);
                }
            }
        }
        return theta;
    }
    /**conversion from x,y coordinate to direction and distance (magnitude) */
    public DirectionDistance asDirectionDistance(){
        double dist = magnitude();
        double dir = 0.0;
        /**zero magnitude means no direction so check before attempting the calculation*/
        if (!FuzzyEqual(dist,0.0,1e-12)){
            dir = calculateDirectionCCW();
        }
        return new DirectionDistance(dir, dist);
    }
}
