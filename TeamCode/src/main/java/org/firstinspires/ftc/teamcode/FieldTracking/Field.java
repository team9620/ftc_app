package org.firstinspires.ftc.teamcode.fieldtracking;


/**
 * Created by ROUS on 3/4/2017.
 */
public class Field {
    /**
     * This class exists to describe the coordinates of objects and fixed positions on the field.
     *
     *                            -X
     *     red             gears          tools
     *         +--------------O----+---------O---------+
     *         ||  /               |                  /|
     *         ||/                 |                /  |
     *         |                   |             /     |
     *  +----+ |                   |          /        O legos
     *  |red | |                   |       /           |
     *  |    | |                   |    /              |
     *  |    | |                +--+--+                |
     *  | -Y | +----------------+  +  +----------------+   Y
     *  |    | |                +--+--+                |
     *  |    | |              /    |                   O wheels
     *  |    | |           /       |                   |
     *  +----+ |        /          |                   |
     *         |     /             |                   |
     *         |  /                |                 /||
     *         |/                  |               /  ||
     *         +-------------------+-------------------+
     *                  +---------------------+           blue
     *                  |          X   blue   |
     *                  +---------------------+
     *
     *
     *  NOTE: all coordinates and dimensions in inches, angles in Degrees
     *
     */

    public static double   FIELD_WIDTH_D = 144.0;
    public static double   FIELD_HALF_WIDTH_D = FIELD_WIDTH_D/2.0;
    public static double   NEAR_BEACON_OFFSET = 12.0;
    public static double   FAR_BEACON_OFFSET = 36.0;
    public static double   BEACON_PRESS_WALL_DIST = 10.57;
    public static double   BEACON_BUTTON_OFFSET = 5.3/2.0; // Beacon center to button.


    public static Vector2d BLUE_CENTER_VORTEX_XY = DirectionDistance.CreateVector2dDeg(45.0, 15.0);
    public static Vector2d BLUE_WHEELS_BEACON_XY = new Vector2d(+NEAR_BEACON_OFFSET, +FIELD_HALF_WIDTH_D);
    public static Vector2d BLUE_LEGOS_BEACON_XY = new Vector2d(-FAR_BEACON_OFFSET, +FIELD_HALF_WIDTH_D);

    /**Coordinates associated with Blue Beacon Button Nav*/
    public static Vector2d BLUE_WHEELS_LINE_POS = BLUE_WHEELS_BEACON_XY.added(0.0, -BEACON_PRESS_WALL_DIST );// -Y dist from wall
    public static Vector2d BLUE_LEGOS_LINE_POS = BLUE_LEGOS_BEACON_XY.added(0.0, -BEACON_PRESS_WALL_DIST );// -Y dist from wall

    public static Vector2d BLUE_POSITION1 = new Vector2d(FIELD_HALF_WIDTH_D-9.0, -36.0 ); //left position
    public static Vector2d BLUE_POSITION2 = new Vector2d(FIELD_HALF_WIDTH_D-9.0,   0.0 ); //center position
    public static Vector2d BLUE_POSITION3 = new Vector2d(FIELD_HALF_WIDTH_D-9.0, +22.25); //right position
    public static Vector2d BLUE_POSITION4 = new Vector2d(FIELD_HALF_WIDTH_D-10.25, +25.25); //right angled 135 degrees position


    public static Vector2d RED_GEARS_BEACON_XY = new Vector2d(-FIELD_HALF_WIDTH_D, -FAR_BEACON_OFFSET );
    public static Vector2d RED_TOOLS_BEACON_XY = new Vector2d(-FIELD_HALF_WIDTH_D, +FAR_BEACON_OFFSET );
    // calculating this one requires understanding the position of the robot center relative to width and length.
    public static Vector2d RED_START_LEFT_XY = new Vector2d( -8.0, -FIELD_HALF_WIDTH_D + 9.0 );

}
