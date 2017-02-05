package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Created by ROUS on 1/8/2017.
 */
public class Tracker {


        public static final String TAG = "Vuforia Tracker";

        OpenGLMatrix lastLocation = null;

        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        VuforiaLocalizer vuforia;
        public static final String VuforiaKey = "AS/XRrf/////AAAAGQZ0+7V9tk+NsbzOnFEyXgVbAH+jDTcbQWffgMSilJRREZ/1pN9FVdp+ITnMY3+6BHjImPV85TKL0rjb/E3TXjrC2ehurR9gbWRpoc77TMFrc3AzSdOGqPs+xvvp92lNpHneD80gKnefCzpIxlu5PBDbJ5hF4zkCwPx2hUcg9mPodX2EcFpRyRPtqZOnkhebymfUWHk7Ndslf4zcdJ3iiI4J7Fq2d80sR9jy745PeQ2nySazvbVWGUY4VDnKl5B2g2VD0UlMv1dc4AvBL1vvnR4ufgIDPDBreMiMDwwgQXeMoffjVrSzJpjqxnCo3EdlNkTnBX5jp7QPXkSpPJRM/JsKH4RFGpwZK7Y8BrwfvtQq\";\n";
        protected VuforiaTrackables trackables;
        protected  List<VuforiaTrackable> allTrackables;
        protected Telemetry telemetry;
        protected VectorF cameraPosOnRobot;

        //This method is to intialize Vuforia
        public void intializefunction(Telemetry telemetryin, VectorF cameraPosOnRobotin) throws InterruptedException {

            telemetry = telemetryin;
            cameraPosOnRobot = cameraPosOnRobotin;
            /**
             * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
             * the camera monitor feedback; if no camera monitor feedback is desired, use the parameterless
             * constructor instead. We also indicate which camera on the RC that we wish to use. For illustration
             * purposes here, we choose the back camera; for a competition robot, the front camera might
             * prove to be more convenient.
             *
             * Note that in addition to indicating which camera is in use, we also need to tell the system
             * the location of the phone on the robot; see phoneLocationOnRobot below.
             *
             * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
             * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
             * Vuforia will not load without a valid license being provided. Vuforia 'Development' license
             * keys, which is what is needed here, can be obtained free of charge from the Vuforia developer
             * web site at https://developer.vuforia.com/license-manager.
             *
             * Valid Vuforia license keys are always 380 characters long, and look as if they contain mostly
             * random data. As an example, here is a example of a fragment of a valid key:
             *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
             * Once you've obtained a license key, copy the string form of the key from the Vuforia web site
             * and paste it in to your code as the value of the 'vuforiaLicenseKey' field of the
             * {@link Parameters} instance with which you initialize Vuforia.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AS/XRrf/////AAAAGQZ0+7V9tk+NsbzOnFEyXgVbAH+jDTcbQWffgMSilJRREZ/1pN9FVdp+ITnMY3+6BHjImPV85TKL0rjb/E3TXjrC2ehurR9gbWRpoc77TMFrc3AzSdOGqPs+xvvp92lNpHneD80gKnefCzpIxlu5PBDbJ5hF4zkCwPx2hUcg9mPodX2EcFpRyRPtqZOnkhebymfUWHk7Ndslf4zcdJ3iiI4J7Fq2d80sR9jy745PeQ2nySazvbVWGUY4VDnKl5B2g2VD0UlMv1dc4AvBL1vvnR4ufgIDPDBreMiMDwwgQXeMoffjVrSzJpjqxnCo3EdlNkTnBX5jp7QPXkSpPJRM/JsKH4RFGpwZK7Y8BrwfvtQq";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

            /**
             * Load the data sets that for the trackable objects we wish to track. These particular data
             * sets are stored in the 'assets' part of our application (you'll see them in the Android
             * Studio 'Project' view over there on the left of the screen). You can make your own datasets
             * with the Vuforia Target Manager: https://developer.vuforia.com/target-manager. PDFs for the
             * example "StonesAndChips", datasets can be found in in this project in the
             * documentation directory.
             */
            trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");
            VuforiaTrackable trackWheels = trackables.get(0);
            trackWheels.setName("Wheels");  //Wheels

            VuforiaTrackable trackTools = trackables.get(1);
            trackTools.setName("Tools");  //Tools

            VuforiaTrackable trackLegos = trackables.get(2);
            trackLegos.setName("Legos");  //Legos

            VuforiaTrackable trackGears = trackables.get(3);
            trackGears.setName("Gears");  //Gears

            /** For convenience, gather together all the trackable objects in one easily-iterable collection */
            allTrackables = new ArrayList<VuforiaTrackable>();
            allTrackables.addAll(trackables);

            /**
             * We use units of mm here because that's the recommended units of measurement for the
             * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
             *      <ImageTarget name="stones" size="247 173"/>
             * You don't *have to* use mm here, but the units here and the units used in the XML
             * target configuration files *must* correspond for the math to work out correctly.
             */
            float mmBotWidth = Units.intomm(18f);            // ... or whatever is right for your robot
            float mmFTCFieldWidth = Units.intomm(12f * 12f - 2f);   // the FTC field is ~11'10" center-to-center of the glass panels
            float mmTargetNearOffset = Units.intomm(12f);
            float mmTargetFarOffset = Units.intomm(36f);
            float mmTargetZHeight = Units.intomm(5f);
            /**
             * In order for localization to work, we need to tell the system where each target we
             * wish to use for navigation resides on the field, and we need to specify where on the robot
             * the phone resides. These specifications are in the form of <em>transformation matrices.</em>
             * Transformation matrices are a central, important concept in the math here involved in localization.
             * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
             * for detailed information. Commonly, you'll encounter transformation matrices as instances
             * of the {@link OpenGLMatrix} class.
             *
             * For the most part, you don't need to understand the details of the math of how transformation
             * matrices work inside (as fascinating as that is, truly). Just remember these key points:
             * <ol>
             *
             *     <li>You can put two transformations together to produce a third that combines the effect of
             *     both of them. If, for example, you have a rotation transform R and a translation transform T,
             *     then the combined transformation matrix RT which does the rotation first and then the translation
             *     is given by {@code RT = T.multiplied(R)}. That is, the transforms are multiplied in the
             *     <em>reverse</em> of the chronological order in which they applied.</li>
             *
             *     <li>A common way to create useful transforms is to use methods in the {@link OpenGLMatrix}
             *     class and the Orientation class. See, for example, {@link OpenGLMatrix#translation(float,
             *     float, float)}, {@link OpenGLMatrix#rotation(AngleUnit, float, float, float, float)}, and
             *     {@link Orientation#getRotationMatrix(AxesReference, AxesOrder, AngleUnit, float, float, float)}.
             *     Related methods in {@link OpenGLMatrix}, such as {@link OpenGLMatrix#rotated(AngleUnit,
             *     float, float, float, float)}, are syntactic shorthands for creating a new transform and
             *     then immediately multiplying the receiver by it, which can be convenient at times.</li>
             *
             *     <li>If you want to break open the black box of a transformation matrix to understand
             *     what it's doing inside, use {@link MatrixF#getTranslation()} to fetch how much the
             *     transform will move you in x, y, and z, and use {@link Orientation#getOrientation(MatrixF,
             *     AxesReference, AxesOrder, AngleUnit)} to determine the rotational motion that the transform
             *     will impart. See {@link #format(OpenGLMatrix)} below for an example.</li>
             *
             * </ol>
             *
             * This example places the "stones" image on the perimeter wall to the Left
             *  of the Red Driver station wall.  Similar to the Red Beacon Location on the Res-Q
             *
             * This example places the "chips" image on the perimeter wall to the Right
             *  of the Blue Driver station.  Similar to the Blue Beacon Location on the Res-Q
             *
             * See the doc folder of this project for a description of the field Axis conventions.
             *
             * Initially the target is conceptually lying at the origin of the field's coordinate system
             * (the center of the field), facing up.
             *
             * In this configuration, the target's coordinate system aligns with that of the field.
             *
             * In a real situation we'd also account for the vertical (Z) offset of the target,
             * but for simplicity, we ignore that here; for a real robot, you'll want to fix that.
             *
             * To place the Stones Target on the Red Audience wall:
             * - First we rotate it 90 around the field's X axis to flip it upright
             * - Then we rotate it  90 around the field's Z access to face it away from the audience.
             * - Finally, we translate it back along the X axis towards the red audience wall.
             */
            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                    .translation(-mmFTCFieldWidth / 2f, -mmTargetNearOffset, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 90f, 0f));
            trackGears.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

       /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(-mmFTCFieldWidth / 2f, mmTargetFarOffset, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 90f, 0f));
            trackTools.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

         /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(-mmTargetFarOffset, mmFTCFieldWidth / 2f, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 0f, 0f));
            trackLegos.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));

         /*
        * To place the Stones Target on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
            OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(mmTargetNearOffset, mmFTCFieldWidth / 2f, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            trackWheels.setLocation(wheelsTargetLocationOnField);
            RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

            /**
             * Create a transformation matrix describing where the phone is on the robot. Here, we
             * put the phone on the right hand side of the robot with the screen facing in (see our
             * choice of BACK camera above) and in landscape mode. Starting from alignment between the
             * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
             *
             * When determining whether a rotation is positive or negative, consider yourself as looking
             * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
             * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
             * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
             * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
             */
            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth / 2, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZY,
                            AngleUnit.DEGREES, -90, 0, 0));
            RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

            /**
             * Let the trackable listeners we care about know where the phone is. We know that each
             * listener is a {@link VuforiaTrackableDefaultListener} and can so safely cast because
             * we have not ourselves installed a listener of a different type.
             */
            ((VuforiaTrackableDefaultListener) trackGears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) trackWheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) trackLegos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
            ((VuforiaTrackableDefaultListener) trackTools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);


            /**
             * A brief tutorial: here's how all the math is going to work:
             *
             * C = phoneLocationOnRobot  maps   phone coords -> robot coords
             * P = tracker.getPose()     maps   image target coords -> phone coords
             * L = redTargetLocationOnField maps   image target coords -> field coords
             *
             * So
             *
             * C.inverted()              maps   robot coords -> phone coords
             * P.inverted()              maps   phone coords -> imageTarget coords
             *
             * Putting that all together,
             *
             * L x P.inverted() x C.inverted() maps robot coords to field coords.
             *
             * @see VuforiaTrackableDefaultListener#getRobotLocation()
             */

            /** Wait for the game to begin */
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            //waitForStart();
        }

    public void beginTracking() throws InterruptedException {
        /** Start tracking the data sets we care about. */
        if (trackables != null) {
            trackables.activate();
        }
    }

    public void updateLastKnownLocation() throws InterruptedException {

        for (VuforiaTrackable trackable : allTrackables) {
            /**
             * getUpdatedRobotLocation() will return null if no new information is available since
             * the last time that call was made, or if the trackable is not currently visible.
             * getRobotLocation() will return null if the trackable is not currently visible.
             */
            telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }
        }
        /**
         * Provide feedback as to where the robot was last located (if we know).
         */
        if (lastLocation != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetry.addData("Pos", format(lastLocation));
        } else {
            telemetry.addData("Pos", "Unknown");
        }
        telemetry.update();
    }


    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        VectorF pos = transformationMatrix.getTranslation();
        return String.format(Locale.getDefault(), "%.04f, %.04f", Units.mmtoint(pos.get(0)), Units.mmtoint(pos.get(1)));

    }
}

