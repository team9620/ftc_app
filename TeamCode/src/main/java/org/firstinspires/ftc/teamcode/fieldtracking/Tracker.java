package org.firstinspires.ftc.teamcode.fieldtracking;

import android.support.annotation.Nullable;

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
import org.firstinspires.ftc.teamcode.utilities.Units;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Created by ROUS on 1/8/2017.
 */
public class Tracker {


        public static final String TAG = "Vuforia Tracker";

        public static final VectorF PhoneOnBot = new VectorF(9f,2f,6f,0f);


        /**
         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
         * localization engine.
         */
        protected VuforiaLocalizer vuforia = null;
        protected static final String VuforiaKey = "AS/XRrf/////AAAAGQZ0+7V9tk+NsbzOnFEyXgVbAH+jDTcbQWffgMSilJRREZ/1pN9FVdp+ITnMY3+6BHjImPV85TKL0rjb/E3TXjrC2ehurR9gbWRpoc77TMFrc3AzSdOGqPs+xvvp92lNpHneD80gKnefCzpIxlu5PBDbJ5hF4zkCwPx2hUcg9mPodX2EcFpRyRPtqZOnkhebymfUWHk7Ndslf4zcdJ3iiI4J7Fq2d80sR9jy745PeQ2nySazvbVWGUY4VDnKl5B2g2VD0UlMv1dc4AvBL1vvnR4ufgIDPDBreMiMDwwgQXeMoffjVrSzJpjqxnCo3EdlNkTnBX5jp7QPXkSpPJRM/JsKH4RFGpwZK7Y8BrwfvtQq\";\n";
        protected VuforiaTrackables trackables = null;
        protected List<VuforiaTrackable> allTrackables = null;
        protected Telemetry telemetry = null;
        protected VectorF cameraPosOnRobot = null;

        protected Map<String,VuforiaTarget> visibleTrackables = null;
        protected OpenGLMatrix lastKnownLocation = null; /** location of bot */
        protected OpenGLMatrix lastKnownTrackableLocation = null;      /** location of image relative to bot */
        protected String       lastKnownTrackableName = null; /** name of image observed */
        protected OpenGLMatrix lastKnownPose = null;      /** location of image relative to bot */

        public static String FormatMatrix( final OpenGLMatrix mat ){
            return mat.formatAsTransform();
        }

        //This method is to intialize Vuforia
        public void intializefunction(Telemetry telemetryin, VectorF cameraPosOnRobotin) throws InterruptedException {

            telemetry = telemetryin;
            cameraPosOnRobot = new VectorF( cameraPosOnRobotin.getData() );
            /**
             * Start up Vuforia.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AS/XRrf/////AAAAGQZ0+7V9tk+NsbzOnFEyXgVbAH+jDTcbQWffgMSilJRREZ/1pN9FVdp+ITnMY3+6BHjImPV85TKL0rjb/E3TXjrC2ehurR9gbWRpoc77TMFrc3AzSdOGqPs+xvvp92lNpHneD80gKnefCzpIxlu5PBDbJ5hF4zkCwPx2hUcg9mPodX2EcFpRyRPtqZOnkhebymfUWHk7Ndslf4zcdJ3iiI4J7Fq2d80sR9jy745PeQ2nySazvbVWGUY4VDnKl5B2g2VD0UlMv1dc4AvBL1vvnR4ufgIDPDBreMiMDwwgQXeMoffjVrSzJpjqxnCo3EdlNkTnBX5jp7QPXkSpPJRM/JsKH4RFGpwZK7Y8BrwfvtQq";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; //.BACK | .FRONT
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

            float mmBotWidth = Units.inchtomm(18f);            // ... or whatever is right for your robot
            float mmFTCFieldWidth = Units.inchtomm(12f * 12f - 2f);   // the FTC field is ~11'10" center-to-center of the glass panels
            float mmTargetNearOffset = Units.inchtomm(12f);
            float mmTargetFarOffset = Units.inchtomm(36f);
            float mmTargetZHeight = Units.inchtomm(5f);

            /*
            *  GEARS target
            * */
            OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                    .translation(-mmFTCFieldWidth / 2f, -mmTargetNearOffset, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 90f, 0f));
            trackGears.setLocation(gearsTargetLocationOnField);
            RobotLog.ii(TAG, "Gears Target=%s", FormatMatrix(gearsTargetLocationOnField));

            /*
            *  TOOLS target
            * */
            OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(-mmFTCFieldWidth / 2f, mmTargetFarOffset, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 90f, 0f));
            trackTools.setLocation(toolsTargetLocationOnField);
            RobotLog.ii(TAG, "Tools Target=%s", FormatMatrix(toolsTargetLocationOnField));

            /*
            *  LEGOS target
            * */
            OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(-mmTargetFarOffset, mmFTCFieldWidth / 2f, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90f, 0f, 0f));
            trackLegos.setLocation(legosTargetLocationOnField);
            RobotLog.ii(TAG, "Legos Target=%s", FormatMatrix(legosTargetLocationOnField));

            /*
            *  WHEELS target
            * */
            OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                    .translation(mmTargetNearOffset, mmFTCFieldWidth / 2f, mmTargetZHeight)
                    .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                            AxesReference.EXTRINSIC, AxesOrder.XZX,
                            AngleUnit.DEGREES, 90, 0, 0));
            trackWheels.setLocation(wheelsTargetLocationOnField);
            RobotLog.ii(TAG, "Wheels Target=%s", FormatMatrix(wheelsTargetLocationOnField));

            /**
             * Phone transform describes offset location, direction, and orientation
             * of phone from robot center
             * */
            OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                    .translation(mmBotWidth / 2, 0, 0)
                    .multiplied(Orientation.getRotationMatrix(
                            AxesReference.EXTRINSIC, AxesOrder.YZY,
                            AngleUnit.DEGREES, -90, 0, 0));
            RobotLog.ii(TAG, "phone=%s", FormatMatrix(phoneLocationOnRobot));

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
        }

    public void beginTracking() throws InterruptedException {
        /** Start tracking the data sets we care about. */
        if (trackables != null) {
            trackables.activate();
        }
    }

    public boolean isInitialized() {
        return (null != this.vuforia && null != this.telemetry);
    }

    /**
     * call after tracking started to update last know location
     * @throws InterruptedException
     */
    public void updateLastKnowLocation() throws InterruptedException {

        if ( !isInitialized() ) { return; }

        /**
         * getUpdatedRobotLocation() will return null if no new information is available since
         * the last time that call was made, or if the trackable is not currently visible.
         * getRobotLocation() will return null if the trackable is not currently visible.
         */
        for (VuforiaTrackable trackable : this.allTrackables) {

            boolean bVisible = ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible();
            if ( !bVisible ) {
                visibleTrackables.remove(trackable.getName());
            } else {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                OpenGLMatrix imageLocation = ((VuforiaTrackableDefaultListener)trackable.getListener()).getPose();
                if (robotLocationTransform != null)
                {
                    this.lastKnownTrackableLocation = trackable.getLocation();
                    this.lastKnownLocation = robotLocationTransform;
                    this.lastKnownTrackableName = trackable.getName();
                    if (imageLocation != null ) {
                        this.lastKnownPose = imageLocation;
                        visibleTrackables.put(trackable.getName(), new VuforiaTarget( trackable.getName(), trackable.getLocation(), robotLocationTransform, this.lastKnownPose ));

                    }
                } else {
                    visibleTrackables.remove(trackable.getName());
                }
            }
        }
    }

    /**
     *
     * @return boolean indicating at lease one target was observed and valid field _position was calculated
     */
    public boolean haveObservations() {
        return !visibleTrackables.isEmpty();
    }

    /**
     * returns the observation that represents the closest target to the robot.
     * under the assumption that this observation is the most accurate.
     */
    public @Nullable
    VuforiaTarget getCurrentObservation() {
        VuforiaTarget obs = null;
        for ( Map.Entry<String, VuforiaTarget> entry : this.visibleTrackables.entrySet() )
        {
            if ( entry.getValue().isValid() ){
                VuforiaTarget cur = entry.getValue();
                if ( null == obs ) { obs = cur; }
                else {
                    if ( cur.getDistanceToTarget() < obs.getDistanceToTarget() ) {
                        obs = cur;
                    }
                }
            }
        }
        return obs;
    }

    /**
     *
     * @param trackableName - name of trackable to look for.
     * @return the observation associated with the trackable name or null if not presently observed
     */
    public @Nullable
    VuforiaTarget getObservationByName(final String trackableName ) {
        VuforiaTarget obs = this.visibleTrackables.get( trackableName );
        return obs;
    }

     // format best current target observation as a string.
     String formatAsString () {
        VuforiaTarget obs = getCurrentObservation();
        return (( null != obs ) ? obs.formatAsString() : "");
    }

    /**
     * updates observation set and returns current when available
     *
     * @return VuforiaTarget if observed, otherwise null
     * @throws InterruptedException
     */
    public VuforiaTarget updateAndGetCurrentObservation()throws InterruptedException {
        updateLastKnowLocation();
        return getCurrentObservation();
    }
}

