package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import java.util.ArrayList;
import java.util.List;

public class Vuforia {

    public enum CameraChoice {
        PHONE_FRONT, PHONE_BACK, WEBCAM1, WEBCAM2;
    }

    private class VuforiaLocalizer extends VuforiaLocalizerImpl {
        public VuforiaLocalizer(Parameters parameters) {
            super(parameters);
        }

        public void close() {
            super.close();
        }
    }

    public static List<VuforiaTrackable> getTrackables() {
        return allTrackables;
    }


    private static final String VUFORIA_KEY = "Ad0Srbr/////AAABmdpa0/j2K0DPhXQjE2Hyum9QUQXZO8uAVCNpwlogfxiVmEaSuqHoTMWcV9nLlQpEnh5bwTlQG+T35Vir8IpdrSdk7TctIqH3QBuJFdHsx5hlcn74xa7AiQSJgUD/n7JJ2zJ/Er5Hc+b+r616Jf1YU6RO63Ajk5+TFB9N3a85NjMD6eDm+C6f14647ELnmGC03poSOeczbX7hZpIEObtYdVyKZ2NQ/26xDfSwwJuyMgUHwWY6nl6mk0GMnIGvu0/HoGNgyR5EkUQWyx9XlmxSrldY7BIEVkiKmracvD7W9hEGZ2nPied6DTY5RFNuFX07io6+I59/d7291NXKVMDnFAqSt4a2JYsECv+j7b25S0mD";

    private static final float mmPerInch        = 25.4f;                    // constant for converting measurements from inches to millimeters
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    private static final float halfField = 72 * mmPerInch;                  // constants for perimeter targets
    private static final float quadField  = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null; // class members
    private VuforiaLocalizer vuforia = null;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    public VuforiaTrackables targetsUltGoal;
    private static ArrayList<VuforiaTrackable> allTrackables;

    public Vuforia(HardwareMap hardwareMap, CameraChoice cameraChoice) {

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();

        vuforia = initVuforia(hardwareMap, cameraChoice);
    }

    private VuforiaLocalizer initVuforia(HardwareMap hardwareMap, CameraChoice cameraChoice) {
        if (vuforia != null)
            vuforia.close();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.useExtendedTracking = false;

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        switch (cameraChoice) {
            case PHONE_FRONT:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
                break;
            case PHONE_BACK:
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
                break;
            case WEBCAM1:
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
                break;
            case WEBCAM2:
                parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 2");
                break;
        }
        vuforia = new VuforiaLocalizer(parameters);

        // TODO most likely will need to end up establishing precise positions in the future
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        targetsUltGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        VuforiaTrackable blueTowerGoalTarget = targetsUltGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal");
        VuforiaTrackable redTowerGoalTarget = targetsUltGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal");
        VuforiaTrackable redAllianceTarget = targetsUltGoal.get(2);
        redAllianceTarget.setName("Red Alliance");
        VuforiaTrackable blueAllianceTarget = targetsUltGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance");
        VuforiaTrackable frontWallTarget = targetsUltGoal.get(4);
        frontWallTarget.setName("Front Wall");

        allTrackables.addAll(targetsUltGoal);

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        targetsUltGoal.activate();

        return vuforia;
    }

    public void close() {
        vuforia.close();
    }

    public Orientation getRobotHeading() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
    }

    public VectorF getRobotPosition() {
        return lastLocation.getTranslation();
    }

    public boolean isTargetVisible(VuforiaTrackable targetTrackable) {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                if (trackable.getName().equals(targetTrackable.getName())) {
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isAnyTargetVisible() {
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                return true;
            }
        }
        return false;
    }

    public void activate() {
        targetsUltGoal.activate();
    }

    public void disable() {
        targetsUltGoal.deactivate();
    }
}
