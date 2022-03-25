/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * ©Thobor 2019-2020
 *
 *           _
 *       .__(.)< (MEOW)
 *        \___)   
 * ~~~~~~~~~~~~~~~~~~
 *
 *
 */



package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Skystone_Hanga", group="Hanga")
public class Skystone_Hanga extends LinearOpMode {

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AT0wukP/////AAABmf2bN3XJaEQjssVFQTY/tbpBmxq2HJn4pooNk0lDszeT1PG7XhV17uBFclV5ILXSgsxUTc0Fepi7bAL5sJD0KW7GWxnoBiLUM67TF6mcD5dNi0vYVJ6iTgWSuHyTdab/nm9jDuZenQpMs6/XHL7nva/NbepfRn49kIrVq0rGQvH51c3z5OqDlxNMUg5Q60g0UroDxmIVLJsPbRPj8HLC3zQEfvz5uXfCWg853pF06UrCRDILcTXz2WUCEJwQMiniJyWSrz6tN96YT1k8dhScMIjpyc7KtsAm0lxYApq23rTieciVzRB7+OMFbgFEuA/BetmBt8+z2ybMZPLgR0veOoU2uT1qu2wBcj+GXKHjRCeX";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float cmPerInch        = 2.54f;
    private static final float mmPerCm          = 10.0f;

    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    
    private ElapsedTime runtime = new ElapsedTime();



    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;



    //
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    double AxaX = 0;
    double AxaY = 0;
    double AxaZ = 0;

    double RotX = 0;
    double RotY = 0;
    double RotZ = 0;


    Servo gheara1, gheara2;

    Servo capstone;


    WebcamName webcamName = null;

    //28 * 20 / (2ppi * 4.125)
    Double width = 12.99; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //

    int am_fost = 0;

    int am_reglat = 0;



    public void runOpMode(){
        //
        initGyro();
        //
        initHard();

        gheara1.setPosition(1);
        gheara2.setPosition(0);
/*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        
        /*

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        */



        final float CAMERA_FORWARD_DISPLACEMENT  = 6.92f * mmPerInch;  // 17,6 distanta totala = 6,92 inch
        final float CAMERA_VERTICAL_DISPLACEMENT = 3.74f * mmPerInch; //15cm 
        final float CAMERA_LEFT_DISPLACEMENT     = 7.87f * mmPerInch; // 15cm » capat


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:




        //
        InLocDeWaitForStart();


        if(am_fost == 0)
        {
            miscare_stanga_encoder(0.5,19);
            am_fost = 1;
        }

        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

/*
 *               __
 *              / _)  (GRRRRR)
 *       .-^^^-/ /          
 *   __/       /              
 *  <__.|_|-|_|  
 *
*/

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) 
            {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                AxaX = translation.get(0)/mmPerInch;
                AxaY = translation.get(1)/mmPerInch;
                AxaZ = translation.get(2)/mmPerInch;

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

                RotX = rotation.firstAngle;
                RotY = rotation.secondAngle;
                RotZ = rotation.thirdAngle;


                if (targetVisible == true) 
                {
                    
                    //miscari pe aici   
                    double RetinX = 0;
                    
                    //miscare_stanga_encoder(0.5,AxaX-20,5);
                    if(am_fost == 1)
                    {
                        if(am_reglat == 0) // PENTRU A MA REGLA LA CENTRU CUBULUI CU CAMERA
                        {
                            if (AxaY > 0) 
                            {
                                moveToPosition(-AxaY, 0.5);

                                if(AxaX > 0) 
                                {
                                    RetinX = AxaX - 2.5;
                                }
                                else 
                                {
                                    RetinX = -AxaX - 2.5;
                                }
                                am_reglat = 1;  
                            }
                            else
                            {
                                
                                moveToPosition(AxaY, 0.5);   
                                if(AxaX > 0) 
                                {
                                    RetinX = AxaX - 2.9;
                                }
                                else 
                                {
                                    RetinX = -AxaX - 2.9;
                                }
                                am_reglat = 1;
                            }
                            
                        }

                       //turnWithGyro(45, 0.3);
                        rotate(-45, 0.3);


                        /*while(AxaX  > 20)
                        {
                            deplasare(0.2,-0.2,-0.2,0.2);
                        } */
                        //deplasare(0, 0, 0, 0);
                       /* miscare_stanga_encoder(0.5,RetinX);
                        miscare_fata_encoder(0.6, 8.9); // pozitionare de la centru cubului pentru a agata lateralul
                        // 60 cm intre cele 2 cuburi
                        lateral2.setPosition(0.6);
                        oprire(200);
                        miscare_dreapta_encoder(0.4,RetinX);
                        miscare_fata_encoder(0.8,12);
                        lateral2.setPosition(0);
                        oprire(200);
                        miscare_spate_encoder(0.8,47);
                        oprire(200);
                        if(am_reglat == 1) // PENTRU A MA REGLA LA CENTRU CUBULUI CU CAMERA
                        {
                            if (AxaY > 0) 
                            {
                                moveToPosition(AxaY, 0.5);

                                if(AxaX > 0) 
                                {
                                    RetinX = AxaX - 2.7;
                                }
                                else 
                                {
                                    RetinX = -AxaX - 2.7;
                                }
                                am_reglat = 2;  
                            }
                            else
                            {
                                
                                moveToPosition(-AxaY, 0.5);   
                                if(AxaX > 0) 
                                {
                                    RetinX = AxaX - 2.7;
                                }
                                else 
                                {
                                    RetinX = -AxaX - 2.7;
                                }
                                am_reglat = 2;
                            }
                            
                        }

                        
                        miscare_stanga_encoder(0.7,RetinX);
                        //miscare_fata_encoder(0.6, 8.9); // pozitionare de la centru cubului pentru a agata lateralul
                        lateral2.setPosition(0.6);
                        oprire(200);
                        miscare_dreapta_encoder(0.7,RetinX);
                        miscare_fata_encoder(0.8,59);
                        lateral2.setPosition(0);
                        oprire(200);
                        am_fost = 2;*/

                    }
                   
                    break;
                }
                else if (targetVisible == false) {
                    //strafe right 8 inches
                    strafeToPosition(8, .5); // o sa revin sa fac asta

                } else {
                    deplasare(0,0,0,0) ;
                }

            }
            else {
                
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();



    //
    }
    //




/*
+-------------------+-------------------+
| /                 |                 \ |
|/                  |                  \|
|                   |                   |
| BLUE BUILD ZONE   |  RED BUILDING ZONE|
|                   |                   |
|                   |                   |
|                   |                   |
|                   |                   |
+-------------------+-------------------+
|                   |                   |
|                   |                   |
|                   |                   |
| BLUE LOAD ZONE    |  RED LOAD ZONE    |
|                   |                   |
|                   |                   |
|------             |             ------|
|     |             |             |     |
|     |             |             |     |
+-------------------+-------------------+
*/



    public void oprire(long milisecunde) {
        deplasare(0,0,0,0) ;
        sleep(milisecunde);
    }

    public void miscare_dreapta_encoder(double putere, double distanta)
    {
        strafeToPosition(-distanta, putere);
    }
    public void miscare_stanga_encoder(double putere, double distanta){
        strafeToPosition(distanta, putere);
    }
    public void miscare_spate_encoder(double putere, double distanta)
    {
        moveToPosition(distanta, putere);
    }
    public void miscare_fata_encoder(double putere, double distanta)
    {
        moveToPosition(-distanta, putere);
    }

    public void deplasare(double lm1, double lm2, double rm1, double rm2) {
        
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        frontleft.setPower(lm1);
        backleft.setPower(rm1);
        frontright.setPower(lm2);
        backright.setPower(rm2);
    }

    public void moveToPosition(double inches, double speed){
        //



        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //

        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
            /*if(angles.firstAngle>-1 && angles.firstAngle<1)
            { 
                frontleft.setPower(speed);
                backleft.setPower(speed);
                frontright.setPower(speed);
                backright.setPower(speed);
            }
            if(angles.firstAngle>1)
            {
                frontleft.setPower(0.2);
                backleft.setPower(0.2);
                frontright.setPower(-0.2);
                backright.setPower(-0.2); 
            }
                
            if(angles.firstAngle<-1)
            {
                frontleft.setPower(-0.2);
                backleft.setPower(-0.2);
                frontright.setPower(0.2);
                backright.setPower(0.2); 
            }    */



        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit)
            {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */

    public void rotate(int degrees, double power) {

        double leftPower, rightPower;


        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn left.
            leftPower = power;
            rightPower = .3;
            telemetry.addLine("left");
            telemetry.update();
        } else if (degrees > 0) {   // turn right.
            leftPower = -.3;
            rightPower = -power;
            telemetry.addLine("right");
            telemetry.update();
        } else return;

        // set power to rotate.
        frontleft.setPower(leftPower);
        backleft.setPower(leftPower);
        frontright.setPower(rightPower);
        backright.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) //-10
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getHeading() == 0) {
            }

            while (opModeIsActive() && getHeading() < degrees) {
            }
        } else    // right turn.
            while (opModeIsActive() && getHeading() > degrees) {
            }

        // turn the motors off.
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);

        // wait for rotation to stop.
        sleep(1000);


    }

public double getHeading() {
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
            AxesOrder.ZYX, AngleUnit.DEGREES);
    double heading = angles.firstAngle;
    return heading;
  }

    public void turnWithGyro(double degrees, double speedDirection){
        //
        degrees = degrees - 10;
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        
        //
        if (speedDirection > 0){//set target positions
            //turn right
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            
        }else{
            //turn left
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            
        }
        //
        //Go to position
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        
        //
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //

    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){}
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }
    //

    public void InLocDeWaitForStart(){
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) 
        {
            telemetry.update();
        }
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */


    private void initHard()
    {

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");


        //motoare roti
        frontleft = hardwareMap.dcMotor.get("leftFront");
        frontright = hardwareMap.dcMotor.get("rightFront");
        backleft = hardwareMap.dcMotor.get("leftRear");
        backright = hardwareMap.dcMotor.get("rightRear");

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);

        //gheare cuva
        gheara1 = hardwareMap.get(Servo.class,"fdr");
        gheara2 = hardwareMap.get(Servo.class,"fst"); 


        capstone = hardwareMap.get(Servo.class,"capstone");
        gheara1.setPosition(-.5);
        gheara2.setPosition(1);
        capstone.setPosition(-0.6);

    }


    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }
    //
}
