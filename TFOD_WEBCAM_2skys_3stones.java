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
 *
 *   . -------------------------------------------------------------------.        
 *   | [Esc] [F1][F2][F3][F4][F5][F6][F7][F8][F9][F0][F10][F11][F12] o o o|        
 *   |                                                                    |        
 *   | [`][1][2][3][4][5][6][7][8][9][0][-][=][_<_] [I][H][U] [N][/][*][-]|        
 *   | [|-][Q][W][E][R][T][Y][U][I][O][P][{][}] | | [D][E][D] [7][8][9]|+||        
 *   | [CAP][A][S][D][F][G][H][J][K][L][;]['][#]|_|           [4][5][6]|_||        
 *   | [^][\][Z][X][C][V][B][N][M][,][.][/] [__^__]    [^]    [1][2][3]| ||        
 *   | [c]   [a][________________________][a]   [c] [<][V][>] [ 0  ][.]|_||        
 *   `--------------------------------------------------------------------'   
 *
 *
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;



import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;


@Autonomous(name="TFOD_WEBCAM_2skys_3stones", group="Hanga")
//@Disabled
public class TFOD_WEBCAM_2skys_3stones extends LinearOpMode
{


    //private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String TFOD_MODEL_ASSET = "detect.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY 
   = "AT0wukP/////AAABmf2bN3XJaEQjssVFQTY/tbpBmxq2HJn4pooNk0lDszeT1PG7XhV17uBFclV5ILXSgsxUTc0Fepi7bAL5sJD0KW7GWxnoBiLUM67TF6mcD5dNi0vYVJ6iTgWSuHyTdab/nm9jDuZenQpMs6/XHL7nva/NbepfRn49kIrVq0rGQvH51c3z5OqDlxNMUg5Q60g0UroDxmIVLJsPbRPj8HLC3zQEfvz5uXfCWg853pF06UrCRDILcTXz2WUCEJwQMiniJyWSrz6tN96YT1k8dhScMIjpyc7KtsAm0lxYApq23rTieciVzRB7+OMFbgFEuA/BetmBt8+z2ybMZPLgR0veOoU2uT1qu2wBcj+GXKHjRCeX";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    int mutat = 0;
    int pozitionat = 0;    

    WebcamName webcamName = null;

    Servo capstone;

    DcMotor absortie1;
    DcMotor absortie2;

    DcMotor ridicare2;
    DcMotor ridicare;

    Servo gheara1, gheara2;


    //28 * 20 / (2ppi * 4.125)
    Double width = 10.74; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;

    int pozitie = -1;


    DcMotor                 frontleft, frontright, backleft, backright;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .95, correction;
    boolean                 aButton, bButton;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        frontleft = hardwareMap.dcMotor.get("leftFront");
        frontright = hardwareMap.dcMotor.get("rightFront");

        backleft = hardwareMap.dcMotor.get("leftRear");
        backright = hardwareMap.dcMotor.get("rightRear");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //gheare cuva
        gheara1 = hardwareMap.get(Servo.class,"fdr");
        gheara2 = hardwareMap.get(Servo.class,"fst"); 

        absortie1 = hardwareMap.dcMotor.get("absortie1");
        absortie2 = hardwareMap.dcMotor.get("absortie2");

        ridicare = hardwareMap.dcMotor.get("ridicare");
        ridicare2 = hardwareMap.dcMotor.get("ridicare2");

        absortie2.setDirection(DcMotor.Direction.REVERSE);


        capstone = hardwareMap.get(Servo.class,"capstone");
        gheara1.setPosition(-.5);
        gheara2.setPosition(1);
        capstone.setPosition(-0.6);



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
      

        // wait for start button.

        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) 
        {
            telemetry.update();
        }

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);


        if (opModeIsActive())
        {
            if(mutat==0)
            {
                strafeToPosition(15.5, 0.4);
                mutat=1;
                //moveToPosition(4.2, 0.5);
                sleep(10);
            }
        }

        while (opModeIsActive())
        {
            // verific daca merg in linie dreapta » still working and thinking « 
            correction = checkDirection();

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();

            //Patratul();// » asta a fost primul test de calibrare
            //StoneRandom();
            //asta_e_cuva();
            //BlindFaraStricare();
            detectie();
            //aruncare_cub();


/*
 *
 *          _.-/`)                              .-.
 *         // / / )                           __| |__
 *      .=// / / / )                         [__   __]
 *     //`/ / / / /      __________             | |
 *    // /     ` /      /          \            | |
 *   ||         /       | WE PRAY  |            | |
 *    \\       /        | TO WORK  |            '-'
 *     ))    .'         \_________/
 *    //    /
 *         /
 *
*/



             /*
             *               __
             *              / _)  (GRRRRR)
             *       .-^^^-/ /          
             *   __/       /              
             *  <__.|_|-|_|  
             *
            */

           // break;
        }

    }


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

        /*
    Functia asta este simpla, merge doar in fata sau in spate
    pentru a deplasa robotul in spate "inches" va avea valoare negativa
     */
    public void moveToPosition(double inches, double speed){
        //
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
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
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

   
   
    public void detectie()
    {   

        if (tfod != null) 
        {
            
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) 
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.update();
            }
            oprire(200);
            
            if(updatedRecognitions.size()!=2)
            {
               /* if(pozitionat == 0)
                {
                    miscare_fata(50);
                    miscare_spate(50);
                    pozitionat = 1;
                } */
            }
            if(updatedRecognitions.size() == 4)
            {
                /*
                if(pozitionat == 0)
                {
                moveToPosition(4.2, 0.5);
                pozitionat = 1;
                } */
            }
           if (updatedRecognitions.size() == 3)
                        {
                            int SkystoneX = -1;
                            int Stone1X = -1;
                            int Stone2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                                    SkystoneX = (int) recognition.getLeft();
                                } else if (Stone1X == -1) {
                                    Stone1X = (int) recognition.getLeft();
                                } else {
                                    Stone2X = (int) recognition.getLeft();
                                }
                            }
                            if (SkystoneX != -1 && Stone1X != -1 && Stone2X != -1) {
                                if (SkystoneX < Stone1X && SkystoneX < Stone2X) {
                                    telemetry.addData("Skystone Position", "Left");
                                    pozitie = 1;
                                    path_stone();
                                } else if (SkystoneX > Stone1X && SkystoneX > Stone2X) {
                                    telemetry.addData("Skystone Position", "Right");
                                    pozitie = 3;
                                    path_stone();
                                } else {
                                    telemetry.addData("Skystone Position", "Center");
                                    pozitie = 2;
                                    path_stone();
                                }
                                telemetry.update();
                            }
                        }
            /**
            if(mutat==1)
            {
                path_stone();
            } */
            telemetry.update();
        }
    }

    public void path_stone()
    {

        if (tfod != null) {
            tfod.shutdown();
        }

        if(pozitie==1) 
        { //stanga
            telemetry.addData("Pozitia e:","stanga");
            telemetry.update();
            //sleep(5000);
            moveToPosition(-14.33, 1.0);
            rotate(-180, power);


            //strafeToPosition(-24.5, 1.0);
            rotate(-45, power);
            absortie1.setPower(-0.6);
            absortie2.setPower(-0.6);
            moveToPosition(-40, 0.8);

            sleep(500);

            
            absortie1.setPower(0);
            absortie2.setPower(0);
            moveToPosition(25, 0.8);
            rotate(50, power);
            strafeToPosition(-4,.8);
            moveToPosition(-65,0.8);
            absortie1.setPower(0.6);
            absortie2.setPower(0.6);
            oprire(500);
            
            moveToPosition(90,0.8);
            oprire(500000);


            
        }
        else if(pozitie==2)//centru
        {
            
            telemetry.addData("Pozitia e:","centru");
            telemetry.update();

            moveToPosition(22, 0.9);
            strafeToPosition(33, 0.7);

            absortie1.setPower(-0.4);
            absortie2.setPower(-0.4);

            moveToPosition(-14, 1);
            sleep(300);

            absortie1.setPower(0);
            absortie2.setPower(0);
            //
            strafeToPosition(-20, 0.9);
            moveToPosition(60, 0.9);
            rotate(-90, power);
            scuipare_cub();
            rotate(86, power);
            moveToPosition(-80, 0.9); // ma intorc dupa al 2 lea

            oprire(100);

            strafeToPosition(20, 0.7);

            absortie1.setPower(-0.4);
            absortie2.setPower(-0.4);

            moveToPosition(-15, 1);
            sleep(300);

            absortie1.setPower(0);
            absortie2.setPower(0);
            //
            strafeToPosition(-20, 0.9);
            moveToPosition(84, 0.9);
            rotate(-180,power);
            absortie1.setPower(0.8);
            absortie2.setPower(0.8);
            moveToPosition(10, 1);
            sleep(100);

            absortie1.setPower(0);
            absortie2.setPower(0);
            oprire(500000);


            //sleep(5000);
        }
        else if(pozitie==3)//dreapta
        {
            
            telemetry.addData("Pozitia e:","dreapta");
            telemetry.update();
            moveToPosition(13, 0.9);
            strafeToPosition(33, 0.6);

            absortie1.setPower(-0.4);
            absortie2.setPower(-0.4);

            moveToPosition(-12, 1);
            sleep(300);

            absortie1.setPower(0);
            absortie2.setPower(0);
            //
            strafeToPosition(-20, 0.6);
            
            moveToPosition(60, 1.0);


            rotate(-90, power);
            scuipare_cub();
            rotate(88, power);
            moveToPosition(-78, 0.9); // ma intorc dupa al 2 lea

            //moveToPosition(-10, 0.5);

            strafeToPosition(20, 0.8);

            absortie1.setPower(-0.4);
            absortie2.setPower(-0.4);

            moveToPosition(-12, 1);
            sleep(200);

            absortie1.setPower(0);
            absortie2.setPower(0);
            //
            strafeToPosition(-20, 1.0);
            moveToPosition(85, 1.0);
            rotate(-180, power);
            scuipare_cub();

            //rotate(-90,power);
            //moveToPosition(20,0.7);
            //aruncare_cub();
            moveToPosition(10, 0.5);
            oprire(500000);

            //sleep(5000);

        }
        else if(pozitie == -1)
        {
            oprire(20000);
            telemetry.addData("Pozitia e:","default");
            telemetry.update();
        }
        //oprire(2000000);
        //BlindFaraStricare();
        //teste();
    }

    public void teste()
    {
        //rotate(-180, power);

        strafeToPosition(-24.5, 1.0);
        //


        absortie1.setPower(-0.5);
        absortie2.setPower(-0.5);

        moveToPosition(-15, 1);

        sleep(300);

        absortie1.setPower(0);
        absortie2.setPower(0);
    }

    public void BlindFaraStricare()
    {
        //strafeToPosition(24.5, 1.0);

        rotate(-180, power);

        strafeToPosition(-24.5, 1.0);
        //


        absortie1.setPower(-0.5);
        absortie2.setPower(-0.5);

        moveToPosition(-15, 1);

        sleep(300);

        absortie1.setPower(0);
        absortie2.setPower(0);
        //
        strafeToPosition(32, 1.0);
        //
        moveToPosition(57, 1);
        rotate(90, power);
        
        //
        strafeToPosition(25, 1);


        moveToPosition(25, 0.5);

        aruncare_cub();

        mutare_placa();

        strafeToPosition(9, 1.0);


        //aici sunt miscarile de agatare
        //
        sleep(950);
        //
    }

    public void aruncare_cub()
    {


        ridicare.setPower(0.7);  
        ridicare2.setPower(-0.7);

        sleep(950);

        ridicare.setPower(0);
        ridicare.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ridicare2.setPower(0);
        ridicare2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        absortie1.setPower(-1);
        absortie2.setPower(-1);
        sleep(1500);
        absortie1.setPower(0);
        absortie2.setPower(0);
        ridicare.setPower(-0.4);
        ridicare2.setPower(0.4);
        sleep(750);
        ridicare.setPower(0);
        ridicare2.setPower(0);
        
    }

    public void scuipare_cub()
    {
        absortie1.setPower(1);
        absortie2.setPower(1);
        sleep(1000);
        absortie1.setPower(0);
        absortie2.setPower(0);
        
    }

    public void asta_e_cuva()
    {
        strafeToPosition(49, 1.0);
        //


        absortie1.setPower(-0.5);
        absortie2.setPower(-0.5);

        moveToPosition(-15, 1);

        sleep(300);

        absortie1.setPower(0);
        absortie2.setPower(0);
        //
        strafeToPosition(-25, 1.0);
        //
        moveToPosition(-37, 1);
        rotate(-90, power);
        absortie1.setPower(0.5);
        absortie2.setPower(0.5);
        sleep(950);
        absortie1.setPower(0);
        absortie2.setPower(0);
        //
        strafeToPosition(25, 1);


        moveToPosition(25, 1);
        mutare_placa();
        //
        strafeToPosition(9, 1.0);
        //
        moveToPosition(-48.2, 1);
        //
    }

    public void mutare_placa()
    {
        gheara1.setPosition(1);
        gheara2.setPosition(0);
        //
        sleep(950);
        moveToPosition(-30, 1);
        rotate(-90, power);
        sleep(950);
        gheara1.setPosition(-.5);
        gheara2.setPosition(1);
    }
    
    public void mutare_placa_orizontal()
    {
        gheara1.setPosition(1);
        gheara2.setPosition(0);
        sleep(1000);
        moveToPosition(-40,0.8);
        rotate(90,power);
        moveToPosition(5,0.8);
        gheara1.setPosition(-.5);
        gheara2.setPosition(1);
        sleep(200);
        
        
        
    }

    public void Patratul()
    {
        strafeToPosition(24.6, 0.7);
        //
        moveToPosition(-34.4, 1);
        //


        rotate(-90, power);
        //
        moveToPosition(-24, 1);
        //

        rotate(-90, power);
        //
        moveToPosition(-34.4, 1);
        rotate(-180, power);
    }

    // acelasi principiu ca la move to position, pentru a te duce in stanga inch va avea valoare pozitiva iar dreapta negativa
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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = stanga, - = dreapta.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        double  leftPowerLent, rightPowerLent;

        double degreesrapid;

        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;

            leftPowerLent = -0.15;
            rightPowerLent = 0.15;
            degreesrapid = degrees +30;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;

            leftPowerLent = 0.15;
            rightPowerLent = -0.15;
            degreesrapid = degrees -30;
        }
        else return;

        // set power to rotate.
        frontleft.setPower(leftPower);
        frontright.setPower(rightPower);
        backleft.setPower(leftPower);
        backright.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) 
            {
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 getangle", getAngle());
                telemetry.addData("4 degrees",  degrees);
                telemetry.addData("5 degrees - 30",  degreesrapid);
                telemetry.update();
                if(getAngle() > degreesrapid)
                {
                    frontleft.setPower(leftPower);
                    frontright.setPower(rightPower);
                    backleft.setPower(leftPower);
                    backright.setPower(rightPower);
                }
                else if(getAngle() > degrees)
                {
                    frontleft.setPower(leftPowerLent);
                    frontright.setPower(rightPowerLent);
                    backleft.setPower(leftPowerLent);
                    backright.setPower(rightPowerLent);
                }
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) 
            {
                
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 getangle", getAngle());
                telemetry.addData("4 degrees",  degrees);
                telemetry.addData("5 degrees - 30",  degreesrapid);
                telemetry.update();

                if(getAngle() < degreesrapid)
                {
                    frontleft.setPower(leftPower);
                    frontright.setPower(rightPower);
                    backleft.setPower(leftPower);
                    backright.setPower(rightPower);
                }
                else if(getAngle() < degrees)
                {
                    frontleft.setPower(leftPowerLent);
                    frontright.setPower(rightPowerLent);
                    backleft.setPower(leftPowerLent);
                    backright.setPower(rightPowerLent);
                }
                
            }

        // turn the motors off.
        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void oprire(long milisecunde) {
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        sleep(milisecunde);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.4;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
