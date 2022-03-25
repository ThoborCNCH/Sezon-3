/** Copyright (c) 2020 FIRST. All rights reserved.
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
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TFOD_WEBCAM_HANGA", group = "HANGA")
//@Disabled
public class TFOD_WEBCAM_HANGA extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
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
    private DcMotor RF, LF, LR, RR;
    public ElapsedTime mRunTime = new ElapsedTime();
    Servo fdr, fst;
    DcMotor absortie1;
    DcMotor absortie2;
    DcMotor ridicare2;
    Servo gheara;
    Servo gheara1, gheara2;
    Servo capstone;
    DcMotor ridicare;
    CRServo extindere = null;

    int mutat = 0;

    double diametru =64;
    double raza= diametru / 2;
    int pozitie=-1;
    int pozitionat = 0;

    @Override
    public void runOpMode() 
    {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initializare_hard();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }


        /** Wait for the game to begin */
                //waitForStart();
        while (!opModeIsActive() && !isStopRequested()) 
        {
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
        }


        if (opModeIsActive())
        {
            if(mutat==0)
            {
                miscare_dreapta(1400); 
                mutat=1;
                sleep(10);
            }
        }
        while (opModeIsActive())
        {
            detectie();
            telemetry.update();
        }

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
            oprire(2000);
            
            if(updatedRecognitions.size()!=2)
            {
               /* if(pozitionat == 0)
                {
                    miscare_fata(50);
                    miscare_spate(50);
                    pozitionat = 1;
                } */
            }
            if(updatedRecognitions.size()==2)
            {
                int SkystoneX =-1;
                int StoneX = -1;
                int SkystoneY = -1;
                int StoneY = -1;
                
                for (Recognition recognition : updatedRecognitions) 
                {
                    if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT))
                    {
                        SkystoneX = (int) recognition.getLeft();
                        SkystoneY = (int) recognition.getRight();
                    }
                    else  if(StoneX == -1) 
                    {   StoneY = (int) recognition.getRight();
                        StoneX = (int) recognition.getLeft();
                    }   

                    if(SkystoneX!=-1 && StoneX !=-1 && SkystoneY !=-1 && StoneY !=-1)
                    {
                        if(SkystoneX < StoneX)
                        {   telemetry.addData("Coordonate SkystoneX", SkystoneX );
                                telemetry.addData("Pozitia Skystone este:", "stanga");
                                telemetry.update();
                                pozitie=1;
                                path_stone();
                                
                                
                        }
                        else if(SkystoneX > StoneX)
                        {   
                            telemetry.addData("Coordonate SkystoneX", SkystoneX );
                            telemetry.addData("Pozitia Skystone este: ", " centru");
                            telemetry.update();
                            pozitie=2;
                            path_stone();
                            
                        }
                    }
                            
                    if(updatedRecognitions.size()==2 && SkystoneX == -1)
                    {
                        telemetry.addData("Pozitie Skystone este:"," dreapta" );
                        telemetry.update(); 
                        pozitie=3;
                        path_stone();
                        
                    }
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
        if(pozitie==1) 
        { //stanga
            telemetry.addData("Pozitia e:","stanga");
            telemetry.update();
            sleep(5000);
        }
        else if(pozitie==2)//centru
        {
            
            telemetry.addData("Pozitia e:","centru");
            telemetry.update();
            sleep(5000);
        }
        else if(pozitie==3)//dreapta
        {
            
            telemetry.addData("Pozitia e:","dreapta");
            telemetry.update();
            sleep(5000);

        }
        else if(pozitie == -1)
        {
            oprire(20000);
            telemetry.addData("Pozitia e:","default");
            telemetry.update();
        }
        oprire(2000000);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void initializare_hard() {

        //motoare roti
        LF = hardwareMap.dcMotor.get("leftFront");
        RF = hardwareMap.dcMotor.get("rightFront");
        RR = hardwareMap.dcMotor.get("rightRear");
        LR = hardwareMap.dcMotor.get("leftRear");
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LR.setDirection(DcMotor.Direction.FORWARD);
        RR.setDirection(DcMotor.Direction.REVERSE);
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
        
        /*

        bratdreapta = hardwareMap.get(DcMotor.class,"brat1");
        bratstanga = hardwareMap.get(DcMotor.class,"brat2");  */



    }

    public void deplasare(double lm1, double lm2, double rm1, double rm2) {
        
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        LF.setPower(lm1);
        RF.setPower(lm2);
        LR.setPower(rm1);
        RR.setPower(rm2);
    }

    public void run_without_encoders()
    {LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        deplasare(0,0,0,0);
    }
    public void oprire_encoder()
    {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RR.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        //deplasare(0,0,0,0) ;
        // Set power to 0
    }

    public void deplasare_encoder(int lm1, int lm2, int rm1, int rm2) 
    {
        oprire_encoder();
        
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RR.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        LF.setTargetPosition(lm1);
        RF.setTargetPosition(lm2);
        LR.setTargetPosition(rm1);
        RR.setTargetPosition(rm2);
    }

   public void miscare_fata(long milisecunde) {
        deplasare(0.5, 0.5, 0.5, 0.5);
        sleep(milisecunde);
    }

    public void miscare_dreapta(long milisecunde) {
        deplasare(0.5,-0.5,-0.5,0.5);
        sleep(milisecunde);
    }

    public void miscare_stanga(long milisecunde) {
        deplasare(-0.5,+0.5,+0.5,-0.5);
        sleep(milisecunde);
    }

 

    public void miscare_spate(long milisecunde) {
        deplasare(-0.5,-0.5,-0.5,-0.5);
        sleep(milisecunde);
    }
    public void oprire(long milisecunde) {
        deplasare(0,0,0,0) ;
        sleep(milisecunde);
    }
    public void ridicare_cuva ()
    { gheara1.setPosition(-.5);
        gheara2.setPosition(1);
    }
    public void apucare_cuva()
    {gheara1.setPosition(1);
     gheara2.setPosition(0); }
     
     
      
    

    void extindere(long milisecunde)
    {
        extindere.setPower(-1);
        sleep(milisecunde);
        extindere.setPower(0);
    }
    

    public void rotire_poz_dr_neg_st(double alfa)
    {   
        oprire_encoder();
        
        double  distanta =( 2 * 3.14 * raza * (alfa / 360.)) / 2.54;   
        
        double circumference = 3.14 * 3.937;
        double rotationNeeded = distanta/circumference;
        int encoderDrivingTarget = (int)(rotationNeeded * 1120);
        
        deplasare_encoder(encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget, encoderDrivingTarget);
        
        LF.setPower(1);
        RF.setPower(1);
        LR.setPower(1);
        RR.setPower(1);
        
         while(LF.isBusy()||RF.isBusy()||LR.isBusy()||RR.isBusy())
        { 
            telemetry.addData("Parcurcand distanta",distanta);
            telemetry.addData("[*]Distanta Calcul", "Running to %7d :%7d", encoderDrivingTarget, encoderDrivingTarget);
            telemetry.addData("[**]Distanta spate", "Running at %7d :%7d", LR.getCurrentPosition(), RR.getCurrentPosition());
            telemetry.addData("[***]Distanta fata", "Running at %7d :%7d", LF.getCurrentPosition(), RF.getCurrentPosition());
             
            telemetry.update();
            
        }
        //oprire_encoder();
        

    }


    public void stop_move() 
    {
        deplasare(0,0,0,0) ;

    }

}
