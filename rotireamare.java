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

import java.util.ArrayList;
import java.util.List;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;


@Autonomous(name="Cod Hanga Teste", group="Hanga")
//@Disabled
public class rotireamare extends LinearOpMode
{

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


    DcMotor                 frontleft, frontright, backleft, backright;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .55, correction;
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
            BlindFaraStricare();
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

            break;
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

    public void StoneRandom()
    {
        // miscare stanga 22.8 inch
        strafeToPosition(22.8, 0.8);
        // miscare fata 11.4 inch
        moveToPosition(16, 0.8);
        // rotire la 45grd la stanga
        //turnWithGyro(45, -0.8);
        rotate(45, power);

        absortie1.setPower(-0.5);
        absortie2.setPower(-0.5);

        //miscare spate 22.4 inch
        moveToPosition(-28, 0.8);
        
        sleep(1000);

        absortie1.setPower(0);
        absortie2.setPower(0);


        // rotire 90grd la dreapta
        //turnWithGyro(90, 0.8);
        rotate(-90, power);
        // miscare spate 22.2 inch
        moveToPosition(-28, 0.8);
        // miscare 45 grade la stanga
        //turnWithGyro(45, -0.8);
        rotate(45, power);
        // miscare spate 34.2 inch
        moveToPosition(-34.2, 0.8);

        absortie1.setPower(0.8);
        absortie2.setPower(0.8);
        sleep(1000);
        absortie1.setPower(0);
        absortie2.setPower(0);

        // miscare fata 22 inch » parcare
        moveToPosition(22, 0.8);
    }


    public void BlindFaraStricare()
    {
        strafeToPosition(24.5, 1.0);

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

        frontleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        backleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        frontright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        backright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

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
            degreesrapid = degrees +20;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;

            leftPowerLent = 0.15;
            rightPowerLent = -0.15;
            degreesrapid = degrees -20;
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
                telemetry.addData("5 degrees - 20",  degreesrapid);
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
                telemetry.addData("5 degrees - 20",  degreesrapid);
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
}
