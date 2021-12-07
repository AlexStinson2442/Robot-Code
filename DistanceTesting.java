/*
 * Green Machine 15321 Gyro Drive Code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 */

@Autonomous(name="GM: Distance Testing", group="Green Machine")
//@Disabled
public class DistanceTesting extends LinearOpMode {
        private Rev2mDistanceSensor sensorRange;


        /* Declare OpMode members. */
    HardwareGreenMachine robot = new HardwareGreenMachine();   // Use a hardware
    GreenMachineSupportClass gm = new GreenMachineSupportClass(this, robot);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.
    static final double STRAFE_SPEED = 0.8;



    // Variables
    int attempts = 0;
    double dsdred = gm.sensorRedDistance.getDistance(DistanceUnit.INCH);
    double dsdblue = gm.sensorBlueDistance.

    double randomvariable = gm.

    @Override
    public void runOpMode() throws InterruptedException {

        // you can use this as a regular DistanceSensor.
        sensorRange = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_red_distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;


        double dsd = 0;
        int distanceCounter = 3;
        double initAngle = 0;
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        gm.motorReset();

        // Set motors to use encoders
        gm.motorWEncoder();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            initAngle = robot.getAngle();
            telemetry.addData("IMU Calib Status", robot.imu.getCalibrationStatus().toString());
            telemetry.addData("Robot Heading: ", "%5.2f", initAngle);
            telemetry.update();
        }

        // Set the hardware for autonomous drive
        robot.autonomousDrive();

//Autonomous Movements------------------------------------------------------------------------------

        //gm.positionDrive(DRIVE_SPEED,27);
        //gm.positionTurn(TURN_SPEED, -92.024659);
        //gm.positionDrive(DRIVE_SPEED, 27);


     //   gm.positionDrive(DRIVE_SPEED, 7);
     //   gm.positionTurn(TURN_SPEED, 117);
     //   gm.wait(1);

        gm.positionDrive(DRIVE_SPEED, 7);
        gm.positionTurn(TURN_SPEED, 117);


        if (dsdred <= 8.0) {
            distanceCounter = 1;
            gm.positionDrive(DRIVE_SPEED, 20);
            gm.positionTurn(TURN_SPEED, -90);
        }
        else{
            gm.positionDrive(DRIVE_SPEED, 8.5);
            dsdred = sensorRange.getDistance(DistanceUnit.INCH);

            if (dsdred <= 8) {
                distanceCounter = 2;
            }
            gm.positionDrive(DRIVE_SPEED, 12);
            gm.positionTurn(TURN_SPEED, -90);
        }


        switch (distanceCounter){
            case 1: gm.positionTurn(TURN_SPEED,90);
                break;
            case 2: gm.positionTurn(TURN_SPEED, -305);
                break;
            case 3: //raise arm for upper level
                break;
            default: //figure out later
                break;


       /* gm.positionDrive(DRIVE_SPEED, 38);
        gm.wait(1);
        gm.positionTurn(TURN_SPEED, -116);
        gm.wait(1);
        gm.positionDrive(DRIVE_SPEED, 5);*/

  //-----wrtten things

        //gm.gyroTurn(TURN_SPEED, 90);
        /*dsd = sensorRange.getDistance(DistanceUnit.INCH);
        if (dsd <= 8) {
            distanceCounter = 1;
            gm.positionDrive(DRIVE_SPEED, 20);
            gm.positionTurn(TURN_SPEED, -90);
        }
        else{
                gm.positionDrive(DRIVE_SPEED, 8.5);
                dsd = sensorRange.getDistance(DistanceUnit.INCH);

            if (dsd <= 8) {
                distanceCounter = 2;
            }
                gm.positionDrive(DRIVE_SPEED, 12);
                gm.positionTurn(TURN_SPEED, -90);
            }

        switch (distanceCounter){
            case 1: //raise arm for bottom level
                break;
            case 2: //raise arm for middle level
                // drive forwards short distance
                // release block
                //back up same distance traveled forward
                break;
            case 3: //raise arm for upper level
                break;
            default: //figure out later
                break;
        }

        gm.positionDrive(DRIVE_SPEED, -12);
        gm.positionTurn(TURN_SPEED, 90);
        gm.positionDrive(DRIVE_SPEED, -60);
        //spin carousel clockwise

        gm.positionDrive(DRIVE_SPEED, 2);
        gm.positionTurn(TURN_SPEED, -20);
        gm.positionDrive(DRIVE_SPEED, 60);
        gm.positionTurn(TURN_SPEED, 20);
        gm.positionDrive(DRIVE_SPEED, 20);



        /*Forward 2 inches












         */
        // stop action code

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}
