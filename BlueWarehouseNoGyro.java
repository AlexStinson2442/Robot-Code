/*
 * Green Machine 15321 Gyro Drive Code
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 */

@Autonomous(name="GM: Blue Warehouse", group="Green Machine")
//@Disabled
public class BlueWarehouseNoGyro extends LinearOpMode {

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

    @Override
    public void runOpMode() {

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

        gm.positionDrive(DRIVE_SPEED,17);
//        gm.gyroDrive(DRIVE_SPEED, 17, 0);
        gm.positionTurn(TURN_SPEED, -92.024659);
        gm.positionDrive(DRIVE_SPEED, 74);


        // stop action code

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
