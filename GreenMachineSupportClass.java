package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;


public class GreenMachineSupportClass {

    /* Declare OpMode members. */
    static final double     COUNTS_PER_MOTOR_REV        = 1120 ;    // eg: AndyMark 40 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION        = 0.44  ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES       = 4.0 ;     // For figuring circumference
    static final double     STRAFE_MODIFIER_PER_INCH    = 1.2;     // Multiplier for strafe per inch
    static final double     COUNTS_PER_INCH             = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                        (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     WHEEL_BASE_WIDTH            = 11.375;//In Inch
    static final double     COUNTS_PER_DEG            = ((2*3.1415 * (WHEEL_BASE_WIDTH))/360)*COUNTS_PER_INCH;

    // These constants define the desired driving/control characteristics
    static final double     HEADING_THRESHOLD               = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF                    = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF                   = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_STRAFE_COEFF                  = 0.5 ;    // Larger is more responsive, but also less stable
    static final double     ENCODER_POSITION_TOLERANCE      = 20;
    static final double     ENCODER_STRAFE_POSITION_TOLERANCE   = 110;

    // Moving average size for correcting heading error
    static final int        AVERAGE_SIZE            = 5;


    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------
    // State used for updating telemetry
    LinearOpMode opModObj;
    HardwareGreenMachine robot;


    /* Constructor */
    public GreenMachineSupportClass(LinearOpMode opmode, HardwareGreenMachine hw){
        opModObj = opmode;
        robot = hw;
    }

    /**
     *  Method to stop and reset encoders
     */
    public void motorReset (){
        robot.leftBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     *  Method to enable encoders
     */
    public void motorWEncoder (){
        robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     *  Method to set motors to RUN TO POSITION
     */
    public void motorRunPosition (){
        robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     *  Method to disable encoders
     */
    public void motorWOEncoder (){
        robot.leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Method to strafe on power.
     *  Move will be based on power. POSITIVE - Strafe right; NEGATIVE - Strafe left.
     *
     * @param speed      Target speed should be within +/- 1.
     */
    public void moveStrafe (double speed){
        robot.leftBackWheel.setPower(speed);
        robot.leftFrontWheel.setPower(-speed);
        robot.rightBackWheel.setPower(-speed);
        robot.rightFrontWheel.setPower(speed);
    }

    /**
     *  Method to drive based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move right from current position.  Negative distance means move left.
     */
    public void positionStrafe ( double speed,
                                 double distance) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        boolean left = false;
        boolean right = false;

        // Ensure that the opmode is still active
        if (opModObj.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH * STRAFE_MODIFIER_PER_INCH);
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + moveCounts; //mecanum wheels opposite direction of front
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + moveCounts;//mecanum wheels opposite direction of back
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            motorRunPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontWheel.setPower(speed);
            robot.leftBackWheel.setPower(speed);
            robot.rightFrontWheel.setPower(speed);
            robot.rightBackWheel.setPower(speed);

            if(distance < 0) // direction will be negative for left movement
                left = true;
            else
                right = true;

            // keep looping while we are still active, and ALL motors are running.
            while (opModObj.opModeIsActive() &&
//                    (left && (robot.leftFrontWheel.isBusy() && robot.leftBackWheel.isBusy())) ||
//                    (right && (robot.rightFrontWheel.isBusy() && robot.rightBackWheel.isBusy()))) {
//                    (left && robot.leftFrontWheel.isBusy()) ||
//                    (right && robot.rightFrontWheel.isBusy())) {
                    (left && (Math.abs(robot.leftFrontWheel.getCurrentPosition()
                            - robot.leftFrontWheel.getTargetPosition()) > ENCODER_STRAFE_POSITION_TOLERANCE)) ||
                    (right && (Math.abs(robot.rightFrontWheel.getCurrentPosition()
                            - robot.rightFrontWheel.getTargetPosition()) > ENCODER_STRAFE_POSITION_TOLERANCE))){

                // Display drive status for the driver.
//                opModObj.telemetry.addData("Target F L/R",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
//                opModObj.telemetry.addData("Actual F L/R",  "%7d:%7d",      robot.leftFrontWheel.getCurrentPosition(),
//                        robot.rightFrontWheel.getCurrentPosition());
//                opModObj.telemetry.addData("Speed ", "%5.2f",  speed);
//                opModObj.telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontWheel.setPower(0);
            robot.leftBackWheel.setPower(0);
            robot.rightFrontWheel.setPower(0);
            robot.rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            motorWEncoder();
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move right from current position.  Negative distance means move left.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroStrafe ( double speed,
                             double distance,
                             double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        int     size = 0;
        double  max;
        double  error;
        double  steer;
        double  frontSpeed;
        double  backSpeed;
        boolean left = false;
        boolean right = false;

//        ArrayList<Double> values = new ArrayList<Double>();

        // Ensure that the opmode is still active
        if (opModObj.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH * STRAFE_MODIFIER_PER_INCH);
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + moveCounts; //mecanum wheels opposite direction of front
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + moveCounts;//mecanum wheels opposite direction of back
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            motorRunPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontWheel.setPower(speed);
            robot.leftBackWheel.setPower(speed);
            robot.rightFrontWheel.setPower(speed);
            robot.rightBackWheel.setPower(speed);

            if(distance < 0) // direction will be negative
                left = true;
            else
                right = true;

            // keep looping while we are still active, and ALL motors are running.
            while (opModObj.opModeIsActive() &&
//                    (left && (robot.leftFrontWheel.isBusy() && robot.leftBackWheel.isBusy())) ||
//                    (right && (robot.rightFrontWheel.isBusy() && robot.rightBackWheel.isBusy()))) {
//                    (left && robot.leftFrontWheel.isBusy()) ||
//                    (right && robot.rightFrontWheel.isBusy())) {
                    (left && (Math.abs(robot.leftFrontWheel.getCurrentPosition()
                            - robot.leftFrontWheel.getTargetPosition()) > ENCODER_STRAFE_POSITION_TOLERANCE)) ||
                    (right && (Math.abs(robot.rightFrontWheel.getCurrentPosition()
                            - robot.rightFrontWheel.getTargetPosition()) > ENCODER_STRAFE_POSITION_TOLERANCE))){

                //Adjust relative speed based on heading error
                error = getError(angle);

                // Get the corrective steer value
                steer = getSteer(error, P_STRAFE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontSpeed = speed + steer;
                backSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
                if (max > 1.0)
                {
                    frontSpeed /= max;
                    backSpeed /= max;
                }

/*
                if(Math.abs(frontSpeed) < (speed*0.8))
                    frontSpeed = speed * 0.8;
                if(Math.abs(backSpeed) < (speed*0.8))
                    backSpeed = speed * 0.8;
 */

                //Set the motor power to steer to the correct angle
                robot.leftFrontWheel.setPower(frontSpeed);
                robot.leftBackWheel.setPower(backSpeed);
                robot.rightFrontWheel.setPower(frontSpeed);
                robot.rightBackWheel.setPower(backSpeed);

/*                // Display drive status for the driver.
                opModObj.telemetry.addData("Err/Steer ",  "%5.1f/%5.1f",  errorCorrection, steer);
                opModObj.telemetry.addData("Target F L/R",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                opModObj.telemetry.addData("Actual F L/R",  "%7d:%7d",      robot.leftFrontWheel.getCurrentPosition(),
                        robot.rightFrontWheel.getCurrentPosition());
                opModObj.telemetry.addData("Speed F/B ",   "%5.2f:%5.2f",  frontSpeed, backSpeed);
                opModObj.telemetry.update();
*/
            }

            // Stop all motion;
            robot.leftFrontWheel.setPower(0);
            robot.leftBackWheel.setPower(0);
            robot.rightFrontWheel.setPower(0);
            robot.rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            motorWEncoder();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHoldStrafe(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModObj.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeadingStrafe(speed, angle, P_TURN_COEFF);
            opModObj.telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeadingStrafe(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double frontSpeed;
        double backSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            frontSpeed  = 0.0;
            backSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            backSpeed  = speed * steer;
            frontSpeed   = -backSpeed;
        }

        //Set the motor power to steer to the correct angle
        robot.leftFrontWheel.setPower(frontSpeed);
        robot.leftBackWheel.setPower(-backSpeed);
        robot.rightFrontWheel.setPower(-frontSpeed);
        robot.rightBackWheel.setPower(backSpeed);

        // Display it for the driver.
        //opModObj.telemetry.addData("Target Deg", "%5.2f", angle);
        //opModObj.telemetry.addData("Err/St    ", "%5.2f/%5.2f", error, steer);
        //opModObj.telemetry.addData("Speed F/B ", "%5.2f:%5.2f", frontSpeed, backSpeed);

        return onTarget;
    }

    /**
     *  Method to turn based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.
     * @param angle      Angle (in deg) to move from current position.  Negative distance means move LEFT .
     *
     */
    public void positionTurn (double speed, double angle){
        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;

        // Ensure that the opmode is still active
        if (opModObj.opModeIsActive()) {
            //distance = -1*distance;

            // Determine new target position, and pass to motor controller
            //moveCounts = (int)(distance * COUNTS_PER_INCH);
            moveCounts = (int)(angle * COUNTS_PER_DEG);
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() - moveCounts;
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() - moveCounts;
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            motorRunPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontWheel.setPower(speed);
            robot.leftBackWheel.setPower(speed);
            robot.rightFrontWheel.setPower(speed);
            robot.rightBackWheel.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModObj.opModeIsActive() &&
//                    (robot.leftFrontWheel.isBusy()
//                            && robot.leftBackWheel.isBusy()
//                            && robot.rightFrontWheel.isBusy()
//                            && robot.rightBackWheel.isBusy())) {
                    ((Math.abs(robot.leftFrontWheel.getCurrentPosition() - robot.leftFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE)
                            && (Math.abs(robot.rightFrontWheel.getCurrentPosition() - robot.leftFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE))){

                // Display drive status for the driver.
                //opModObj.telemetry.addData("Target L/R",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                //opModObj.telemetry.addData("Actual L/R",  "%7d:%7d",      robot.leftFrontWheel.getCurrentPosition(),
                //        robot.rightFrontWheel.getCurrentPosition());
                //opModObj.telemetry.addData("Speed L/R ",   "%5.2f:%5.2f",  speed, speed);
                //opModObj.telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontWheel.setPower(0);
            robot.leftBackWheel.setPower(0);
            robot.rightFrontWheel.setPower(0);
            robot.rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            motorWEncoder();
        }
    }

    /**
     *  Method to drive on power.
     *  Move will be based on power. POSITIVE - forward; NEGATIVE - backward.
     *
     * @param speed      Target speed should be within +/- 1.
     */
    public void moveDrive (double speed){
        //Set all 4 Wheel Powers
        robot.leftBackWheel.setPower(-speed);
        robot.leftFrontWheel.setPower(-speed);
        robot.rightBackWheel.setPower(-speed);
        robot.rightFrontWheel.setPower(-speed);
    }

    /**
     *  Method to drive based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     */
    public void positionDrive ( double speed,
                                double distance) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;

        // Ensure that the opmode is still active
        if (opModObj.opModeIsActive()) {
            distance = -1*distance;

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            motorRunPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontWheel.setPower(speed);
            robot.leftBackWheel.setPower(speed);
            robot.rightFrontWheel.setPower(speed);
            robot.rightBackWheel.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModObj.opModeIsActive() &&
//                    (robot.leftFrontWheel.isBusy()
//                           && robot.leftBackWheel.isBusy()
//                            && robot.rightFrontWheel.isBusy()
//                            && robot.rightBackWheel.isBusy())) {
                    ((Math.abs(robot.leftFrontWheel.getCurrentPosition() - robot.leftFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE)
                            && (Math.abs(robot.rightFrontWheel.getCurrentPosition() - robot.rightFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE))){

                // Display drive status for the driver.
                //opModObj.telemetry.addData("Target L/R",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                //opModObj.telemetry.addData("Actual L/R",  "%7d:%7d",      robot.leftFrontWheel.getCurrentPosition(),
                //        robot.rightFrontWheel.getCurrentPosition());
                //opModObj.telemetry.addData("Speed L/R ",   "%5.2f:%5.2f",  speed, speed);
                //opModObj.telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontWheel.setPower(0);
            robot.leftBackWheel.setPower(0);
            robot.rightFrontWheel.setPower(0);
            robot.rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            motorWEncoder();
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftFrontTarget;
        int     newLeftBackTarget;
        int     newRightFrontTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  desiredAngle;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModObj.opModeIsActive()) {
            distance = -1*distance;

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontWheel.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackWheel.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontWheel.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontWheel.setTargetPosition(newLeftFrontTarget);
            robot.leftBackWheel.setTargetPosition(newLeftBackTarget);
            robot.rightFrontWheel.setTargetPosition(newRightFrontTarget);
            robot.rightBackWheel.setTargetPosition(newRightBackTarget);

            motorRunPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontWheel.setPower(speed);
            robot.leftBackWheel.setPower(speed);
            robot.rightFrontWheel.setPower(speed);
            robot.rightBackWheel.setPower(speed);


            // keep looping while we are still active, and ALL motors are running.
            while (opModObj.opModeIsActive() &&
//                    (robot.leftFrontWheel.isBusy()
//                            && robot.leftBackWheel.isBusy()
//                            && robot.rightFrontWheel.isBusy()
//                            && robot.rightBackWheel.isBusy())) {
                    ((Math.abs(robot.leftFrontWheel.getCurrentPosition() - robot.leftFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE)
                            && (Math.abs(robot.rightFrontWheel.getCurrentPosition() - robot.rightFrontWheel.getTargetPosition())
                            > ENCODER_POSITION_TOLERANCE))){

                //Adjust relative speed based on heading error
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                //Set the motor power to steer to the correct angle
                robot.leftFrontWheel.setPower(leftSpeed);
                robot.leftBackWheel.setPower(leftSpeed);
                robot.rightFrontWheel.setPower(rightSpeed);
                robot.rightBackWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                //opModObj.telemetry.addData("Err/Steer ",  "%5.1f/%5.1f",  error, steer);
                //opModObj.telemetry.addData("Target L/R",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                //opModObj.telemetry.addData("Actual L/R",  "%7d:%7d",      robot.leftFrontWheel.getCurrentPosition(),
                //        robot.rightFrontWheel.getCurrentPosition());
                //opModObj.telemetry.addData("Speed L/R ",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                //opModObj.telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontWheel.setPower(0);
            robot.leftBackWheel.setPower(0);
            robot.rightFrontWheel.setPower(0);
            robot.rightBackWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            motorWEncoder();
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModObj.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opModObj.telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModObj.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            opModObj.telemetry.update();

        }

        // Stop all motion;
        robot.leftFrontWheel.setPower(0);
        robot.leftBackWheel.setPower(0);
        robot.rightFrontWheel.setPower(0);
        robot.rightBackWheel.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double      error ;
        double      steer ;
        boolean     onTarget = false ;
        double      leftSpeed;
        double      rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        //Set the motor power to steer to the correct angle
        robot.leftFrontWheel.setPower(leftSpeed);
        robot.leftBackWheel.setPower(leftSpeed);
        robot.rightFrontWheel.setPower(rightSpeed);
        robot.rightBackWheel.setPower(rightSpeed);

        // Display it for the driver.
        //opModObj.telemetry.addData("Target Deg", "%5.2f", angle);
        //opModObj.telemetry.addData("Err/St    ", "%5.2f/%5.2f", error, steer);
        //opModObj.telemetry.addData("Speed L/R ", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn RIGHT (CCW) to reduce error.
     *          -ve errors means the robot should turn LEFT (CW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        robotError = targetAngle - robot.getAngle();

        if(robotError > 180){
            robotError -= 360;
        } else if(robotError <= -180){
            robotError += 360;
        }

        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return steer correction value
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip((error/180) * PCoeff, -1, 1);

        //method to get distance

        redSensorDistance = gm.sensorRedDistance.getDistance(DistanceUnit.INCH);

    }
}

