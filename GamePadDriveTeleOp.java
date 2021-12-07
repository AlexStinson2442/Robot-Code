package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "GM: GamePad Driver", group = "Green Machine")
//@Disabled
public class GamePadDriveTeleOp extends OpMode {

//Definitions---------------------------------------------------------------------------------------

    HardwareGreenMachine robot = new HardwareGreenMachine();   // Use GreenMachine's hardware

    final double ROTATE                 = 1;
    final double MOTOR_POWER_ZERO       = 0;
    final double DRIVE_MOTOR_LOW_POWER  = 0.7;
    final double DRIVE_MOTOR_HIGH_POWER = 1;

    final double CAROUSEL_MOTOR_POWER = 0.978765;

    //static final double ARM_ROTATE_MIN_POSITION = 0;
    //static final double ARM_ROTATE_MAX_POSITION = 1000;
    static double ARM_ROTATE_POWER = 0.9762;



    //Drive
    double leftFrontWheelPower      = 0;
    double leftBackWheelPower       = 0;
    double rightFrontWheelPower     = 0;
    double rightBackWheelPower      = 0;
    double forward                  = 0;
    double strafe                   = 0;
    double clockWise                = 0;
    double maxPower                 = 0;
    double motorPowerConstant       = DRIVE_MOTOR_LOW_POWER;
    double armRotateGamepadTrigger  = 0;
    int armRotatePosition           = 0;
//Init & Loop---------------------------------------------------------------------------------------

    @Override
    public void init() {
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        telemetry.addData(">", "Initializing Robot Hardware");
        telemetry.update();
        robot.init(hardwareMap);
        telemetry.addData(">", "Robot Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Set the hardware for TeleOp drive
        //robot.teleOpDrive();
//Drive Wheel Controls & Math-----------------------------------------------------------------------
        forward = gamepad1.left_stick_y + (gamepad2.left_stick_y/2);
        clockWise = (-gamepad1.left_stick_x + (-gamepad2.left_stick_x/2)) * ROTATE;
        strafe = -gamepad1.right_stick_x + (-gamepad2.right_stick_x/2);

        leftFrontWheelPower = forward + clockWise + strafe;
        rightFrontWheelPower = forward - clockWise - strafe;
        leftBackWheelPower = forward + clockWise - strafe;
        rightBackWheelPower = forward - clockWise + strafe;

        maxPower = Math.abs(leftFrontWheelPower);
        if (Math.abs(rightFrontWheelPower) > maxPower)
            maxPower = Math.abs(rightFrontWheelPower);
        if (Math.abs(leftBackWheelPower) > maxPower)
            maxPower = Math.abs(leftBackWheelPower);
        if (Math.abs(rightBackWheelPower) > maxPower)
            maxPower = Math.abs(rightBackWheelPower);

        if (maxPower > 1) {
            leftFrontWheelPower /= maxPower;
            leftBackWheelPower /= maxPower;
            rightFrontWheelPower /= maxPower;
            rightBackWheelPower /= maxPower;
        }

        motorPowerConstant = DRIVE_MOTOR_HIGH_POWER;

        //Set all 4 Wheel Powers
        robot.leftBackWheel.setPower(leftBackWheelPower * motorPowerConstant);
        robot.leftFrontWheel.setPower(leftFrontWheelPower * motorPowerConstant);
        robot.rightBackWheel.setPower(rightBackWheelPower * motorPowerConstant);
        robot.rightFrontWheel.setPower(rightFrontWheelPower * motorPowerConstant);

        //Update Driver Station with all 4 Wheel Powers
        //telemetry.addData("Left Wheel F/B: ", "%5.2f:%5.2f", (leftFrontWheelPower*motorPowerConstant), (leftBackWheelPower*motorPowerConstant));
        //telemetry.addData("Right Wheel F/B:", "%5.2f:%5.2f", (rightFrontWheelPower*motorPowerConstant), (rightBackWheelPower*motorPowerConstant));
//Carousel Motor Controls---------------------------------------------------------------------------
        if(gamepad1.right_bumper){
            robot.carouselMotor.setPower(CAROUSEL_MOTOR_POWER);
        }else if(gamepad1.left_bumper){
            robot.carouselMotor.setPower(-CAROUSEL_MOTOR_POWER);
        } else {
            robot.carouselMotor.setPower(MOTOR_POWER_ZERO);
        }



//Hand Clamp Controls-------------------------------------------------------------------------------
        if(gamepad2.a){
            robot.closeHand();
        }

        if(gamepad2.b){
            robot.openHand();
        }
//-------------------------------------------------------------------------------
        //Arm Rotate
        armRotateGamepadTrigger = gamepad2.right_trigger - gamepad2.left_trigger;
        armRotatePosition = robot.armRotate.getCurrentPosition();

       /* if(armRotateGamepadTrigger < 0 && armRotatePosition > ARM_ROTATE_MIN_POSITION) {
            robot.armRotate.setPower(ARM_ROTATE_POWER);
        } else if(armRotateGamepadTrigger > 0 && armRotatePosition < ARM_ROTATE_MAX_POSITION) {
            robot.armRotate.setPower(-ARM_ROTATE_POWER);
        } else {
            robot.armRotate.setPower(MOTOR_POWER_ZERO);
        }



        //Update Driver Station with Arm Rotate
        telemetry.addData("Arm Trigger", "%5.2f", armRotateGamepadTrigger);
        telemetry.addData("Arm Position", "%7d", robot.armRotate.getCurrentPosition());*/


//Arm Rotate 2--------------------------------------------------------------------------------------
        if(armRotateGamepadTrigger >  0.1 || armRotateGamepadTrigger < 0.1) {
            robot.armRotate.setPower(armRotateGamepadTrigger*ARM_ROTATE_POWER);
        } else {
            robot.armRotate.setPower(MOTOR_POWER_ZERO);
        }

/*
        if(gamepad2.right_trigger > 0){
            robot.armRotate.setPower(ARM_ROTATE_POWER);
        } else if(gamepad2.left_trigger > 0) {
            robot.armRotate.setPower(-ARM_ROTATE_POWER);
        } else {
            robot.armRotate.setPower(MOTOR_POWER_ZERO);
        }
*/

//--------------------------------------------------------------------------------------------------
        //Free space


//--------------------------------------------------------------------------------------------------
        //Free space


//--------------------------------------------------------------------------------------------------
        telemetry.update();
    }
}