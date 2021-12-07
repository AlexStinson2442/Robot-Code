package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareGreenMachine {

//Definitions---------------------------------------------------------------------------------------

    final double MOTOR_POWER_ZERO                               = 0;

    static final double     COUNTS_PER_MOTOR_REV                = 288 ;    // REV Motor Encoder
    static final double     COUNTS_PER_MOTOR_TETRIX             = 1440 ;    // Tetrix Motor Encoder

    // Movement Constants
    static final double LEFT_FRONT_MOTOR_NORMALIZED             = 1;
    static final double LEFT_BACK_MOTOR_NORMALIZED              = 1;
    static final double RIGHT_FRONT_MOTOR_NORMALIZED            = 1;
    static final double RIGHT_BACK_MOTOR_NORMALIZED             = 1;

    final double SERVO_MAX_DEG                                  = 180;
    final double HAND_OPEN                                      = 50 / SERVO_MAX_DEG;
    final double HAND_CLOSED                                    = 180 / SERVO_MAX_DEG;

    public DcMotor leftFrontWheel = null;
    public DcMotor leftBackWheel = null;
    public DcMotor rightFrontWheel = null;
    public DcMotor rightBackWheel = null;

    public DcMotor carouselMotor = null;

    public DcMotor armRotate = null;

    public Servo handClamp = null;

    DistanceSensor sensorRedDistance;
    DistanceSensor sensorBlueDistance;






    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /* Constructor */
    public HardwareGreenMachine(){
    }

//Normalization-------------------------------------------------------------------------------------
    /**
     * Method to get normalized/balance the wheel power to revolutions
     * Left of the robot is looking from the driver seat (left hand drive), if the robot has a driver seat
     * @param lfMotor Left Front motor power
     * @param lbMotor Left Back motor power
     * @param rfMotor Right Front motor power
     * @param rbMotor Right Back motor power
     *
     */
    public void motorPower(double lfMotor, double lbMotor, double rfMotor, double rbMotor){
        double max;

        //Motor power normalization
        lfMotor = lfMotor * LEFT_FRONT_MOTOR_NORMALIZED;
        lbMotor = lbMotor * LEFT_BACK_MOTOR_NORMALIZED;
        rfMotor = rfMotor * RIGHT_FRONT_MOTOR_NORMALIZED;
        rbMotor = rbMotor * RIGHT_BACK_MOTOR_NORMALIZED;

        //Get maximum value
        max = Math.abs(lfMotor);
        if (Math.abs(lbMotor) > max)
            max = Math.abs(lbMotor);
        if (Math.abs(rfMotor) > max)
            max = Math.abs(rfMotor);
        if (Math.abs(rbMotor) > max)
            max = Math.abs(rbMotor);

        //Normalize speeds if either one exceeds +/- 1.0;
        if (max > 1.0) {
            lfMotor /= max;
            lbMotor /= max;
            rfMotor /= max;
            rbMotor /= max;
        }

        //Set all 4 Wheel Powers
        leftFrontWheel.setPower(lfMotor);
        leftBackWheel.setPower(lbMotor);
        rightFrontWheel.setPower(rfMotor);
        rightBackWheel.setPower(rbMotor);
    }

//Hand Definitions----------------------------------------------------------------------------------
    public void openHand() {handClamp.setPosition(HAND_OPEN);}
    public void closeHand() {handClamp.setPosition(HAND_CLOSED);}
//Get & Return Imu Z Angle--------------------------------------------------------------------------
    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double angle = angles.firstAngle;
        return angle;
    }
//Autonomous Drive Hardware Initialization----------------------------------------------------------
    public void autonomousDrive(){
    openHand();
    }
//TeleOp Drive Hardware Initialization--------------------------------------------------------------
    public void teleopDrive(){

            }
//Standard Hardware Interfaces Initialization-------------------------------------------------------
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
        }

        sensorRedDistance = hwMap.get(DistanceSensor.class, "sensor_red_distance");
        sensorBlueDistance = hwMap.get(DistanceSensor.class, "sensor_blue_distance");



//Motor Hardware Definitions------------------------------------------------------------------------
        //Define and Initialize Wheel Motors / Servos
        leftFrontWheel = hwMap.get(DcMotor.class, "left_front_wheel");
        rightFrontWheel = hwMap.get(DcMotor.class, "right_front_wheel");
        leftBackWheel = hwMap.get(DcMotor.class, "left_back_wheel");
        rightBackWheel = hwMap.get(DcMotor.class, "right_back_wheel");

        //Put Right Side Wheels in Reverse Mode
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);

        //Run Motors without Encoders
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set all Motors to zero
        leftFrontWheel.setPower(MOTOR_POWER_ZERO);
        leftBackWheel.setPower(MOTOR_POWER_ZERO);
        rightFrontWheel.setPower(MOTOR_POWER_ZERO);
        rightBackWheel.setPower(MOTOR_POWER_ZERO);

        /*//Arm rotation motor
        armRotate = hwMap.get(DcMotor.class, "arm_rotate");
        //armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotate.setPower(MOTOR_POWER_ZERO);*/

//Carousel Hardware Definitions---------------------------------------------------------------------
        carouselMotor = hwMap.get(DcMotor.class, "carousel_motor");
        carouselMotor.setPower(MOTOR_POWER_ZERO);
//Hand Clamp Hardware Definitions-------------------------------------------------------------------
    //    handClamp = hwMap.get(Servo.class, "hand_clamp");
      //  openHand();
//Arm Rotate----------------------------------------------------------------------------------------
        armRotate = hwMap.get(DcMotor.class,"arm_rotate");
        armRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRotate.setPower(MOTOR_POWER_ZERO);
//Distance Sensor Definition
      //  distanceSensor = hwMap.get(DistanceSensor.class, "distance_sensor");
        //double DSD = distanceSensor.getDistance(DistanceUnit.INCH)
//Hand Servo Definitions
      //  handServo = hwMap.get(Serv)

//------------------------------------------------------------------------------------------------
    }

}