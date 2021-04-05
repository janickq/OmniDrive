package frc.robot.subsystems;

//Java imports
import java.util.Map;

//Vendor imports
import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Cobra;
import com.studica.frc.ServoContinuous;
import com.studica.frc.TitanQuad;
//import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//WPI imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OmniDrive extends SubsystemBase
{
    //Creates all necessary hardware interface here for omni-drive

    //Motors and encoders
    private final TitanQuad[] motors;
    //private final TitanQuadEncoder[] encoders;
    private final Encoder[] encoders;

    //PID stuff
    private PIDController[] pidControllers;
    private double[] pidInputs;
    private double[] pidOutputs;
    private double[] encoderDists;
    private double[] encoderDists_2;
    private double[] encoderSpeeds;
    private double curHeading, targetHeading;
    private double[] motorOuts;

    //For testing. These should be in another subsystem
    private final Servo servo;
    private final ServoContinuous servoC;
    private double dT = 0.02;

    // Sensors
    private final DigitalInput input10;
    private final DigitalOutput outDebug11;
    private final Cobra cobra;
    //private final Ultrasonic sonic;
    private final AnalogInput sharp;
    private final AHRS gyro;

    // Shuffleboard
    private final ShuffleboardTab tab = Shuffleboard.getTab("Training");
    //private final NetworkTableEntry D_servoPos = tab.add("Servo Position", 0).withWidget(BuiltInWidgets.kNumberSlider)
    //       .withProperties(Map.of("min", 0, "max", 300)).getEntry();
    private final NetworkTableEntry D_sharpIR = tab.add("Sharp IR", 0).getEntry();
    //private final NetworkTableEntry D_ultraSonic = tab.add("Ultrasonic", 0).getEntry();
    private final NetworkTableEntry D_cobraRaw = tab.add("Cobra Raw", 0).getEntry();
    //private final NetworkTableEntry D_cobraVoltage = tab.add("Cobra Voltage", 0).getEntry();
    private final NetworkTableEntry D_navYaw = tab.add("Nav Yaw", 0).getEntry();
    private final NetworkTableEntry D_curHeading = tab.add("curHeading", 0).getEntry();
    private final NetworkTableEntry D_tgtHeading = tab.add("tgtHeading", 0).getEntry();
    private final NetworkTableEntry D_inputDisp = tab.add("Input10", false).getEntry();
    private final NetworkTableEntry D_encoderDisp0 = tab.add("Encoder0", 0).getEntry();
    private final NetworkTableEntry D_encoderDisp1 = tab.add("Encoder1", 0).getEntry();
    private final NetworkTableEntry D_encoderDisp2 = tab.add("Encoder2", 0).getEntry();
    private final NetworkTableEntry D_inputW = tab.add("inputW", 0).getEntry();

    //Subsystem for omnidrive
    public OmniDrive() {
        
        input10 = new DigitalInput(10);
        outDebug11 = new DigitalOutput(8);

        //Omni drive motors
        motors = new TitanQuad[Constants.MOTOR_NUM];
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motors[i] = new TitanQuad(Constants.TITAN_ID, i);
            motors[i].setInverted(true);   //Positive is CW. Need to reverse
        }


        //encoders = new TitanQuadEncoder[Constants.MOTOR_NUM];
        encoders = new Encoder[Constants.MOTOR_NUM];
        encoderDists = new double[Constants.MOTOR_NUM];
        encoderDists_2 = new double[Constants.MOTOR_NUM];
        encoderSpeeds = new double[Constants.MOTOR_NUM];
        motorOuts = new double[Constants.MOTOR_NUM];

        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            encoders[i] = new Encoder(i*2, i*2+1, false, Encoder.EncodingType.k4X);
            encoders[i].setDistancePerPulse(Constants.KENCODERDISTPERPULSE);
            //encoders[i] = new TitanQuadEncoder(motors[i], i, Constants.KencoderDistPerPulse);
            //encoders[i].reset();
            encoderDists[i] = encoders[i].getDistance();
        }
        
        // x, y and w speed controler
        pidControllers = new PIDController[Constants.PID_NUM];
        pidControllers[0] = new PIDController(2.0,32.0,0.02);  //x
        pidControllers[1] = new PIDController(2.0,32.0,0.02);  //y
        pidControllers[2] = new PIDController(2.0,0.0,0.1);    //w
        pidControllers[2].enableContinuousInput(-Math.PI, Math.PI);

        //Inputs and Outputs for wheel controller
        pidInputs = new double[Constants.PID_NUM];
        pidOutputs = new double[Constants.PID_NUM];

        servo = new Servo(Constants.SERVO);
        servoC = new ServoContinuous(Constants.SERVO_C);

        // Sensors
        cobra = new Cobra();
        sharp = new AnalogInput(Constants.SHARP);
        //sonic = new Ultrasonic(Constants.SONIC_TRIGG, Constants.SONIC_ECHO);

        // gyro for rotational heading control
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();
        curHeading = targetHeading = getYawRad();
    }

    public double getYawRad() {
        return -gyro.getYaw()*Math.PI/180;
    }
    public Boolean getSwitch() {
        return input10.get();
    }

    /**
     * Call for the raw ADC value
     * <p>
     * 
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return value between 0 and 2047 (11-bit)
     */
    public int getCobraRawValue(final int channel) {
        return cobra.getRawValue(channel);
    }

    /**
     * Call for the voltage from the ADC
     * <p>
     * 
     * @param channel range 0 - 3 (matches what is on the adc)
     * @return voltage between 0 - 5V (0 - 3.3V if the constructor Cobra(3.3F) is
     *         used)
     */
    public double getCobraVoltage(final int channel) {
        return cobra.getVoltage(channel);
    }

    /**
     * Call for the distance measured by the Sharp IR Sensor
     * <p>
     * 
     * @return value between 0 - 100 (valid data range is 10cm - 80cm)
     */
    public double getIRDistance() {
        return (Math.pow(sharp.getAverageVoltage(), -1.2045) * 27.726);
    }


    /**
     * Call for the current angle from the internal NavX
     * <p>
     * 
     * @return yaw angle in degrees range -180° to 180°
     */
    public double getYaw() {
        //return gyro.getYaw();
        return gyro.getRawGyroZ();
    }

    /**
     * Resets the yaw angle back to zero
     */
    public void resetGyro() {
        gyro.zeroYaw();
    }

  
    /**
     * Sets the servo angle
     * <p>
     * 
     * @param degrees degree to set the servo to, range 0° - 300°
     */
    public void setServoAngle(final double degrees) {
        servo.setAngle(degrees);
    }


    /**
     * Sets the servo speed
     * <p>
     * 
     * @param speed sets the speed of the servo in continous mode, range -1 to 1
     */
    public void setServoSpeed(final double speed) {
        servoC.set(speed);
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setMotorSpeedAll(final double speed)
    {
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motors[i].set(speed);
        }
        
    }
    // CCW is positive
    public void setMotorSpeed012(double speed0, double speed1, double speed2)
    {
        
        motors[0].set(speed0);
        motors[1].set(speed1);
        motors[2].set(speed2);
        
    }
    
    /***
     * 
     * @param x - x speed in m/s
     * @param y - y speed in m/s
     * @param w - rotational speed in rad/s
     */
    public void setRobotSpeedXYW(double x, double y, double w) {
        pidInputs[0] = x; 
        pidInputs[1] = y;
        pidInputs[2] = w; 
    }
    public void setRobotSpeedType(int type, double speed) {
        pidInputs[type] = speed; 
    }

    public void doPID( ){

        //This is for translational speed PID
        //First calculate wheel speed from encoder feedback
        double dcValue = 0.0;
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            encoderDists[i] = encoders[i].getDistance();
            encoderSpeeds[i] = (encoderDists[i]-encoderDists_2[i])/dT;
            dcValue += encoderSpeeds[i];
            encoderDists_2[i] = encoderDists[i];
        }

        //Subtract rotational component from encoder speed
        //Rotational PID is handled by gyro separately.
        //Maybe good to combine this dc value with gyro value??????
        dcValue /= 3;
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            encoderSpeeds[i] -= dcValue;
        }

        //Estimates x and y speed from individual wheel speeds
        //See formula below
        double speedX = (-(encoderSpeeds[0] + encoderSpeeds[2]) + encoderSpeeds[1])/2;
        double speedY = (-encoderSpeeds[0] + encoderSpeeds[2])/(0.866025*2);

        //PID control for x and y speed
        pidOutputs[0] = pidControllers[0].calculate(speedX, pidInputs[0]);
        pidOutputs[1] = pidControllers[1].calculate(speedY, pidInputs[1]);
        
        //Translate x and y output to wheel outputs
        // The x and y speed are resolved into individual wheel speed
        // 3 wheel omni drive
        // R is distance of wheel from robot centre
        // M0 = [-sin(150) cos(150) R] * [x y w]    //Left-front wheel
        // M1 = [-sin(270) cos(270) R]              //Back wheel
        // M2 = [-sin(30)  cos(30)  R]              //Right-front wheel
        motorOuts[0] = (-0.5*pidOutputs[0] - 0.866025*pidOutputs[1]);
        motorOuts[1] = (     pidOutputs[0] + 0               );
        motorOuts[2] = (-0.5*pidOutputs[0] + 0.866025*pidOutputs[1]);
        
        /////////////////////////////////////////////////////////////////////////////////////////
        //This is for rotational speed PID
        /////////////////////////////////////////////////////////////////////////////////////////
        curHeading = getYawRad();
        
        targetHeading += pidInputs[2]*dT;   

        //Limit targetHeading to -Pi to +Pi
        if (targetHeading>Math.PI) targetHeading -= Math.PI*2;
        if (targetHeading<-Math.PI) targetHeading += Math.PI*2;

        pidOutputs[2] = pidControllers[2].calculate(curHeading, targetHeading);

        //Limit output to -1.0 to 1.0 as PID outputs may be greater then 1.0
        double max=0;
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
            motorOuts[i] += pidOutputs[2];          // add w component
            max = Math.max(max, Math.abs(motorOuts[i]));
        }
        if (max<1.0) max = 1.0;   
        for (int i=0; i<Constants.MOTOR_NUM; i++) {
             motors[i].set(motorOuts[i]/max);
        }   
   }
    /**
     * Code that runs once every robot loop
     */
    int initCnt=0;
    @Override
    public void periodic()
    {
        System.out.print("obj");
        if (initCnt<1) {
            initCnt++;
            gyro.zeroYaw();
            curHeading = targetHeading = getYawRad();
            return;
        }
        outDebug11.set(true);
        //
        //encoders[0].getEncoderDistance();

        doPID();
        //setServoAngle(D_servoPos.getDouble(0.0));
        //setMotorSpeedAll(D_motorSpeed.getDouble(0.0));
        /**
         * Updates for outputs to the shuffleboard
         */
        D_inputDisp.setBoolean(getSwitch());
        D_sharpIR.setDouble(getIRDistance());
        //D_ultraSonic.setDouble(getSonicDistance(true)); //set to true because we want metric
        //double s0 = getCobraRawValue(0);
        //double s1 = getCobraRawValue(1);
        //double s2 = getCobraRawValue(2);
       // double s3 = getCobraRawValue(3);
        //double offset = (s0*-3.0 + s1*-1.0 + s2*1.0 + s3*3.0)/(s0+s1+s2+s3);
        D_cobraRaw.setDouble(0); //Just going to use channel 0 for demo

        //D_cobraVoltage.setDouble(getCobraVoltage(0));
        //D_curHeading.setDouble(curHeading);
        D_curHeading.setDouble(curHeading*180/Math.PI);
        D_tgtHeading.setDouble(targetHeading*180/Math.PI);
        D_navYaw.setDouble(-gyro.getYaw());
        D_encoderDisp0.setDouble(encoders[0].getRaw());//encoderSpeeds[0]);
        D_encoderDisp1.setDouble(encoders[1].getDistance());//encoderSpeeds[1]);
        D_encoderDisp2.setDouble(encoders[2].getDistance());//encoderSpeeds[2]);
        D_inputW.setDouble(pidInputs[2]);
        outDebug11.set(false);
    }
}