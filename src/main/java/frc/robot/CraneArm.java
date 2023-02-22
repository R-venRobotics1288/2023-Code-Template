package frc.robot; 

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import javax.swing.text.Position;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class CraneArm {
    private XboxController m_Guapo; // switch variable name back to "controller"
    private static final int deviceID = 4;
    private CANSparkMax m_CraneMotor;
    private RelativeEncoder m_CraneEncoder;

    private TalonSRX m_extendingMotor;

    private TalonSRX ClawMotor;

    public static final double kMaxSpeed = 3.0; // 3 meters per second
    private static final double deadZone = 0.1;
    private double desiredPosition = 0;

    private PIDController m_CranePIDController = new PIDController(0.25, 0, 0);
    
    public void init() {
        TalonSRX m_extendingMotor = new TalonSRX(1);

        // initialize SPARK MAX
        m_CraneMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_CraneMotor.restoreFactoryDefaults();
        
        /**
        * In order to read encoder values an encoder object is created using the 
        * getEncoder() method from an existing CANSparkMax object
        */
        m_CraneEncoder = m_CraneMotor.getEncoder();
        m_Guapo = new XboxController(0);

        /**
        * Soft Limits restrict the motion of the motor in a particular direction
        * at a particular point. Soft limits can be applied in only one direction, 
        * or both directions at the same time.
        * 
        * If the soft limits are disabled and then re-enabled, they will retain
        * the last limits that they had for that particular direction.
        * 
        * The directions are rev::CANSparkMax::kForward and rev::CANSparkMax::kReverse
        */
      //  m_CraneMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
       // m_CraneMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        
      //  REVLibError forwardLibError =  m_CraneMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ArmConstants.downSoftLimit);
      //  REVLibError revsRevLibError =  m_CraneMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ArmConstants.upSoftLimit);

      //  System.out.println(forwardLibError);
      //  System.out.println(revsRevLibError);

        SmartDashboard.putBoolean("Forward Soft Limit Enabled",
                              m_CraneMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kForward));
        SmartDashboard.putBoolean("Reverse Soft Limit Enabled",
                              m_CraneMotor.isSoftLimitEnabled(CANSparkMax.SoftLimitDirection.kReverse));                          
        SmartDashboard.putNumber("Forward Soft Limit",
                              m_CraneMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
        SmartDashboard.putNumber("Reverse Soft Limit",
                              m_CraneMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
    }

    public void setZero() {
        m_CraneEncoder.setPosition(0);
       // desiredPosition = 0;
        System.out.println("\n**Robot Init**\n");
    }
    
    public void run() {
        // set the motor output based on jostick position
        // if (m_Guapo.getLeftY() != deadZone) {
        //     m_CraneMotor.set(m_Guapo.getLeftY());
        // // }

        // Ground Position - A
        if (m_Guapo.getRawButton(2)) {
            desiredPosition = ArmConstants.groundPosition;
        }
        // Middle position - X
        if (m_Guapo.getRawButton(1)) {
            desiredPosition = ArmConstants.middlePosition;
        }
        // Human position - B
        if (m_Guapo.getRawButton(3)) {
            desiredPosition = ArmConstants.humanPosition;
        }
        // High position - Y
        if (m_Guapo.getRawButton(4)) {
            desiredPosition = ArmConstants.highPosition;
        }
        
        final double craneOutput = m_CranePIDController.calculate(m_CraneEncoder.getPosition(), desiredPosition);
        m_CraneMotor.set(craneOutput);

        // For testing purposes
        int delta = (int)m_CraneEncoder.getPosition() - (int)desiredPosition;
        System.out.println("Encoder: " + m_CraneEncoder.getPosition());
        System.out.println("desiredPosition: " + desiredPosition);
        System.out.println("delta: " + delta);

        // OLD CODE
        // if (delta > ArmConstants.deadband) {
        //     m_CraneMotor.set(-ArmConstants.speed);
        //     System.out.println("Moving down");
        // }  
        // else if (delta < -ArmConstants.deadband) {
        //     m_CraneMotor.set(ArmConstants.speed);
        //     System.out.println("Moving up");
        // }
        // else if (delta >= -ArmConstants.deadband && delta <= ArmConstants.deadband) {
        //     m_CraneMotor.set(0);
        //     System.out.println("Stopped");
        // }

        // This will be implemented after we get the rotational crane motion working
        // if (m_Guapo.getRawButton(5)) {
        //     m_extendingMotor.set(ControlMode.Position,-1.0);
        // }
        // if (m_Guapo.getRawButton(7)) {
        //     m_extendingMotor.set(ControlMode.Position,1.0);
        // }


        /**
        * Encoder position is read from a RelativeEncoder object by calling the
        * GetPosition() method.
        * 
        * GetPosition() returns the position of the encoder in units of revolutions
        */
        SmartDashboard.putNumber("Encoder Position", m_CraneEncoder.getPosition());

        /**
        * Encoder velocity is read from a RelativeEncoder object by calling the
         * GetVelocity() method.
        * 
        * GetVelocity() returns the velocity of the encoder in units of RPM
        */
        SmartDashboard.putNumber("Encoder Velocity", m_CraneEncoder.getVelocity());
    }
    /** SmartDashboard.putNumber("Voltage", m_motor.getBusVoltage());
    use if we have brown outs and need to know the voltage */
}