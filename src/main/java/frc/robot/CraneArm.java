package frc.robot; 

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ArmConstants;

public class CraneArm {
    private static final int deviceID = 4;
    private CANSparkMax m_CraneMotor;
    private RelativeEncoder m_CraneEncoder;
    private ExtensionArm m_extension;
    private String extensionPosition;

    public static final double kMaxSpeed = 3.0; // 3 meters per second

    public double desiredPosition = 0;
    private XboxController o_controller;

    private PIDController m_CraneUpPIDController = new PIDController(ArmConstants.craneUpP, 0, 0);
    private PIDController m_CraneDownPIDController = new PIDController(ArmConstants.craneDownP, 0, 0);

    private boolean manualExtension = false;
    private boolean manualArm = false;
    
    public CraneArm(XboxController o_controller) {
        this.o_controller = o_controller;
        m_extension = new ExtensionArm(o_controller);

        

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

    public double encoderPosition() {
        return m_CraneEncoder.getPosition();
    }

    public double extenisonEncoder() {
        return m_extension.extenisonEncoder();
    }

    
    public void craneRun() {
        if (m_CraneEncoder.getPosition() > ArmConstants.armDownHardLimit && o_controller.getLeftY() > 0.3) {
            // Move down if you are within the limit and dedband
            System.out.println("Going down");
            manualArm = true;
            m_CraneMotor.set(-.1);
        } else if (m_CraneEncoder.getPosition() < ArmConstants.armUpHardLimit && o_controller.getLeftY() < -0.3) {
            // Move up if you are within the limit and dedband
            System.out.println("Going up");
            manualArm = true;
            m_CraneMotor.set(.1);
        } else if (manualArm) {
            System.out.println("MANUAL ENDING");
            manualArm = false;
            desiredPosition = m_CraneEncoder.getPosition();
        }

        if (m_extension.extenisonEncoder() < ArmConstants.extensionLimit && o_controller.getRawButton(7)) {
            System.out.println("Extending");
            manualExtension = true;
            m_extension.m_extendingMotor.set(.1);
        } else if (m_extension.extenisonEncoder() > ArmConstants.retractionLimit && o_controller.getRawButton(5)) {
            System.out.println("Retracting");
            manualExtension = true;
            m_extension.m_extendingMotor.set(-.1);
        } else if (manualExtension) {
            System.out.println("MANUAL ENDING");
            manualExtension = false;
            m_extension.extensionDesiredPosition = m_extension.extenisonEncoder();
            extensionPosition = "manual";
        }

        if (o_controller.getRawButton(9)) {
            desiredPosition = ArmConstants.driveLimit;
            extensionPosition = "drive";
        }
        // Ground Position - A
        if (o_controller.getRawButton(2)) {
            desiredPosition = ArmConstants.groundPosition;
            extensionPosition = "ground";
        }
        // Middle position - X
        if (o_controller.getRawButton(1)) {
            desiredPosition = ArmConstants.middlePosition;
            extensionPosition = "middle";
        }
        // Human position - B
        if (o_controller.getRawButton(3)) {
            desiredPosition = ArmConstants.humanPosition;
            extensionPosition = "human";
        }
        // High position - Y
        if (o_controller.getRawButton(4)) {
            desiredPosition = ArmConstants.highPosition;
            extensionPosition = "high";
        }
   
     
        // desiredPosition = ArmConstants.middlePosition;
        // m_extension.extendRun();
        if (extensionPosition != null) {
            m_extension.buttonExtension(extensionPosition, manualExtension);
        }
        
      
        double craneOutput = m_CraneUpPIDController.calculate(m_CraneEncoder.getPosition(), desiredPosition);
        if (craneOutput < 0) {
            craneOutput = m_CraneDownPIDController.calculate(m_CraneEncoder.getPosition(), desiredPosition);
        }
        System.out.println("Desired position: " + desiredPosition);
        System.out.println("Crane output: " + craneOutput);
        if (!manualArm) {
            m_CraneMotor.set(craneOutput);
        }


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