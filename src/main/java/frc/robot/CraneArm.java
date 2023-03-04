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

    private double desiredPosition = 0;
    private XboxController o_controller;

    private PIDController m_CraneUpPIDController = new PIDController(ArmConstants.craneUpP, 0, 0);
    private PIDController m_CraneDownPIDController = new PIDController(ArmConstants.craneDownP, 0, 0);
    
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
        // set the motor output based on jostick position
        // if (o_controller.getLeftY() != deadZone) {
        //     m_CraneMotor.set(o_controller.getLeftY());
        // }
        if (o_controller.getRawButton(9)) {
            desiredPosition = ArmConstants.driveLimit;
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
        // if (o_controller.getLeftX() > 0.3) {
        //     desiredPosition += .5;
        // }
        // if (o_controller.getLeftX() < .3) {
        //     desiredPosition -= .5;
        // }
        if (desiredPosition < ArmConstants.armDownHardLimit) {
            desiredPosition = ArmConstants.armDownHardLimit;
        }
        desiredPosition = ArmConstants.middlePosition;
        // m_extension.extendRun();
        m_extension.buttonExtension(extensionPosition);
        
      
      
        double craneOutput = m_CraneUpPIDController.calculate(m_CraneEncoder.getPosition(), desiredPosition);
        if (craneOutput < 0) {
            craneOutput = m_CraneDownPIDController.calculate(m_CraneEncoder.getPosition(), desiredPosition);
        }
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
        // if (o_controller.getRawButton(5)) {
        //     m_extendingMotor.set(ControlMode.Position,-1.0);
        // }
        // if (o_controller.getRawButton(7)) {
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