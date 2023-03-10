package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;

public class Auto {
    private PigeonIMU m_gyro;
    private Claw m_claw;
    private CraneArm m_crane;
    private Drivetrain m_swerve;
    private Timer timer;
    private boolean craneArmUp1 = false;
    private boolean craneInPosition = false;
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(DriveConstants.speedRateLimit);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(DriveConstants.speedRateLimit);

    private boolean isGoingUpChargeStation = false;
    private boolean isOnChargeStation = false;
    private boolean isGoingDownChargeStation = false;
    private boolean isOverChargeStation = false;

    public Auto(Claw claw, CraneArm crane, PigeonIMU gyro, Drivetrain swerve) {
        this.m_gyro = gyro;
        this.m_claw = claw;
        this.m_crane = crane;
        this.m_swerve = swerve;
        timer = new Timer();

    }

    /**
     * This plan places a cone in the grid, then goes over the
     * charging station, out of the community, and then back on the charging
     * station.
     */
    public void plan1() {
        // Place the cone in the grid
        placeConeLow();
        // Drive forward until the gyro goes positive, negative, then levels out

        // Drive backwards until the gyro goes negative

        // Start attempting to balance.
    }

    /**
     * Place object then move robot to middle
     */
    public double plan2() {
        // Left Or Right placement of the robot

        // Place a cone
        placeCone();
        // Drive Backwards

        return 1.0;
    }

    /**
     * Places the cone on the grid.
     */
    public void placeCone() {

        if (!craneArmUp1) {
            if (m_crane.getCraneOutput()[0] > .05 && m_crane.getCraneOutput()[1] > .05) {
                m_crane.autoCraneRun(ArmConstants.highPosition, "high");
                
            } else {
                craneArmUp1 = true;
                timer.start();
                System.out.println();
            }
        } else if (!craneInPosition) {
        }
        // Move forard for a set amount of time
        double xSpeed = -m_xspeedLimiter.calculate(0) * Drivetrain.kMaxSpeed * .3;
        double ySpeed = -m_yspeedLimiter.calculate(.1) * Drivetrain.kMaxSpeed * .3;
        // Adjust the arm extension and position to the highest peg
        // Drop the cone on the peg
    }

    public void placeConeLow() {
        timer.start();
        if (!craneArmUp1) {
            if (!timer.hasElapsed(3)) {
                m_crane.autoCraneRun(ArmConstants.groundPosition + 20, "none");
                System.out.println("Arm raising");
            } else if (!timer.hasElapsed(6)) {
                m_crane.autoCraneRun(ArmConstants.groundPosition + 20, "ground");
                System.out.println("Arm extensing");
            } else {
                craneArmUp1 = true;
                timer.reset();
                timer.start();
                System.out.println("In Position");
            }

        } else {
            if (timer.get() < 1.5) {
                m_claw.autoClawRun("out");
                System.out.println("Claw Running");
            } 
            else if (!timer.hasElapsed(6)) {
                m_claw.autoClawRun("stop"); // Stop claw
                m_crane.autoCraneRun(0, "drive");
                driveInDirection("right");
                System.out.println("Drive back");
            } else {
                m_swerve.stop();
            }

        }
    }

    /**
     * Drives the robot forward until the gyro goes positive, then negative, then
     * levels out.
     */
    private void exitCommunityOverChargeStation() {
        // TODO Figure out how to keep track of where in the process the robot is
    }

    /**
     * This runs a function that will move the robot forward if the gyro is positi
     */
    private void dockAndEngage() {
        // If the gyro is positive, move forward
        // If the gyro is negative, move backwards
        // If the gyro is level, stop moving
    }

    public void driveInDirection(String direction) {
        double xSpeedVal = 0.0;
        double ySpeedVal = 0.0;
        if (direction.equalsIgnoreCase("up")) {
            xSpeedVal = DriveConstants.upJoytickInputs[0];
            ySpeedVal = DriveConstants.upJoytickInputs[1];
        } else if (direction.equalsIgnoreCase("down")) {
            xSpeedVal = DriveConstants.downJoytickInputs[0];
            ySpeedVal = DriveConstants.downJoytickInputs[1];
        } else if (direction.equalsIgnoreCase("left")) {
            xSpeedVal = DriveConstants.leftJoytickInputs[0];
            ySpeedVal = DriveConstants.leftJoytickInputs[1];
        } else if (direction.equalsIgnoreCase("right")) {
            xSpeedVal = DriveConstants.rightJoytickInputs[0];
            ySpeedVal = DriveConstants.rightJoytickInputs[1];
        }

        double xSpeed = -m_xspeedLimiter.calculate(xSpeedVal) * Drivetrain.kMaxSpeed * .3;
        double ySpeed = -m_yspeedLimiter.calculate(ySpeedVal) * Drivetrain.kMaxSpeed * .3;

        m_swerve.drive(xSpeed, ySpeed, 0, true);
    }

}
