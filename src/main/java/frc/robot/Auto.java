package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

public class Auto {
    private PigeonIMU m_gyro = new PigeonIMU(30);

    /**
     * This plan places a cone in the grid, then goes over the
     * charging station, out of the community, and then back on the charging
     * station.
     */
    public void plan1A() {
        // Place the cone in the grid

        // Drive forward until the gyro goes positive, negative, then levels out

        // Drive backwards until the gyro goes negative

        // Start attempting to balance.
    }

    /**
     * This plan places a cube in the grid, then goes over the
     * charging station, out of the community, and then back on the charging
     * station.
     */
    public void plan1B() {
        // Place the cone in the grid

        // Drive forward until the gyro goes positive, negative, then levels out

        // Drive backwards until the gyro goes negative

        // Start attempting to balance.
    }

    /**
     * Place object then move robot to middle
     */
    public void plan2() {

    }

    /**
     * Places the cone on the grid.
     */
    public void placeCone() {
        // Align the robot with the target peg using the reflective sensor output

        // Adjust the arm extension and position to the highest peg

        // Drop the cone on the peg
    }

    /**
     * Places the cube on the grid.
     */
    public void placeCube() {
        // Align the robot with the target platform using the camera output (April Tags)

        // Adjust the arm extension and position to the highest peg

        // Drop the cone on the peg
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

}
