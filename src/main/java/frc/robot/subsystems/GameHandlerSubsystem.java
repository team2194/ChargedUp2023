// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class GameHandlerSubsystem extends SubsystemBase {

    private static GridDrop activeDrop = GridDrop.COOP_CUBE;

    public static int gridHeight = 0;// 0 floor, 1 mid, 2 high

    public static final double xDist = 1.25;

    private static double fieldLength = Units.inchesToMeters((54 * 12) + 3.25);

    private static double fieldWidth = Units.inchesToMeters((26 * 12) + 3.5);

    public static boolean setDone;

    /**
     * Grid values as viewed by Blue Drivers starting on their right at the field
     * RIGHT
     * Red drivers have the field RIGHT on their left
     * The X distance is the center of the robot as it strafes past the grids
     * For Blue it will be the distance used.
     * For Red it will be the field length minus that distance
     * Similarly Y will be used direct or by subtracting from the field width
     * Grids are 66" wide
     */

    public enum GridDrop {

        RIGHT_HYBRID_PIPE(xDist, Units.inchesToMeters(20.19), true),

        RIGHT_CUBE(xDist, Units.inchesToMeters(47.19), false), // April Tag 47.19"

        RIGHT_PIPE(xDist, Units.inchesToMeters(20.19 + 44), true),

        COOP_RIGHT_PIPE(xDist, Units.inchesToMeters(20.19 + 66), true),

        COOP_CUBE(xDist, Units.inchesToMeters(108.19), false), // April Tag 108.19"

        COOP_LEFT_PIPE(xDist, Units.inchesToMeters(108.19), true),

        LEFT_PIPE(xDist, Units.inchesToMeters(108.19 + 22), true),

        LEFT_CUBE(xDist, Units.inchesToMeters(147.19), false), // AprilTag 147.19"

        LEFT_HYBRID_PIPE(xDist, Units.inchesToMeters(108.19 + 44), true);

        private final double yVal;

        private final double xVal;

        private final boolean isPipe;

        private GridDrop(double xVal, double yVal, boolean isPipe) {
            this.yVal = yVal;
            this.xVal = xVal;
            this.isPipe = isPipe;
        }

        public double getYVal() {
            return yVal;
        }

        public double getXVal() {
            return xVal;
        }

        public boolean getIsPipe() {
            return isPipe;
        }
    }

    String[] pieceName = { "CONE", "CUBE" };

    String[] levelName = { "FLOOR", "MID", "HIGH" };

    public int selectedGrid;
    public int selectedPiece;
    public int selectedLevel;

    private boolean allianceBlue;

    public int activeDropNumber = 0;

    public String[] dropNames =

            { GridDrop.RIGHT_HYBRID_PIPE.toString(), GridDrop.RIGHT_CUBE.toString(), GridDrop.RIGHT_PIPE.toString(),
                    GridDrop.COOP_RIGHT_PIPE.toString(), GridDrop.COOP_CUBE.toString(),
                    GridDrop.COOP_LEFT_PIPE.toString(),
                    GridDrop.LEFT_PIPE.toString(), GridDrop.LEFT_CUBE.toString(), GridDrop.LEFT_HYBRID_PIPE.toString()
            };

    private int[] lookupRedIndex = { 8, 7, 6, 5, 4, 3, 2, 1, 0 };

    private int nameIndex = 0;

    public boolean wantConeForPickup;

    public boolean wantCubeForPickup;

    public boolean robotHasCube;

    public boolean robotHasCone;

    private int dropOfflevel;

    public GameHandlerSubsystem() {

    }

    public void setAllianceBlue(boolean alliance) {
        allianceBlue = alliance;
    }

    /**
     * drop number comes in from the driver station as a value from 0 to 8
     * In the Blue Alliance case, the number cam go straight though and be used as
     * is
     * It is used to set the active drop and this picks out the x, y and pipe
     * entries
     * It also picks the name out of the String array for display purposes
     * In the RedAlliance case, the number needs to be treated differently
     * The String selection remains the same but the x, y and pipe come from the
     * opposite way
     * in the enum
     * For example, the RightHybridPipe is index (ordinal)0. However for Red,
     * the values returned have to be from the LeftHybridPipe - index 8
     * So for Red internally setting the active drop will look wrong but the values
     * will be correct
     * The original index gets used for the String name
     * 
     * 
     * @param drop
     */
    public static void setActiveDrop(GridDrop drop) {
        activeDrop = drop;
    }

    public GridDrop getActiveDrop() {
        return activeDrop;
    }

    public void setDropByNumber(int n) {
        nameIndex = n;
        if (!getAllianceBlue()) {
            n = lookupRedIndex[n];
        }

        setDone = false;
        SmartDashboard.putNumber("SETN", n);

        switch (n) {
            case 0:
                activeDrop = GridDrop.RIGHT_HYBRID_PIPE;
                break;
            case 1:
                activeDrop = GridDrop.RIGHT_CUBE;
                break;
            case 2:
                setActiveDrop(GridDrop.RIGHT_PIPE);
                break;
            case 3:
                setActiveDrop(GridDrop.COOP_RIGHT_PIPE);
                break;
            case 4:
                setActiveDrop(GridDrop.COOP_CUBE);
                break;
            case 5:
                setActiveDrop(GridDrop.COOP_LEFT_PIPE);
                break;
            case 6:
                setActiveDrop(GridDrop.LEFT_PIPE);
                break;
            case 7:
                setActiveDrop(GridDrop.LEFT_CUBE);
                break;
            case 8:
                setActiveDrop(GridDrop.LEFT_HYBRID_PIPE);
                break;

            case 9:
                break;

            default:
                break;

        }
        setDone = true;

        SmartDashboard.putString("ADRP", activeDrop.toString());

    }

    public double getXDistance() {
      
            return xDist;
       
    }

    public void setActiveDropNumber(int n) {

    }

    public boolean getAllianceBlue() {
        return allianceBlue;
    }

    public Command setDrop(int n) {
        return new InstantCommand(() -> setActiveDropNumber(n));
    }

    public String getActiveName() {
        return dropNames[nameIndex];
    }

    public void setConeForPickup() {
        wantConeForPickup = true;
        wantCubeForPickup = false;
    }

    public void setCubeForPickup() {
        wantCubeForPickup = true;
        wantConeForPickup = false;
    }

    public void setDropOffLevel(int level) {
        dropOfflevel = level;
    }

    @Override

    public void periodic() {
    }

}
