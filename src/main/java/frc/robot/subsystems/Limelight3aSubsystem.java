package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight3aSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;

    public Limelight3aSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getX() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    public void setLED(boolean on) {
        limelightTable.getEntry("ledMode").setNumber(on ? 3 : 1);
    }
}