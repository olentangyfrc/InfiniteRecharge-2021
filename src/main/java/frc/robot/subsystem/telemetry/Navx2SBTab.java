package frc.robot.subsystem.telemetry;

import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystem.SBInterface;

public class Navx2SBTab implements SBInterface{
    private static Logger logger = Logger.getLogger(TelemetrySBTab.class.getName());
    public Navx2 navx2;
    public ShuffleboardTab tab;

    public NetworkTableEntry axis;
    public NetworkTableEntry angle;

    public Navx2SBTab(Navx2 gyro){
        navx2 = gyro;

        tab = Shuffleboard.getTab("Navx2");

        axis = tab.add("Gyro Axis", 0.0).getEntry();
        angle = tab.add("Gyro Angle", 0.0).getEntry();
    }

    public void update(){
        angle.setDouble(navx2.getAngle());
    }
}
