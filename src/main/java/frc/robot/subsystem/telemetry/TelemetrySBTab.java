/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.telemetry;

import java.util.logging.Logger;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystem.SBInterface;

/**
 * Add your docs here.
 */
public class TelemetrySBTab implements SBInterface {
    private static Logger logger = Logger.getLogger(TelemetrySBTab.class.getName());
    public Telemetry telemetry;
    public ShuffleboardTab tab;

    public NetworkTableEntry frontDistance;
    public NetworkTableEntry rearDistance;
    public NetworkTableEntry isSquare;
    public NetworkTableEntry tolerance;
    public NetworkTableEntry horizontalTolerance;
    public NetworkTableEntry horizontalDirection;
    public NetworkTableEntry targetDistance;
    public double lidarTolerance = 2.34;

    public TelemetrySBTab(Telemetry te){
        telemetry = te;
        
        tab = Shuffleboard.getTab("Telemetry");

        frontDistance = tab.add("Front Lidar Distance", 0).getEntry();
        rearDistance = tab.add("Rear Lidar Distance", 0).getEntry();
        isSquare = tab.add("Is Squared?", false).getEntry();
        tolerance = tab.add("Lidar Tolerance", 0.0).getEntry();
        horizontalTolerance = tab.add("Horizontal Lidar Tolerance", 0.0).getEntry();
        horizontalDirection = tab.add("Horizontal Direction", 0.0).getEntry();
        targetDistance = tab.add("Target Distance", 0.0).getEntry();

    }
    public void update(){
        isSquare.setBoolean(telemetry.isSquare(lidarTolerance));
        frontDistance.setDouble(telemetry.getFrontLidarDistance());
        rearDistance.setDouble(telemetry.getRearLidarDistance());
       // tolerance.setDouble(telemetry.getTolerance());
       // telemetry.setTolerance(tolerance.getDouble(5.0));
        telemetry.setTolerance(tolerance.getDouble(10.0));
        telemetry.setHorDirection(telemetry.directionToGo());
        telemetry.setTargetDistance(targetDistance.getDouble(10.0));
    }
}
