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
    public NetworkTableEntry backDistance;
    public NetworkTableEntry isSquare;
    public NetworkTableEntry tolerance;
    public NetworkTableEntry translationalTolerance;
    public NetworkTableEntry horizontalDirection;
    public NetworkTableEntry horizontalTargetDistance;
    public NetworkTableEntry verticalTargetDistance;
    public NetworkTableEntry verticalDirection;
    public NetworkTableEntry rotationalSpeed;
    public NetworkTableEntry translationalSpeed;
    public NetworkTableEntry lidarDifference;
    public double lidarTolerance = 2.34;

    public TelemetrySBTab(Telemetry te){
        telemetry = te;
        
        tab = Shuffleboard.getTab("Telemetry");

        frontDistance = tab.add("Front Lidar Distance", 0).getEntry();
        rearDistance = tab.add("Rear Lidar Distance", 0).getEntry();
        backDistance = tab.add("Back Lidar Distance", 0).getEntry();
        isSquare = tab.add("Is Squared?", false).getEntry();
        tolerance = tab.add("Rotational Tolerance", 0.0).getEntry();
        translationalTolerance = tab.add("Translational Lidar Tolerance", 0.0).getEntry();
        horizontalDirection = tab.add("Horizontal Direction", 0.0).getEntry();
        horizontalTargetDistance = tab.add("Horizontal Target Distance", 0.0).getEntry();
        verticalTargetDistance = tab.add("Vertical Target Distance", 0.0).getEntry();
        verticalDirection = tab.add("Vertical Direction", 0.0).getEntry();
        rotationalSpeed = tab.add("Rotational Speed", 0.0).getEntry();
        translationalSpeed = tab.add("Translational Speed", 0.0).getEntry();
        lidarDifference = tab.add("Lidar Difference", 0.0).getEntry();
    }
    public void update(){
        isSquare.setBoolean(telemetry.isSquare(lidarTolerance));
        frontDistance.setDouble(telemetry.getFrontLidarDistance());
        rearDistance.setDouble(telemetry.getRearLidarDistance());
        backDistance.setDouble(telemetry.getBackLidarDistance());
        lidarDifference.setDouble(Math.abs(telemetry.getFrontLidarDistance() - telemetry.getRearLidarDistance()));
       // tolerance.setDouble(telemetry.getTolerance());
       // telemetry.setTolerance(tolerance.getDouble(5.0));
        telemetry.setTolerance(tolerance.getDouble(10.0));
        telemetry.setTolerance(translationalTolerance.getDouble(10.0));
        telemetry.setHorDirection(telemetry.directionToGo());
        telemetry.setHorizontalTargetDistance(horizontalTargetDistance.getDouble(10.0));
        telemetry.setVerticalDirection(telemetry.verticalDirectionToGo());
        telemetry.setVerticalTargetDistance(verticalTargetDistance.getDouble(10.0));
        telemetry.setRotationalSpeed(rotationalSpeed.getDouble(0.1));
        telemetry.setTranslationalSpeed(translationalSpeed.getDouble(0.1));
    }
}
