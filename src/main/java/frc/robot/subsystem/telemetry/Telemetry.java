/**
 * two lidars
 * one boolean to see if the robot is parallel at a certain distance
 * 
 * boolean isSquare(double distance)
 * 
 */

package frc.robot.subsystem.telemetry;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystem.PortMan;
import java.util.logging.Logger;

import javax.lang.model.util.ElementScanner6;

public class Telemetry extends SubsystemBase{
    
    private LidarPWM frontLidar, rearLidar, backLidar;
    private double frontLidarDistance, rearLidarDistance, backLidarDistance;
    private double frontLidarOffset = 0;
    private double rearLidarOffset = 0;
    private double backLidarOffset = 0;

    private static Logger logger = Logger.getLogger(Telemetry.class.getName());

    private double betweenLidarDistance = 0;
    private double lidarTolerance = 10.0;
    private double correction = Math.PI/180;
    private MedianFilter filterFront;
    private MedianFilter filterRear;
    private MedianFilter filterBack;

    private int horDirection = 0;
    private int verticalDirection = 0;

    //targetDistance is the distance away from the wall
    private double horizontalTargetDistance = 100;
    private double verticalTargetDistance = 100;

    public Telemetry() {
    }
    
    public void init(PortMan portMan) throws Exception{
        logger.entering(Telemetry.class.getName(), "init()");

        frontLidar = new LidarPWM(portMan.acquirePort(PortMan.digital1_label, "Telemetry.frontLidar"));
        rearLidar = new LidarPWM(portMan.acquirePort(PortMan.digital5_label, "Telemetry.rearLidar"));
        backLidar = new LidarPWM(portMan.acquirePort(PortMan.digital6_label, "Telemetry.backLidar"));
        filterFront = new MedianFilter(10);
        filterRear = new MedianFilter(10);
        filterBack = new MedianFilter(10);

        CameraServer.getInstance().startAutomaticCapture();
        //CameraServer.getInstance().startAutomaticCapture();

        logger.exiting(Telemetry.class.getName(), "init()");
    }

    public boolean isSquare(double tolerance){
        if (Math.abs(getFrontLidarDistance() - getRearLidarDistance()) <= tolerance)
            return true;
        else
            return false;
    }

    public int whereAmI(){
    {
        //multiplies speed by the value that is returned to set direction of rotation
        frontLidarDistance = frontLidar.getDistance();
        rearLidarDistance = rearLidar.getDistance();
            if (frontLidarDistance > rearLidarDistance && !isSquare(lidarTolerance)){
                //rotate left
                return -1;
            } else if(frontLidarDistance < rearLidarDistance && !isSquare(lidarTolerance)){
                //rotate right
                return 1;
            } else {
                //already square
                return 0;
            }
        }     
    }

    public int directionToGo(){
        frontLidarDistance = frontLidar.getDistance();
        if(Math.abs(frontLidarDistance - horizontalTargetDistance) < lidarTolerance){
            //already at target
            return 0;
        }
        else if(frontLidarDistance - horizontalTargetDistance > 0){
            //go left
            return 1;
        }
        else{
            //go right
            return -1;
        }
    }

    public int verticalDirectionToGo(){
        backLidarDistance = backLidar.getDistance();
        if(Math.abs(backLidarDistance - verticalTargetDistance) < lidarTolerance){
            //already at target
            return 0;
        }
        else if(backLidarDistance - verticalTargetDistance > 0){
            //go back
            return -1;
        }
        else{
            //go forward
            return 1;
        }
    }
    

    public double getFrontLidarDistance(){
        if (frontLidar == null)
            return 0.0;
        return filterFront.calculate(frontLidar.getDistance());
    }

    public double getRearLidarDistance(){
        if (rearLidar == null)
            return 0.0;
            
        return filterRear.calculate(rearLidar.getDistance());
    }

    public double getBackLidarDistance(){
        if(backLidar == null)
            return 0.0;
        return filterBack.calculate(backLidar.getDistance());
    }

    
    public double getTolerance(){
        return lidarTolerance;
    }

    public void setTolerance(double tol){
        lidarTolerance = tol;
    }

    public double getBetweenLidar(){
        return betweenLidarDistance;
    }

    public void setHorDirection(int dir){
        horDirection = dir;
    }

    public void setHorizontalTargetDistance(double dis)
    {
        horizontalTargetDistance = dis;
    }

    public void setVerticalTargetDistance(double dist)
    {
        verticalTargetDistance = dist;
    }

    public void setVerticalDirection(int dire)
    {
        verticalDirection = dire;
    }
}