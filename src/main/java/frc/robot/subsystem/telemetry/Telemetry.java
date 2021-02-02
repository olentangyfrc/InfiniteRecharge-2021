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
    
    private LidarPWM frontLidar, rearLidar;
    private double frontLidarDistance, rearLidarDistance;
    private double frontLidarOffset = 0;
    private double rearLidarOffset = 0;

    private static Logger logger = Logger.getLogger(Telemetry.class.getName());

    private double betweenLidarDistance = 0;
    private double lidarTolerance = 10.0;
    private double correction = Math.PI/180;
    private MedianFilter filterFront;
    private MedianFilter filterRear;

    //targetDistance is the distance away from the wall
    private double targetDistance = 100;

    public Telemetry() {
    }
    
    public void init(PortMan portMan) throws Exception{
        logger.entering(Telemetry.class.getName(), "init()");

        frontLidar = new LidarPWM(portMan.acquirePort(PortMan.digital5_label, "Telemetry.frontLidar"));
        rearLidar = new LidarPWM(portMan.acquirePort(PortMan.digital1_label, "Telemetry.rearLidar"));
        filterFront = new MedianFilter(10);
        filterRear = new MedianFilter(10);

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
            if (frontLidarDistance > rearLidarDistance + lidarTolerance){
                //rotate left
                return -1;
            } else if(frontLidarDistance < rearLidarDistance - lidarTolerance){
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
        if(frontLidarDistance > frontLidarDistance - targetDistance + lidarTolerance){
            //go left
            return -1;
        }
        else if(frontLidarDistance < frontLidarDistance + targetDistance - lidarTolerance){
            //go right
            return 1;
        }
        else {
            //already at target
            return 0;
        }
    }
    
    /*
    //breaks lidar on shuffleboard
    public boolean isSquare(double targetDistance)
    {
        frontLidarDistance = frontLidar.getDistance();
        rearLidarDistance = rearLidar.getDistance();

        if (Math.abs(frontLidarDistance-targetDistance) > lidarTolerance || Math.abs(rearLidarDistance-targetDistance) > lidarTolerance || Math.abs(frontLidarDistance-rearLidarDistance) > lidarTolerance)
        {
            double angleError = Math.atan((Math.max(frontLidarDistance, rearLidarDistance)-Math.min(frontLidarDistance, rearLidarDistance))/betweenLidarDistance);

            if (frontLidarDistance*Math.cos(angleError)-targetDistance > rearLidarDistance*Math.cos(angleError)-targetDistance)
            {
                if (frontLidarDistance < rearLidarDistance)
                {
                    //move front wheels right angleError, turn right
                }
                else
                {
                    //move front wheels left angleError, turn left
                }
            }
            else
            {
                if (frontLidarDistance < rearLidarDistance)
                {
                    //move back wheels left angleError, turn right
                }
                else
                {
                    //move back wheels right angleError, turn left
                }
            }

            while(Math.abs(frontLidarDistance-rearLidarDistance) > lidarTolerance)
            {
                if (frontLidarDistance*Math.cos(angleError)-targetDistance > rearLidarDistance*Math.cos(angleError)-targetDistance)
                {
                    if (frontLidarDistance < rearLidarDistance)
                    {
                        //move front wheels right correction, turn right
                    }
                    else
                    {
                        //move front wheels left correction, turn left
                    }
                
                }
                else
                {
                    if (frontLidarDistance < rearLidarDistance)
                    {
                        //move back wheels left correction, turn right
                    }
                    else
                    {
                        //move back wheels right correction, turn left
                    }
                }
            }
            
            double distanceError = Math.abs(frontLidarDistance - targetDistance);

            if (distanceError > lidarTolerance)
            {
                if (frontLidarDistance > targetDistance)
                {
                    //move left distanceError
                }
                else
                {
                    //move right distanceError
                }
            }
        }
        return true;
    }
    */

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

    /*public boolean isSquare()
    {
        return whereAmI(100);
    }
    */
    
    public double getTolerance(){
        return lidarTolerance;
    }

    public void setTolerance(double tol){
        lidarTolerance = tol;
    }

    public double getBetweenLidar(){
        return betweenLidarDistance;
    }
}