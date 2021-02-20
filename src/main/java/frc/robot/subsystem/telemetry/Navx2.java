package frc.robot.subsystem.telemetry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;

import frc.robot.subsystem.PortMan;

public class Navx2 extends TimedRobot{
    AHRS ahrs;

    public void init(PortMan portMan) throws Exception{
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        }
        catch(RuntimeException ex){
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }    
    }
    
    public double getAngle()
    {
        return ahrs.getAngle();
    }
}
