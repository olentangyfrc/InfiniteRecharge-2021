/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystem.balldelivery;

import java.util.logging.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.subsystem.PortMan;

/**
 * Add your docs here.
 */
public class BallDelivery extends SubsystemBase{

    private static Logger logger = Logger.getLogger(BallDelivery.class.getName());
    private TalonSRX shootingMotor1;
    private TalonSRX shootingMotor2;
    private TalonFX eatingMotor;
    private TalonFX carouselMotor;

    private DigitalInput stopCarousel;
    private DigitalInput zeroShooter;

    private double pValue;
    private double iValue;
    private double dValue;

    private double carouselVelocity;
    private double eatingVelocity;
    private double shootingVelocity;
    
    public void init(final PortMan portMan) throws Exception {
        logger.info("init");
        motor = new TalonFX(portMan.acquirePort(PortMan.can_28_label, "BallDelivery")); // change port

        pValue = .8;
        iValue = 0;
        dValue = .002;
        carouselVelocity = 10790; //don't know if this value is right
        eatingVelocity = 10790;
        shootingVelocity = 10790;

      /*motor.setNeutralMode(NeutralMode.Coast);
      motor.configFactoryDefault();
      motor.configAllowableClosedloopError(0, 5);
      motor.setSelectedSensorPosition(0, 0, 0);

      motor.config_kP(0, pValue, 0);
      motor.config_kI(0, iValue, 0);
      motor.config_kD(0, dValue, 0);
      motor.config_kF(0, 0, 0);


      motor.configClosedloopRamp(.9);
        */
    }
    
    //spin the carousel
    public void spinCarousel(double vel){
        //boolean stop = false; 
        carouselVelocity = vel;

        logger.info("spin carousel");
        logger.info("spin [" + carouselVelocity + "]");

        //spin carousel
        carouselMotor.set(ControlMode.Velocity, carouselVelocity);
        logger.info("[" + carouselVelocity + "]");

        // if switch is triggered, set percent output to 0 to stop spinning
        if(!stopCarousel.get() && output >= 0)
        {
            //output = 0;
            logger.info("stop carousel");
            carouselMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    //angle the shooter
    public void angleShooter(double pos){
        logger.info("angle shooter");
        logger.info("angle [" + pos + "]");
        
        //set off switch to start (be at bottom)
        //start motors, stop when it's in the right position
        //look at position mode -- fx might not have it 
    }

    public void eatBall(double vel){
        eatingVelocity = vel;

        logger.info("eat ball");
        logger.info("spin green wheels [" + eatingVelocity + "]");
        eatingMotor.set(ControlMode.Velocity, eatingVelocity);
        logger.info("[" + eatingVelocity + "]");
    }

    public void spitOut(double vel){
        eatingVelocity = vel; 

        logger.info("spit out ball");
        logger.info("spin green wheels [" + eatingVelocity + "]");
        eatingMotor.set(ControlMode.Velocity, eatingVelocity);
        logger.info("[" + eatingVelocity + "]");
    }

    public void stopEating(){
        //stop eating
        logger.info("stop eating");
        eatingMotor.set(ControlMode.PercentOutput, 0);
    }

    public void shootBall(double vel){
        shootingVelocity = vel; 

        logger.info("shoot ball");
        logger.info("shoot ball [" + shootingVelocity + "]");
        shootingMotor1.set(ControlMode.Velocity, shootingVelocity);
        logger.info("[" + shootingVelocity + "]" );
        shootingMotor2.follow(shootingMotor1);
        shootingMotor1.setInverted(true);
        shootingMotor2.setInverted(false);
    }

    public void reverseShooter(double vel){
        //enter regular positive velocity, method will spin it backwards. can also assume velocity parameter is negative
        shootingVelocity = -vel; 

        logger.info("reverse shooter");
        logger.info("reverse shooter [" + shootingVelocity + "]");
        shootingMotor1.set(ControlMode.Velocity, shootingVelocity);
        logger.info("[" + shootingVelocity + "]");
        shootingMotor2.follow(shootingMotor1);
        shootingMotor1.setInverted(true);
        shootingMotor2.setInverted(false);
    }

    public void stopShooting(){
        //stop the shooter
        logger.info("stop shooting");
        shootingMotor1.set(ControlMode.PercentOutput, 0);
        shooterMotor2.follow(shootingMotor1);
    }

    public void changePID(double p, double i, double d){
        if(pValue != p)
            pValue = p;
        if(iValue != i)
            iValue = i;
        if(dValue != d)
            dValue = d;
    }

    public double getCurrentCarouselVelocity(){
        return carouselMotor.getSelectedSensorVelocity();
    }

    public double getCurrentEatingVelocity(){
        return eatingMotor.getSelectedSensorVelocity();
    }

    public double getCurrentShootingVelocity(){
        return shootingMotor1.getSelectedSensorVelocity();
    }

    public double getCarouselVelocity(){
        return carouselVelocity;
    }

    public double getEatingVelocity(){
        return eatingVelocity;
    }

    public double getShootingVelocity(){
        return shootingVelocity;
    }

    public double getPValue(){
        return pValue;
    }

    public double getIValue(){
        return iValue;
    }

    public double getDValue(){
        return dValue;
    }

    public double getCurrent(){
        return motor.getSupplyCurrent();
    }
}
