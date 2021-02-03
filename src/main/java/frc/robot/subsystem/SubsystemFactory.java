package frc.robot.subsystem;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.logging.Logger;

import frc.common.drivers.Gyroscope;
import frc.robot.OI;
import frc.robot.OzoneException;
import frc.robot.subsystem.climber.Climber;
import frc.robot.subsystem.controlpanel.ControlPanel;
import frc.robot.subsystem.controlpanel.commands.RotateToColor;
import frc.robot.subsystem.controlpanel.commands.SpinManual;
import frc.robot.subsystem.controlpanel.commands.SpinRotations;
import frc.robot.subsystem.controlpanel.commands.SpinnerRetract;
import frc.robot.subsystem.controlpanel.commands.SpinnerUp;
import frc.robot.subsystem.controlpanel.commands.Stop;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.intake.commands.IntakeDown;
import frc.robot.subsystem.intake.commands.IntakeSpinBack;
import frc.robot.subsystem.intake.commands.IntakeSpinForward;
import frc.robot.subsystem.intake.commands.IntakeStop;
import frc.robot.subsystem.intake.commands.IntakeUp;
import frc.robot.subsystem.telemetry.Telemetry;
import frc.robot.subsystem.telemetry.Pigeon;
import frc.robot.subsystem.telemetry.commands.SquareSelf;
import frc.robot.subsystem.onewheelshooter.OneWheelShooter;
import frc.robot.subsystem.winch.Winch;
import frc.robot.subsystem.winch.commands.WinchUp;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelReverse;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelShoot;
import frc.robot.subsystem.onewheelshooter.commands.OneWheelStop;
import frc.robot.subsystem.pixylinecam.PixyLineCam;
import frc.robot.subsystem.pixylinecam.commands.PollPixyLine;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.climber.commands.Climb;
import frc.robot.subsystem.climber.commands.ClimberControl;
import frc.robot.subsystem.climber.commands.ClimberControlBack;
import frc.robot.subsystem.climber.commands.ClimberRetract;
import frc.robot.subsystem.commandgroups.CollectionMode;
import frc.robot.subsystem.commandgroups.ControlPanelMode;
//import frc.robot.subsystem.commandgroups.ControlPanelMode;
import frc.robot.subsystem.commandgroups.MoveMode;
import frc.robot.subsystem.commandgroups.ScoreLowMode;
import frc.robot.subsystem.commandgroups.SpitBallsMode;
import frc.robot.subsystem.commandgroups.ScoreHighMode;
import frc.robot.subsystem.commandgroups.StartingConfiguration;
//import frc.robot.subsystem.commandgroups.StartingConfiguration;
import frc.robot.subsystem.transport.commands.ScoreLow;
import frc.robot.subsystem.transport.Transport;
import frc.robot.subsystem.transport.commands.*;
import frc.robot.subsystem.transport.commands.TakeIn;
import frc.robot.subsystem.transport.commands.StopTransport;
import frc.robot.subsystem.swerve.DrivetrainSubsystem2910;

public class SubsystemFactory {

    private static SubsystemFactory me;

    static Logger logger = Logger.getLogger(SubsystemFactory.class.getName());

    private static DisplayManager displayManager;

    private PowerDistributionPanel pdp;

    /**
     * keep all available subsystem declarations here.
     */

    private Transport transport;
    private ControlPanel controlPanel;
    private Climber climber;
    private OneWheelShooter oneWheelShooter;
    private Telemetry telemetry;
    private PixyLineCam pixyLineCam;
    private DrivetrainSubsystem2910 driveTrain;
    private Intake intake;
    private Winch winch;
    private Pigeon pigeon;
    
    private static ArrayList<SBInterface> subsystemInterfaceList;

    private SubsystemFactory() {
    }

    public static SubsystemFactory getInstance() {

        if (me == null) {
            me = new SubsystemFactory();
        }

        return me;
    }

    public void init(DisplayManager dm, PortMan portMan) throws Exception {

        logger.info("initializing");

        displayManager = dm;
        subsystemInterfaceList = new ArrayList<SBInterface>();
        pdp = new PowerDistributionPanel(1);

        initComp(portMan);
    }


    private void initComp(PortMan portMan) throws Exception {

        logger.info("initiatizing");

        driveTrain = DrivetrainSubsystem2910.getInstance();
        /*
        WinchUp w = new WinchUp(winch);
        OI.getInstance().bind(w, OI.RightButtonBox4, OI.WhileHeld);
        OI.getInstance().bind(w, OI.RightJoyButton11, OI.WhileHeld);
        */

        pigeon = driveTrain.getGyroscope();
        pigeon.calibrate();
        pigeon.setInverted(true);

        /**
         * All of the OneWheelShooter stuff goes here
         
        
        oneWheelShooter = new OneWheelShooter();
        oneWheelShooter.init(portMan);
        displayManager.addShooter(oneWheelShooter);
        
        OneWheelShoot sh = new OneWheelShoot(oneWheelShooter);

        OneWheelReverse owr = new OneWheelReverse(oneWheelShooter);
        OI.getInstance().bind(owr, OI.LeftJoyButton2, OI.WhenPressed);

        OneWheelStop st = new OneWheelStop(oneWheelShooter);
        OI.getInstance().bind(st, OI.LeftButtonBox10, OI.WhenPressed);
        OI.getInstance().bind(st, OI.LeftJoyButton3, OI.WhenPressed);
        */

         /**
         * All of Intake Stuff goes here
         

        intake = new Intake();
        intake.init(portMan);
        displayManager.addIntake(intake);

     
        IntakeUp iu = new IntakeUp(intake);

        IntakeDown id = new IntakeDown(intake);

        IntakeSpinForward isf = new IntakeSpinForward(intake);

        IntakeSpinBack isb = new IntakeSpinBack(intake);

        IntakeStop is = new IntakeStop(intake);
        */  


         /** * All of the Transport stuff goes here
         * 
        transport = new Transport();
        transport.init(portMan);
        displayManager.addTransport(transport);
        
        //TakeIn tc = new TakeIn(transport);

        PushOut pc = new PushOut(transport);

        SideGateOpen tu = new SideGateOpen(transport);
        OI.getInstance().bind(tu, OI.LeftJoyButton6, OI.WhenPressed);


        SideGateClose td = new SideGateClose(transport);
        OI.getInstance().bind(td, OI.LeftJoyButton7, OI.WhenPressed);


        TailGateUp tgu = new TailGateUp(transport);
        OI.getInstance().bind(tgu, OI.LeftJoyButton11, OI.WhenPressed);
        
        TailGateDown tgd = new TailGateDown(transport);
        OI.getInstance().bind(tgd, OI.LeftJoyButton10, OI.WhenPressed);


        StopTransport si = new StopTransport(transport);
        OI.getInstance().bind(si, OI.LeftButtonBox9, OI.WhenPressed);

        ScoreLow sl = new ScoreLow(transport);
        OI.getInstance().bind(sl, OI.RightJoyButton1, OI.WhenPressed);
        OI.getInstance().bind(sl, OI.RightButtonBox11, OI.WhenPressed);

        ScoreHigh sHigh = new ScoreHigh(transport);
        OI.getInstance().bind(sHigh, OI.LeftJoyButton1, OI.WhenPressed);
        OI.getInstance().bind(sHigh, OI.RightButtonBox10, OI.WhenPressed);
        
        */

         /**
         * All of the ControlPanel stuff goes here
         

        controlPanel = new ControlPanel();
        controlPanel.init(portMan, telemetry);
        displayManager.addCP(controlPanel);

        
        RotateToColor dc = new RotateToColor(controlPanel);
        OI.getInstance().bind(dc, OI.RightButtonBox2, OI.WhenPressed);

        SpinRotations ss = new SpinRotations(controlPanel, 7);
        OI.getInstance().bind(ss, OI.RightButtonBox5, OI.WhenPressed);

        SpinManual sm = new SpinManual(controlPanel);
        OI.getInstance().bind(sm, OI.LeftJoyButton4, OI.WhileHeld);

        SpinnerUp su = new SpinnerUp(controlPanel);
        SpinnerRetract sr = new SpinnerRetract(controlPanel);
        
        Stop stop = new Stop(controlPanel);
        OI.getInstance().bind(stop, OI.RightButtonBox7, OI.WhenPressed);
        */

        /**
         * All of the Climber stuff goes here
         
        climber = new Climber();
        climber.init(portMan);
        displayManager.addClimber(climber);

        
        Climb c = new Climb(climber);

        ClimberRetract cr = new ClimberRetract(climber);

        ClimberControl cc = new ClimberControl(climber);
        OI.getInstance().bind(cc, OI.RightButtonBox3,OI.WhileHeld);
        OI.getInstance().bind(cc, OI.RightJoyButton3, OI.WhileHeld);

        ClimberControlBack ccb = new ClimberControlBack(climber);
        OI.getInstance().bind(ccb,OI.RightButtonBox1,OI.WhileHeld);
        OI.getInstance().bind(ccb, OI.RightJoyButton2,OI.WhileHeld);
        */

         /**
         * All of the Telemery Stuff goes here
         */

        telemetry = new Telemetry();
        telemetry.init(portMan);
        displayManager.addTelemetry(telemetry);

        //Command Groups
        /*
        CollectionMode collectionMode = new CollectionMode(transport, intake, controlPanel, oneWheelShooter);
        OI.getInstance().bind(collectionMode, OI.LeftButtonBox1, OI.WhenPressed);
        
        StartingConfiguration startConfig = new StartingConfiguration(transport, intake, controlPanel, oneWheelShooter);
        OI.getInstance().bind(startConfig, OI.LeftButtonBox5, OI.WhenPressed);
        
        MoveMode moveMode = new MoveMode(transport, intake, controlPanel);
        OI.getInstance().bind(moveMode, OI.LeftButtonBox4, OI.WhenPressed);

        ScoreLowMode scoreLow = new ScoreLowMode(transport, intake, controlPanel);
        OI.getInstance().bind(scoreLow, OI.LeftButtonBox7, OI.WhenPressed);

        ScoreHighMode scoreHigh = new ScoreHighMode(transport, intake, controlPanel, oneWheelShooter);
        OI.getInstance().bind(scoreHigh, OI.LeftButtonBox6, OI.WhenPressed);

        ControlPanelMode controlPanelMode = new ControlPanelMode(transport, intake, controlPanel, oneWheelShooter);
        OI.getInstance().bind(controlPanelMode, OI.LeftButtonBox3, OI.WhenPressed);

        SpitBallsMode spitBallsMode = new SpitBallsMode(transport, intake);
        OI.getInstance().bind(spitBallsMode, OI.LeftButtonBox11, OI.WhenPressed);
        */
    }

    public PowerDistributionPanel getPDP(){
        return pdp;
    }
    public ControlPanel getControlPanel() {
        return controlPanel;
    }
    public DrivetrainSubsystem2910 getDriveTrain(){
        return driveTrain;
    }
    public Climber getClimber() {
        return climber;
    }

    public Transport getTransport() {
        return transport;
    }
    public Intake getIntake(){
        return intake;
    }
    public OneWheelShooter getShooter(){
        return oneWheelShooter;
    }
    public Pigeon getGyro() {
        return pigeon;
    }
}