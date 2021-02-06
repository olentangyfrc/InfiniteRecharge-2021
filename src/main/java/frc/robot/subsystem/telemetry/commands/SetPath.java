package frc.robot.subsystem.telemetry.commands;

import frc.robot.subsystem.telemetry.Position;
import java.util.ArrayList;

public class SetPath {
    ArrayList<Position> path1 = new ArrayList<Position>();
    ArrayList<Position> path2 = new ArrayList<Position>();
    ArrayList<Position> path3 = new ArrayList<Position>();
    ArrayList<ArrayList<Position>> paths = new ArrayList<ArrayList<Position>>();
    
    public void makePath1(){        
        path1.add(new Position(90, 90));
        path1.add(new Position(90, 150));
        path1.add(new Position(90, 90));
        path1.add(new Position(126, 90));
        path1.add(new Position(126, 36));
        path1.add(new Position(180, 36));
        path1.add(new Position(180, 150));
        path1.add(new Position(180, 136));
        path1.add(new Position(270, 36));
        path1.add(new Position(270, 150));
        path1.add(new Position(270, 90));

        paths.add(path1);
    }

    public void makePath2(){
        
        paths.add(path2);
    }

    public void makePath3(){

    }

    public ArrayList<Position> getPath1(){
        return path1;
    }

    public ArrayList<Position> getPath2(){
        return path2;
    }

    public ArrayList<Position> getPath3(){
        return path3;
    }
}
