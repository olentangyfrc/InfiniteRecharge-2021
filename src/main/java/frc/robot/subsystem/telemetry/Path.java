package frc.robot.subsystem.telemetry;

import java.util.ArrayList;

public class Path {
    private ArrayList<Position> robotPath = new ArrayList<Position>;

    public void addPosition(Position pos){
        robotPath.add(pos);
    }

    public Position getPosition(int index){
        return robotPath.get(index);
    }
}
