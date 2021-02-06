package frc.robot.subsystem.telemetry;

public class Position {
    private double x;
    private double y;

    public Position(double x, double y, double robotWidth){
        this.x = x - robotWidth/2 * 2.54;
        this.y = y - robotWidth/2 * 2.54;
    }

    public void updatePosition(Position pos){
        x = pos.getx();
        y = pos.gety();
    }

    public void setx(double x){
        this.x = x;
    }

    public void sety(double y){
        this.y = y;
    }

    public double getx(){
        return x;
    }

    public double gety(){
        return y;
    }
}
