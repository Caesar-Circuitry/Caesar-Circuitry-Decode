package org.firstinspires.ftc.teamcode.Config.Utils;

public class FlywheelKinematics {
    double targetVelocity = 0;
    double hgoal = 0;
    double hbot = 0;
    double xlaunch = 0;
    double LaunchAngle = 45;
    final double gravity = 9.81;

    public FlywheelKinematics(double hgoal, double hbot, double xlaunch, double LaunchAngle) {
        this.hgoal = hgoal;
        this.hbot = hbot;
        this.xlaunch = xlaunch;
        this.LaunchAngle = LaunchAngle;
    }
    public void setXlaunch(double x){
        this.xlaunch = x;
    }
    public double calculateLaunchVelocity(){
        targetVelocity = Math.sqrt((gravity * Math.pow(xlaunch, 2)) / ((2 * Math.pow(Math.cos(LaunchAngle),2)) * (xlaunch * Math.tan(LaunchAngle) - (hgoal - hbot))));
        return targetVelocity;
    }
}
