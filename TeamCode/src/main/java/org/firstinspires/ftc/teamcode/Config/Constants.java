package org.firstinspires.ftc.teamcode.Config;

import com.bylazar.configurables.annotations.Configurable;

public class Constants {
//    public static class insertSubsystemName {
        // put subsystem specific constants here
        // example: public static final int MOTOR_PORT = 0;
//    }
    @Configurable
    public static class Launcher{
        public static double Kv = 0; //feed forward velocity
        public static double Ks = 0; //feed forward speed
        public static double Ka = 0; //feed forward acceleration

        public static double A = 0; //accel for feedforwardTune

        public static double V = 0; //vel for feedworwardTune
}
}
