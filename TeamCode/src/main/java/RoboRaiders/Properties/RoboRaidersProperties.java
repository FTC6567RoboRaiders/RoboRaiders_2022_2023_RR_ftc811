package RoboRaiders.Properties;
public class RoboRaidersProperties {
    /** Class to contain static variables to hold data that may be needed
     * for opmodes to communicate with each other.
     *
     *   Variable Name          Description
     *   lastHeading            The last heading of the robot in raidans, initial setting is 0.0
     *   redAlliance            A boolean, that when true indicates we are on the red alliance
     *   blueAlliance           A boolean, that when true indicates we are on the blue alliance
     *
     *
     **/
    public static double lastHeading;
    public static boolean redAlliance;
    public static boolean blueAlliance;
    //Setter method
    public static void setHeading(double aHeading){
        lastHeading = aHeading;
    }
    //Getter method
    public static double getHeading(){
        return lastHeading;
    }
}
