package RoboRaiders.Utilities.RRStopWatch;

// RoboRaider StopWatch - its a stop watch for the RoboRaiders...
public class RRStopWatch {
    private long start;

    /**
     * startTime - starts the timer
     */
    public void startTime() {
        this.start = System.nanoTime();
    }

    /**
     * getElaspedTime - returns the elasped time since the RR stop watch was started
     * @return the elasped time
     */
    public double getElaspedTime() {
        return ((double)System.nanoTime() - (double)this.start)/1000000000.0; // convert to seconds from billionths of a second
    }
}
