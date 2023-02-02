package RoboRaiders.PID;

//import RoboRaiders.Logger.Logger;

public class RoboRaidersPID {


    //public double Kp = 0.001;
    //public double Ki = 0.0;
    //public double Kd = 0.0;
    //public double Kd = 0.00009;
    public double error;
    public double integral;
    public double derivative;
    public double previous_error;
    public double power;
    private double kp,ki,kd;

    private double currentTime;
    private double previous_time = 0;
    private double timeChange;

    public RoboRaidersPID (double Kp, double Ki, double Kd) {
        this.setCoeffecients(Kp, Ki, Kd);
    }



    public void setCoeffecients(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }




    public double CalculatePIDPowers(double Target, double currentValueFromSensor) {

        //Target = Math.abs(Target);
        //currentValueFromSensor = Math.abs(currentValueFromSensor);



        currentTime = (double)System.currentTimeMillis();
        timeChange = currentTime - previous_time;

        error = (Target) - (currentValueFromSensor);
        integral = integral + (error * timeChange);
        derivative = (error - previous_error) / timeChange;
        previous_error = error;
        power = (kp * error) + (ki * integral) + (kd * derivative);

        previous_time = (double) System.currentTimeMillis();
        return power;
    }



    public double pidWithCounts(double Target, double Sensor) {

    //    Logger myLogger =  new Logger("RRpid");
        currentTime = (double)System.currentTimeMillis();
        timeChange = currentTime - previous_time;

        error = (Target) - (Sensor);
        integral = integral + (error * timeChange);

        if (error == 0) {
            integral = 0;
        }
        // if (Math.abs(error) > 100) {
        //integral = 0;
        //}
        derivative = (error - previous_error) / timeChange;
        previous_error = error;
        power = (kp * error) + (ki * integral) + (kd * derivative);

        previous_time = (double) System.currentTimeMillis();
     //   myLogger.Debug("pwc", error, integral, derivative);
        return power;


    }


    /**
     * Will re-initialize variables
     */
    public void initialize(){
        timeChange = 0.0;
        error = 0.0;
        integral = 0.0;
        derivative = 0.0;
        previous_error = 0.0;
        power = 0.0;
        previous_time = 0.0;

    }


    public double pidWithDistance (double Sensor, double Target){
        currentTime = System.currentTimeMillis();
        timeChange = (currentTime - previous_time);

        error = (Sensor) - (Target);
        integral = integral + (error * timeChange);

        if (error == 0) {
            integral = 0;
        }
        if (Math.abs(error) > 100) {
            integral = 0;
        }
        derivative = (error - previous_error) / timeChange;
        previous_error = error;
        power = kp * error + ki * integral + kd * derivative;


        return power;
    }
    public double pidwithimuturning (double power) {
        currentTime = System.currentTimeMillis();
        timeChange = integral + (error * timeChange);


        power = kp * error + ki * integral + kd * derivative;
        previous_time = (double) System.currentTimeMillis();
        return power;
    }
}

