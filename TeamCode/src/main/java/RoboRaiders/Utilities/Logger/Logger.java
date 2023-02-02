package RoboRaiders.Utilities.Logger;

import android.util.Log;

/**
 * Logger is a wrapper class that wraps the android.util.Log class.
 */
public class Logger {


    private String tag;

    /**
     * constructor for Logger
     * @param tag the tag to prepend to the log message
     */

    public Logger(String tag) {this.tag = tag;}


    // Debug
    public void Debug(String str)              { Log.d(tag,str); }

    public void Debug(String str, String str1)  { Log.d(tag,str+str1); }

    public void Debug(String str, int num)     { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, double num)  { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, double num, double num1)  {
        Log.d(tag,str+String.valueOf(num)+","+String.valueOf(num1)); }

    public void Debug(String str, double num, double num1, double num2)  {
        Log.d(tag,str+String.valueOf(num)+","+String.valueOf(num1)+","+String.valueOf(num2)); }

    public void Debug(String str, double num, double num1, double num2, double num3)  {
        Log.d(tag,str+String.valueOf(num)+","+String.valueOf(num1)+","+String.valueOf(num2)+","+String.valueOf(num3)); }

    public void Debug(String str, float num)   { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, long num)    { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, boolean num) { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, char num)    { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, short num)   { Log.d(tag,str+String.valueOf(num)); }

    public void Debug(String str, byte num)    { Log.d(tag,str+String.valueOf(num)); }


    // Error
    public void Error(String str)              { Log.e(tag,str); }

    public void Error(String str, int num)     { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, double num)  { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, float num)   { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, long num)    { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, boolean num) { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, char num)    { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, short num)   { Log.e(tag,str+String.valueOf(num)); }

    public void Error(String str, byte num)    { Log.e(tag,str+String.valueOf(num)); }


    // Info
    public void Info(String str)              { Log.i(tag,str); }

    public void Info(String str, int num)     { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, double num)  { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, float num)   { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, long num)    { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, boolean num) { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, char num)    { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, short num)   { Log.i(tag,str+String.valueOf(num)); }

    public void Info(String str, byte num)    { Log.i(tag,str+String.valueOf(num)); }


    // Verbose
    public void Verbose(String str)              { Log.v(tag,str); }

    public void Verbose(String str, int num)     { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, double num)  { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, float num)   { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, long num)    { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, boolean num) { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, char num)    { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, short num)   { Log.v(tag,str+String.valueOf(num)); }

    public void Verbose(String str, byte num)    { Log.v(tag,str+String.valueOf(num)); }


    // Warn
    public void Warn(String str)              { Log.i(tag,str); }

    public void Warn(String str, int num)     { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, double num)  { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, float num)   { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, long num)    { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, boolean num) { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, char num)    { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, short num)   { Log.w(tag,str+String.valueOf(num)); }

    public void Warn(String str, byte num)    { Log.w(tag,str+String.valueOf(num)); }

}
