package frc.robot.logging;

import java.util.Hashtable;
import java.util.Calendar;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotController;
import java.io.*;

import frc.robot.Utils;
import frc.robot.controllers.PID;

// FILE HAS NOT BEEN CLEANED UP //
public class Logger{

    public enum DefaultValue {Previous, Empty} // If in a given cycle a value isn't set, what should that value be in the next row? Empty : "", Previous : the same as the previous value
    public enum CommandStatus {Initializing, Executing, Ending, Interrupted}
    public final static String LOGGING_FILE_PATH = "/home/lvuser/MainLog.csv"; // Path to file where data is logged
    public Hashtable<String,Object> currentData; // Data of the current cycle
    public Hashtable<String,DefaultValue> defaultValues; // Data of the current default values for each column
    // TODO: make default values not reset every deploy
    private CSVHelper csvHelper;
    private Calendar calendar;
    private boolean loggingDisabled = false;
    private Timer timer;
    private double matchStartTime;
    // Universal naming conventions
    public String commandStatusName = "status";

    public Logger(){
        this.calendar = Calendar.getInstance();
        try{                
            this.csvHelper = new CSVHelper(LOGGING_FILE_PATH);
        }catch (Exception E){
            try{
                (new File(LOGGING_FILE_PATH)).createNewFile();
                this.csvHelper = new CSVHelper(LOGGING_FILE_PATH);
            }catch (Exception E2){
                fileNotFound();
            }
        }
        resetCurrentData();
        this.defaultValues = new Hashtable<String,DefaultValue>();
        this.timer = new Timer();
        this.timer.start();
        this.startMatchTimer();
    }

    public String getCallerClassName() { 
        StackTraceElement[] elements = Thread.currentThread().getStackTrace();
        for (int i = 1; i < elements.length; i++) {
            StackTraceElement element = elements[i];
            if (!element.getClassName().equals(Logger.class.getName()) && !element.getClassName().contains("java.lang.Thread")) {
                return element.getClassName() + ".java";
            }
        }
        // At this point, this function is not called beyond this class or some other error occurred
        String trace = "";
        for (StackTraceElement i : elements) {
            trace += i.toString() + "\n";
        }
        throw new RuntimeException("Unable to determine calling class or only Logger calls this function:\n" + trace);
    }

    private void fileNotFound(){
        System.out.println("FILE NOT FOUND (logger)");
    }

    private void resetCurrentData(){
        this.currentData = new Hashtable<String,Object>();
    }

    public String getColumnName(String valueName){
        return getColumnName(getCallerClassName(), valueName);
    }
    public String getColumnName(String filename, String valueName){
        // This is simply how we name the columns. That way, everything is organized by file and differnet files can have data of the same name
        return filename + ": " + valueName;
    }

    public ArrayList<String> getColumns(){
        return this.csvHelper.getTopics();
        // return new ArrayList<String>();
    }

    private void addNewColumn(String columnName){
        // adds a new column to the file and records it in the column hashset
        this.csvHelper.addTopic(columnName);
    }
    public void newDataPoint(String valueName){
        newDataPoint(getCallerClassName(), valueName);
    }
    public void newDataPoint(String filename, String valueName){
        if (this.loggingDisabled){return;}
        // Same as add new column but is what should generally be called directly
        addNewColumn(getColumnName(filename, valueName));
    }

    public void addData(String valueName, Object data, DefaultValue defaultValue){
        addData(getCallerClassName(), valueName, data, defaultValue);
    }
    public void addData(String filename, String valueName, Object data, DefaultValue defaultValue){
        // Adds a singular piece of data to the currentData hash. Also will add the column if it is unrecognized
        if (this.loggingDisabled){return;}
        String columnName = getColumnName(filename, valueName);
        if (!columnExists(columnName)) { 
            //System.out.println("adding column " + columnName);
            addNewColumn(columnName);
        }
        this.defaultValues.put(columnName, defaultValue);
        this.currentData.put(columnName, data);
    }
    public void logFinalField(String fieldName, Object fieldValue){
        logFinalField(getCallerClassName(), fieldName, fieldValue);
    }
    public void logFinalField(String filename, String fieldName, Object fieldValue){
        addData(filename, fieldName, fieldValue, DefaultValue.Previous);
    }
    public void logFinalPIDConstants(String pidName, PID pid){
        logFinalPIDConstants(getCallerClassName(), pidName, pid);
    }
    public void logFinalPIDConstants(String filename, String pidName, PID pid){
        addData(filename, pidName, "(" + pid.getP() + " | " + pid.getI() + " | " + pid.getD() + ")", DefaultValue.Previous);
    }
    public void logCommandStatus(CommandStatus commandStatus){
        logCommandStatus(getCallerClassName(), commandStatus);
    }
    public void logCommandStatus(String filename, CommandStatus commandStatus){
        String stringStatus = "";
        switch (commandStatus) {
            case Initializing: stringStatus = "initializing"; break;
            case Executing:    stringStatus = "executing";    break;
            case Ending:       stringStatus = "ending";       break;
            case Interrupted:  stringStatus = "interrupted";  break;
        }
        addData(filename, commandStatusName, stringStatus, DefaultValue.Empty);
    }

    public DefaultValue getDefaultValue(String column){
        if (this.defaultValues.containsKey(column)){
            return this.defaultValues.get(column);
        } else {
            return DefaultValue.Empty;
        }
    }
    
    public Hashtable<String,String> getLastLog(){
        // Gets the most recent log (IE: row) in the logging file
        return this.csvHelper.getLastRow();
    }
    private String getLastLoggedInColumn(String columnName){
        // Given a column name, it gives the most recent value of that column as a string
        return getLastLog().get(columnName);
    }
    public String getLastValueLogged(String valueName){
        // Same as get lastLoggedInColumn but should generally be called
        return getLastValueLogged(getCallerClassName(), valueName);
    }
    public String getLastValueLogged(String filename, String valueName){
        // Same as get lastLoggedInColumn but should generally be called
        return getLastLoggedInColumn(getColumnName(filename, valueName));
    }
    public double getLastValueLoggedDouble(String valueName){
        return getLastValueLoggedDouble(getCallerClassName(), valueName);
    }
    public double getLastValueLoggedDouble(String filename, String valueName){
        // Converts last log value as a string to a double, returns 0 if it isn't a number
        // Maybe TODO - should it throw an error if the previous value is not a number of ""? I think it shouldn't but to consider
        String stringValue = getLastValueLogged(filename, valueName);
        try {
            return Double.valueOf(stringValue);
        } catch (Exception e) {
            return 0;
        }
    }
    public boolean getLastValueLoggedBool(String valueName){
        // Converts last log to a bool
        return getLastValueLoggedBool(getCallerClassName(), valueName);
    }
    public boolean getLastValueLoggedBool(String filename, String valueName){
        // Converts last log to a bool
        return Boolean.valueOf(getLastValueLogged(filename, valueName));
    }

    public boolean columnExists(String columnName){
        return getColumns().contains(columnName);
    }

    public void addToPrevious(String valueName, DefaultValue defaultValue, double incrementAmount){
        addToPrevious(getCallerClassName(), valueName, defaultValue, incrementAmount);
    }
    public void addToPrevious(String filename, String valueName, DefaultValue defaultValue, double incrementAmount){
        // Logs the next values as the incrementAmount + the bool value of the most recent logged data point
        double lastValue = getLastValueLoggedDouble(filename, valueName);
        addData(filename, valueName, lastValue + incrementAmount, defaultValue);
    }
    public void incrementPrevious(String valueName, DefaultValue defaultValue){
        incrementPrevious(getCallerClassName(), valueName, defaultValue);
    }
    public void incrementPrevious(String filename, String valueName, DefaultValue defaultValue){
        addToPrevious(filename, valueName, defaultValue, 1);
    }

    private void addDefaultData(){
        // All this data will be done automatically
        Double year   = (double) this.calendar.get(Calendar.YEAR);
        Double month  = (double) this.calendar.get(Calendar.MONTH);
        Double day    = (double) this.calendar.get(Calendar.DAY_OF_MONTH);
        Double hour   = (double) this.calendar.get(Calendar.HOUR_OF_DAY);
        Double minute = (double) this.calendar.get(Calendar.MINUTE);
        Double second = (double) this.calendar.get(Calendar.SECOND);
        double timeSinceStartup = getTimeSinceStartup();
        Double matchTime = Timer.getMatchTime();
        Double batteryVoltage = RobotController.getBatteryVoltage();
        String prefix = "default";
        addData(prefix, "year",year,                      DefaultValue.Previous);
        addData(prefix, "month",month,                    DefaultValue.Previous);
        addData(prefix, "day",day,                        DefaultValue.Previous);
        addData(prefix, "hour",hour,                      DefaultValue.Previous);
        addData(prefix, "minute",minute,                  DefaultValue.Previous);
        addData(prefix, "second",second,                  DefaultValue.Previous);
        addData(prefix, "sec startup",timeSinceStartup,   DefaultValue.Previous);
        addData(prefix, "match time",matchTime,           DefaultValue.Previous);
        addData(prefix, "battery voltage",batteryVoltage, DefaultValue.Previous);
    }

    private Hashtable<String,String> createFullCurrentData(){
        // Takes the defaultData() and the currentData to create the hash that will be given for the csvHelper to record
        addDefaultData();
        Hashtable<String,Object> fullData = new Hashtable<>();
        for(String column : getColumns()) {
            if (this.currentData.containsKey(column)) {
                fullData.put(column, this.currentData.get(column));
            } else if (getDefaultValue(column) == DefaultValue.Previous) {
                String data = getLastLoggedInColumn(column);
                fullData.put(column, data);
            }
        }
        return Utils.hatshtableDataToString(fullData);
    }

    public void logData(){
        if (this.loggingDisabled){return;}
        // adds a new data record to the file and resets our current data
        this.csvHelper.addRow(createFullCurrentData());
    }

    public void writeLoggedData(){
        this.csvHelper.writeRows();
        resetCurrentData();
    }

    public double getTimeSinceStartup(){
        return this.timer.get();
    }

    public void startMatchTimer(){
        this.matchStartTime = this.timer.get();
    }

    public double getTimeSinceMatchStart(){
        return this.timer.get() - this.matchStartTime;
    }
}
