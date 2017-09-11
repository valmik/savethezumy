/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo;

import java.util.ArrayList;
import java.util.Scanner;

/**
 * The active calibration
 * @author Sam Skuce
 */
public class Calibration 
{
    private int       ActiveCalibration   = 0;               /** active calibration*/
    private int       ActiveTransducer    = 0;               /** active transducer*/
    private String    SerialNumber        = "";              /** serial number. */
    private String    PartNumber          = "";              /** part number of a calibration. */
    private String    CalibrationDate;                       /** calibration date. */
    private float[][] Matrix              = null;            /** calibration matrix. */
    private int  [][] GainOffset          = null;            /** gain and offset data matrix. */
    private String    ForceUnits          = "Force Counts";  /** force units. */
    private String    TorqueUnits         = "Torque Counts"; /** torque units. */
    private int       CountsPerUnitForce  = 1;               /** counts per unit force. */
    private int       CountsPerUnitTorque = 1;               /** counts per unit torque. */
    private String[]  MaxRatings;                            // The max ratings of each Gage
    
    /** 
     * Fills a calibration structure from values in the response to the "CAL"
     * console command of the Wireless F/T.
     * @param calCommandResponse The response to the "CAL" command from the
     * Wireless F/T.
     * @return A calibration structure initialized based on the fields in the
     * response.
     */
    public static Calibration parseCalibrationFromTelnetResponse(String calCommandResponse) 
    {
        Calibration cal       = new Calibration();
        String[]    lines     = calCommandResponse.split("\r\n"); /* The individual lines. */
        String[]    splitLine = lines[3].trim().split("\\s+");    /* The first matrix line split at whitespace. */
        
        cal.ActiveTransducer  = Integer.parseInt(splitLine[0]);
        cal.ActiveCalibration = Integer.parseInt(splitLine[1]);
        cal.SerialNumber      = getFieldValue(calCommandResponse, "Serial: ", "\r\n");
        cal.CalibrationDate   = getFieldValue(calCommandResponse, "Date:   ", "\r\n");
        cal.PartNumber        = getFieldValue(calCommandResponse, "Part:   ", "\r\n");
        
        // Parse matrix info.
        final int numAxes   = 6;
        final int numGauges = 6;
        cal.Matrix          = new float[numAxes][];
        cal.MaxRatings      = new String[numAxes];
        String[] axisNames  = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
        
        for (int axis = 0; axis < numAxes; axis++)
        {
            cal.Matrix[axis] = new float[numGauges];
            
            // The line containing the gage coefficients for this axis.
            String   axisText         = getFieldValue( calCommandResponse, axisNames[axis], "\r\n").trim();
            String[] axisCoefficients = axisText.split("\\s+");
            
            if (axisCoefficients.length < numGauges)
            {
                throw new IllegalArgumentException("Could not parse all gage coefficients for axis from command response.");
            }
            
            for (int gage = 0; gage < numGauges; gage++)
            {
                cal.Matrix[axis][gage] = Float.parseFloat(axisCoefficients[gage]);
            }
        }
        
        // Parse gain and offset info
        cal.GainOffset      = new int[numAxes][2];
        int numLinesConsole = 11;                
        
        String[]                     consoleLines     = new String[numLinesConsole];
        ArrayList<ArrayList<String>> consoleLinesList = new ArrayList<>();
        Scanner                      console          = new Scanner(calCommandResponse);
        
        for (int i = 0; i < consoleLines.length; i++) 
        {
            ArrayList<String> temp = new ArrayList<>();
            consoleLinesList.add(temp);
            consoleLines[i] = console.nextLine();
            
            for (String split : consoleLines[i].split(" ")) 
            {
                consoleLinesList.get(i).add(split);
                
                for (int k = 0; k < consoleLinesList.get(i).size(); k++) 
                {
                    if (consoleLinesList.get(i).get(k).equals(""))
                    {
                        consoleLinesList.get(i).remove(k);
                    }
                }
            }
        }
       
        for (int i = 0; i < 3; i++)  //remove header information.
        {
            consoleLinesList.remove(0);
        }
        
        int gainGrab   = 2;
        int offsetGrab = 3;
        
        for (int i = 0; i < cal.GainOffset.length; i++) 
        {
            cal.GainOffset[i][0] = Integer.valueOf(consoleLinesList.get(i).get(gainGrab));
            cal.GainOffset[i][1] = Integer.valueOf(consoleLinesList.get(i).get(offsetGrab));
        }
        
        // The force and torque counts/unit string.
        String   countsAndUnits;
        String[] splitCountsAndUnits;
        
        countsAndUnits          = getFieldValue(calCommandResponse, "Force:", "\r\n").trim();
        splitCountsAndUnits     = countsAndUnits.split("counts/");
        cal.CountsPerUnitForce  = Integer.parseInt(splitCountsAndUnits[0].trim());  
        cal.ForceUnits          = splitCountsAndUnits.length > 1 ? splitCountsAndUnits[1].trim() : ""; // Don't check if there's no units set.
        
        countsAndUnits          = getFieldValue(calCommandResponse, "Torque:", "\r\n").trim();
        splitCountsAndUnits     = countsAndUnits.split("counts/");
        cal.CountsPerUnitTorque = Integer.parseInt(splitCountsAndUnits[0].trim());
        cal.TorqueUnits         = splitCountsAndUnits.length > 1 ? splitCountsAndUnits[1].trim() : ""; // Don't check if there's no units set.
        
        // Get all six max ratings
        for (int i = 0; i < lines.length; i++) // Find the line that says "MaxRatings"
        {
            String line = lines[i];
            
            if (line.contains("MaxRatings"))
            {
                int n;
                String[] tokens = lines[i + 1].split("\\s+"); // Divide the next line into tokens
                n = tokens.length - 3;
                n = Math.max(n, 0);
                n = Math.min(n, 6);
                
                for (int j = 0; j < n; j++)
                {
                    cal.MaxRatings[j] = tokens[j + 3];
                }
            }
        }
        
        return cal;
    }
    
    // Note: Float.max does not exist in Java 7.
    private double max(double a, double b)
    {
        return a > b ? a : b;
    }
    
    public String getMaxRangeForce()
    {
        double max;
        max =     Double.parseDouble(this.MaxRatings[0]);
        max = max(Double.parseDouble(this.MaxRatings[1]), max);
        max = max(Double.parseDouble(this.MaxRatings[2]), max);
        return Double.toString(max);
    }
    
    public String getMaxRangeTorque()
    {
        double max;
        max =     Double.parseDouble(this.MaxRatings[3]);
        max = max(Double.parseDouble(this.MaxRatings[4]), max);
        max = max(Double.parseDouble(this.MaxRatings[5]), max);
        return Double.toString(max);
    }
    
    private static String getFieldValue(String commandResponse, String beginMarker, String endMarker ) 
    {
        int startIndex = commandResponse.indexOf(beginMarker) + beginMarker.length();
        int endIndex   = commandResponse.indexOf(endMarker, startIndex);
        
        if (startIndex == -1 || endIndex == -1)
        {
            throw new IllegalArgumentException("Field begin or end not found in command response.");
        }
        return commandResponse.substring(startIndex, endIndex);
    }
    
    public String getSerialNumber() {
        return SerialNumber;
    }

    public void setSerialNumber(String SerialNumber) {
        this.SerialNumber = SerialNumber;
    }

    public String getPartNumber() {
        return PartNumber;
    }

    public void setPartNumber(String PartNumber) {
        this.PartNumber = PartNumber;
    }

    public String getCalibrationDate() {
        return CalibrationDate;
    }

    public void setCalibrationDate(String CalibrationDate) {
        this.CalibrationDate = CalibrationDate;
    }
    
    public int getActiveCalibration() {
        return ActiveCalibration;
    }
    
    public int getActiveTransducer() {
        return ActiveTransducer;
    }

    public float[][] getMatrix() {
        return Matrix;
    }

    public void setMatrix(float[][] Matrix) {
        this.Matrix = Matrix;
    }

    public String getForceUnits() {
        return ForceUnits;
    }

    public void setForceUnits(String ForceUnits) {
        this.ForceUnits = ForceUnits;
    }

    public String getTorqueUnits() {
        return TorqueUnits;
    }

    public void setTorqueUnits(String TorqueUnits) {
        this.TorqueUnits = TorqueUnits;
    }

    public int getCountsPerUnitForce() {
        return CountsPerUnitForce;
    }

    public void setCountsPerUnitForce(int CountsPerUnitForce) {
        this.CountsPerUnitForce = CountsPerUnitForce;
    }

    public int getCountsPerUnitTorque() {
        return CountsPerUnitTorque;
    }

    public void setCountsPerUnitTorque(int CountsPerUnitTorque) {
        this.CountsPerUnitTorque = CountsPerUnitTorque;
    }
    
    public int[][] getGainOffset() {
        return GainOffset;
    }

    public void setCountsPerUnitTorque(int[][] GainOffset) {
        this.GainOffset = GainOffset;
    }
    
    public double[] getForceTorqueConversionFactors(String outputForceUnits, String outputTorqueUnits, int transducer) throws IllegalArgumentException
    {        
        String[] forceUnits        = {"", "lbf",    "klbf",     "n",       "kn",          "g",        "kg",         "mv"}; /* The supported force  units. */
        String[] torqueUnits       = {"", "lbf-in", "lbf-ft",   "n-m",     "n-mm",        "kg-cm",    "kn-m",       "mv"}; /* The supported torque units. */
        
        double[] forceConversions  = {1.0, 1.0,      0.001,      4.44822,     0.004448222, 453.5924,   0.4535924,    1.0}; /* Conversion from lbf    to each element in forceUnits.  */
        double[] torqueConversions = {1.0, 1.0,      0.08333333, 0.1129848, 112.9848,        1.152128, 0.0001129848, 1.0}; /* Conversion from lbf-in to each element in torqueUnits. */
        
        // It might be better to just throw an Exception here if they try to convert to/from voltage.
        
        String reqForceUnits  = getForceUnits () .toLowerCase(); /* For case-insensitive searching. */
        String reqTorqueUnits = getTorqueUnits() .toLowerCase();
        
        outputForceUnits      = outputForceUnits .toLowerCase();
        outputTorqueUnits     = outputTorqueUnits.toLowerCase();
        
        int i;                          /* generic loop/array index. */
        int calibrationForceIndex = -1; /* The index of the calibration force units in forceUnits. */
        int userForceIndex        = -1; /* The index of the requested   force units in forceUnits. */
        
        for (i = 0; i < forceUnits.length; i++)
        {
            if (forceUnits[i].equals(reqForceUnits))
            {
                calibrationForceIndex = i;                
            }
            
            if (forceUnits[i].equals(outputForceUnits))
            {
                userForceIndex = i;
            }
        }
        
        if (calibrationForceIndex == -1)
        {
            throw new IllegalArgumentException("Calibration force unit '" + reqForceUnits    + "' on Transducer " + (transducer + 1) + " is not a supported unit."); // Transducer is origin 1 to the user
        }
        
        if (userForceIndex == -1)
        {
            throw new IllegalArgumentException("Requested force unit '"   + outputForceUnits + "' on Transducer " + (transducer + 1) + " is not a supported unit."); // Transducer is origin 1 to the user
        }
        
        int calibrationTorqueIndex = -1; /* The index of the calibration torque units in torqueUnits. */
        int userTorqueIndex        = -1;
        
        for (i = 0; i < torqueUnits.length; i++)
        {
            if (torqueUnits[i].equals(reqTorqueUnits))
            {
                calibrationTorqueIndex = i;
            }
            
            if (torqueUnits[i].equals(outputTorqueUnits))
            {
                userTorqueIndex = i;
            }
        }
        
        if (calibrationTorqueIndex == -1)
        {
            throw new IllegalArgumentException( "Calibration torque unit '" + reqTorqueUnits    + "' on Transducer " + (transducer + 1) + " is not a supported unit." ); // Transducer is origin 1 to the user
        }
        
        if (userTorqueIndex == -1)
        {
            throw new IllegalArgumentException( "Requested torque unit '"   + outputTorqueUnits + "' on Transducer " + (transducer + 1) + " is not a supported unit." ); // Transducer is origin 1 to the user
        }
        
        double forceFactor  = forceConversions [userForceIndex]  / forceConversions [calibrationForceIndex];
        double torqueFactor = torqueConversions[userTorqueIndex] / torqueConversions[calibrationTorqueIndex];
        return new double[] { forceFactor, forceFactor, forceFactor, torqueFactor, torqueFactor, torqueFactor };
    }
}