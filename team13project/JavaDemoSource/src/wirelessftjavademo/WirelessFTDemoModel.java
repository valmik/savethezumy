/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.concurrent.Task;
import javafx.concurrent.Worker;
import static wirelessftjavademo.IPSettings.BandSetting.Spectrum2_4Ghz;
import static wirelessftjavademo.IPSettings.BandSetting.Spectrum5Ghz;
import wirelessftjavademo.userinterface.WirelessFTDemoMainScreenController;
import wirelessftsensor.WirelessFTSensor;
import wirelessftsensor.WirelessFTSensor.CommSeveredException;

/**
 * Represents the connected WNet and its state.
 * @author Sam Skuce, Chris Collins
 */
public class WirelessFTDemoModel 
{
    public static String Cal1Serial;
    public static String Cal1PartNumber;
    public static String Cal1Date;
    public static String Cal1Force;
    public static String Cal1Torque;
    
    public static String Cal2Serial;
    public static String Cal2PartNumber;
    public static String Cal2Date;
    public static String Cal2Force;
    public static String Cal2Torque;
    
    public static String Cal3Serial;
    public static String Cal3PartNumber;
    public static String Cal3Date;
    public static String Cal3Force;
    public static String Cal3Torque;
    
    public static int    ActiveCalibration;
    public static int    ActiveTransducer;
    public static float[][] Matrix;
    
    public static final String PREF_USER_ROOT_NODE_PATHNAME = "com.FTDemo.preference.Settings"; // The preferences user root node pathname.
  
    public static final String PREF_LATEST_IP               = "latestIPConnectAttempt";         // The latest IP setting, a preference keyword.

    public              int m_udpRate        = -1; // The current UDP packet transfer rate.

    public static final int MAX_SENSORS      =  6; // The maximum number of Transducers in a WNet.
    
    public static final int MAX_CALIBRATIONS =  3; // The maximum number of calibrations each sensor may have.
    
    public static final int NUM_AXES         =  6; // The number of gages on a sensor.
    
    /**
     * Keeps track of active sensors.
     */
    public boolean m_sensorActive[] = {false, false, false, false, false, false};

    /**
     * Indicates that we are reading streaming data.
     */
    private final SimpleObjectProperty<Boolean> m_flagReadStreaming = new SimpleObjectProperty<>(false);
    
    /**
     * Indicates that a file upload is complete (typically firmware upgrades).
     */
    public SimpleObjectProperty<Boolean> m_flagFileUploadComplete = new SimpleObjectProperty<>(false);
    
    /**
     * The WNet's IP Address.
     */
    private String m_sensorAddressOrHostName;
    
    /**
     * Used to communicate with the WNet over Telnet/UDP.
     */
    public WirelessFTSensor m_sensor = null;
    
    /**
     * An error log for the application.
     */
    private static final Logger m_logger = Logger.getLogger("wirelessft");

    private WirelessFTDemoMainScreenController m_controller = null; // 

    /** 
     * The progress of the file/firmware upload.
     */
    private final SimpleObjectProperty<Double> m_fileUploadProgress = new SimpleObjectProperty<>(0.0);
    
    /**
     * Gets the percentage progress of the latest file upload.
     * 
     * @return The percentage 0-100 of the file upload process.
     */
    public ObservableValue<Double> fileUploadProgress() 
    {
        return m_fileUploadProgress;
    }
    
    /**
     * Sets the UDP Transfer rate for this model.
     *
     * @param rate How many ADC reads are between each sent packet.
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public void applyRate(int rate) throws CommSeveredException 
    {
        m_sensor.sendTelnetCommand("RATE " + rate + " 1", true);
    }
    
/**
     * Gets whether or not F/T records will also
     * be written to the MicroSD card.
     * 
     * @return True if the MicroSD is recording, false otherwise.
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public boolean getSDRecording() throws CommSeveredException 
    {
        return sendCommandAndWaitForResponse("SDREC").toUpperCase().contains("ON");
    }
    
    /**
     * Set Technician Mode
     *
     * @return previous user mode
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     */
    public boolean SetTechnicianMode() throws CommSeveredException
    {
        String  response;
        boolean userMode;
        
        response = sendCommandAndWaitForResponse("TECHNICIAN ?"); // See what terminal mode we are in
        response = response.toUpperCase();
        userMode = response.contains("USER");
        
        if (userMode)                                    // If we are in user mode,
        {
            sendCommandAndWaitForResponse("TECHNICIAN"); // set technician mode.
        }
        
        return userMode;
    }

    /**
     * Restore WNet Mode
     *
     * @param userMode  true = user, false = technician
     * 
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     */
    public void RestoreWnetMode(boolean userMode) throws CommSeveredException
    {
        if (userMode)                              // If we were in user mode,
        {
            sendCommandAndWaitForResponse("USER"); // restore user mode.
        }
    }
    
    /**
     * Connects to the WNet and begins reading data.
     *
     * @param ipAddress The address or host name of the WNet.
     * @param profile   The WNet profile from which to read.
     * @param save      Should WNet profile changes be saved
     *                  to the unit's permanent memory?
     * 
     * startup commands sent to the unit.
     * @param controller
     * @throws UnknownHostException If there is an error resolving the hostname.
     * @throws IOException If there is an error connecting to the sensor.
     * @throws CommSeveredException if the additional commands used at
     * connect-time cannot be sent for any reason.
     */
    public void connect(String ipAddress, WirelessFTDemoProfile profile, boolean save, WirelessFTDemoMainScreenController controller) throws UnknownHostException, IOException, CommSeveredException 
    {
        m_controller              = controller;
        m_sensorAddressOrHostName = ipAddress;
        m_sensor                  = new WirelessFTSensor(m_sensorAddressOrHostName);
        m_flagReadStreaming.set(true);
        
        // Ensure the active transducers match the xpwr commands sent.
        for (int transducer = 0; transducer < MAX_SENSORS; transducer++) 
        {
            String power = profile.m_xpwr[transducer].toUpperCase();
            m_sensorActive[transducer] = power.contains("ON");
        }

        sendCommandAndWaitForResponse(profile.getRateCommand());  // Send RATE command
       
        sendCommandAndWaitForResponse("T OFF");                   // Ensure packet transmit is off.
        
        boolean userMode = SetTechnicianMode();                   // Set technician mode
       
        if (   profile.m_ntpUse                                   // If NTP is selected,
            && profile.m_ntpServer.length() > 0)                  // and there is an NTP server specified,
        {
            setNtpParameters(profile);                            // set the NTP parameters.
        }
        else                                                      // If we are not using NTP,
        {
            setTimeAndDate();                                     // Set time/date from the system clock.
        }
       
        sendStartupCommands(profile);                             // Send startup commands from the WNet profile.
       
        RestoreWnetMode(userMode);                                // restore Wnet mode

        sendCommandAndWaitForResponse("T ON");                    // WNet has been prepared, ensure packet transmit is on.
       
        if (save)                                                 // If the "Save profile settings to my Wireless F/T" box is checked,
        {
            sendCommandAndWaitForResponse("SAVEALL");             // save changes.
        }
        
        /* Initialize the connection for the data collection thread
         * and retrieve the mask that identifies active sensors.
         */
        assert m_sensor != null : "CollectDataThread can't run because sensor is null.";
        
        try 
        {
            m_sensor.startStreamingData();
        } 
        catch (IOException exc) 
        {
            m_logger.log(Level.SEVERE, "Exception requesting streaming data: {0}", exc.getMessage());
        }
    }
    
    /**
     * Sets the time and date within the WNet to
     * match the time of the Java Virtual Machine.
     * 
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    private void setTimeAndDate() throws CommSeveredException 
    {
        sendCommandAndWaitForResponse("NTP ENA 0"); // turn off NTP

        DateFormat dateFormat  = new SimpleDateFormat("yyyy/MM/dd_HH:mm:ss.SSS");
        Calendar   cal         = Calendar.getInstance();
        Date       date        = new Date(cal.getTimeInMillis() + 28); // add fudge factor
        String     connectedAt = dateFormat.format(date);
        String[]   dateAndTime = connectedAt.split("_");
        
        sendCommandAndWaitForResponse("NET DATE " + dateAndTime[0]);
        sendCommandAndWaitForResponse("NET TIME " + dateAndTime[1]);
    }
    
    /**
     * Sets the NTP parameters within the WNet.
     * 
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    private void setNtpParameters(WirelessFTDemoProfile profile) throws CommSeveredException 
    {
        String ntpDsp = profile.m_ntpDst ? "ON" : "OFF";
        
        sendCommandAndWaitForResponse("NTP SERVER " + profile.m_ntpServer);
        sendCommandAndWaitForResponse("NTP ZONE "   + String.format("%02d%02d", profile.m_ntpOffsetHours, profile.m_ntpOffsetMinutes));
        sendCommandAndWaitForResponse("NTP DST "    + ntpDsp);
        sendCommandAndWaitForResponse("NTP ENA 1");
    }
    
    /**
     * Gets the active calibrations from the WNet.
     * 
     * @return the active calibrations from the WNet
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    public int[] getActiveCalibrations() throws CommSeveredException
    {
        String response;
        String lines[];
        int[]  calibs = {0, 0, 0, 0, 0, 0};
        int    transducer;
        int    calibration;
        
        response = sendCommandAndWaitForResponse("CALIB");
        lines    = response.split("\r\n");                   // Split response into individual lines.
        
        for (String line : lines) 
        {
            String[] fields = line.trim().split("\\s+");     // Split this line into individual tokens.
            
            if (fields.length >= 2)                          // If there are at least two tokens in this line,
            {
                try
                {
                    transducer  = Integer.parseInt(fields[0]) - 1; // Transducer  is origin 1 to the user
                    calibration = Integer.parseInt(fields[1]) - 1; // Calibration is origin 1 to the user
                }
                catch (NumberFormatException nfe)
                {
                    transducer  = -1;
                    calibration = -1;
                }
            
                if (   transducer  >= 0 && transducer  < MAX_SENSORS
                    && calibration >= 0 && calibration < MAX_CALIBRATIONS)
                {
                    calibs[transducer] = calibration;
                }
            }
        }
        
        return calibs;
    }
    
    /**
     * Gets the powered Transducers from the WNet.
     * 
     * @return the powered Transducers from the WNet
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    public boolean[] getPoweredTransducers() throws CommSeveredException
    {
        String response;
        String lines[];
        boolean[] transducers = {false, false, false, false, false, false};
        int    transducer;
        String stateIn;
        
        response = sendCommandAndWaitForResponse("XPWR");
        lines    = response.split("\r\n");                   // Split response into individual lines.
        
        for (String line : lines) 
        {
            String[] fields = line.trim().split("\\s+");     // Split this line into individual tokens.
            
            if (fields.length >= 2)                          // If there are at least two tokens in this line,
            {
                try
                {
                    transducer = Integer.parseInt(fields[0]) - 1; // Transducer is origin 1 to the user
                }
                catch (NumberFormatException nfe)
                {
                    transducer = -1;
                }

                stateIn = fields[1];
            
                if (   transducer >= 0 && transducer < MAX_SENSORS
                    && stateIn.equals("ON"))
                {
                    transducers[transducer] = true;
                }
            }
        }
        
        return transducers;
    }
    
    private String getOperand(String operand, String buf, String cmd)
    {
        int    i;
        
        buf = buf.toUpperCase();
        cmd = cmd.toUpperCase();
        i   = buf.indexOf(cmd);
        
        if (i >= 0) // If found
        {
            operand = buf.substring(i + cmd.length());
            operand = operand.trim();
        }
        
        return operand;
    }
    
    /**
     * Gets the Component Versions from the WNet.
     * 
     * @return the Component Versions from the WNet
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    public String[] getComponentVersions() throws CommSeveredException
    {
        String   response;
        String   lines[];
        String[] versions = {"", "", "", ""};
      
        response = sendCommandAndWaitForResponse("VERSIONS");
        lines    = response.split("\r\n");                   // Split response into individual lines.
        
        for (String line : lines) 
        {
            versions[0] = getOperand(versions[0], line, "Firmware");
            versions[1] = getOperand(versions[1], line, "WLAN Module");
            versions[2] = getOperand(versions[2], line, "CPLD 0");
            versions[3] = getOperand(versions[3], line, "CPLD 1");
        }
      
        return versions;
    }
    
    /**
     * Sends all of the commands from the given
     * profile so that the WNet is in the same
     * state as the profile itself.
     * 
     * @param profile The WNet profile to read commands from.
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    private void sendStartupCommands(WirelessFTDemoProfile profile) throws CommSeveredException 
    {
        sendCommandAndWaitForResponse(profile.getSDCommand()); // Send SDREC [ON | OFF]
        
        // Send transducer-specific commands.
        for (int transducer = 0; transducer < MAX_SENSORS; transducer++) // For each Transducer,
        {
            if (m_sensorActive[transducer])                              // If this transducer is active,
            {
                String xpwr   = profile.getXPWRCommand       (transducer);
                String filter = profile.getFilterCommand     (transducer);
                String trans  = profile.getTransducerCommand (transducer);
                String calib  = profile.getCalibrationCommand(transducer);

                sendCommandAndWaitForResponse(xpwr);      // Send XPWR   command
                sendCommandAndWaitForResponse(filter);    // Send FILTER command
                
                if (calib.length() > 0)                   // If the calibration is not Default,
                {
                    sendCommandAndWaitForResponse(trans); // send TRANS command,
                    sendCommandAndWaitForResponse(calib); // send CALIB command.
                }
            }
        }
    }

    /**
     * Selects either Gage or Force/Torque data output.
     *
     * @param forceTorqueButton True to select F/T data, false to select Gage data.
     * @throws CommSeveredException if the
     * commands cannot be sent for any reason.
     */
    public void selectGageOrFTData(boolean forceTorqueButton) throws CommSeveredException 
    {
        /* Demo keeps it simple for user by selecting gage or FT data globally, 
         * but you actually have to set it for each transducer.
         * 
         * 2/24/14: Matrix multiplication is set to on/off for all transducers, regardless of whether or not they
         * are sending data.
         */
        boolean userMode = SetTechnicianMode(); // Set technician mode

        for (int transducer = 0; transducer < MAX_SENSORS; transducer++) // For all Transducers,
        {
            if (m_sensorActive[transducer])                              // that are active,
            {
                sendCommandAndWaitForResponse("TRANS "    + (transducer + 1)); // Transducer is origin 1 to the user
                sendCommandAndWaitForResponse("CAL MULT " + (forceTorqueButton ? "ON" : "OFF"));
            }
        }

        RestoreWnetMode(userMode);              // restore Wnet mode
    }

    /**
     * Sets the active calibration index.
     *
     * @param cal The calibration index (0-2).
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public void setActiveCalibration(int cal) throws CommSeveredException 
    {
        sendCommandAndWaitForResponse("CALIB " + (cal + 1)); // Calibration is origin 1 to the user
    }

    /**
     * Sets the active sensor index.
     *
     * @param transducer The transducer index (0-5).
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public void setActiveSensor(int transducer) throws CommSeveredException 
    {
        sendCommandAndWaitForResponse("TRANS " + (transducer + 1)); // Transducer is origin 1 to the user
    }

    /**
     * Reads the active calibration of the active transducer.
     *
     * @return The calibration data.
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public Calibration readActiveCalibration() throws CommSeveredException 
    {
        String      response = sendCommandAndWaitForResponse("CAL");
        Calibration rtn      = Calibration.parseCalibrationFromTelnetResponse(response);
        return      rtn;
    }

    /**
     * Stops writing to any open files and closes
     * UDP and TCP connections for the WNet.
     */
    public void disconnect() 
    {
        try 
        {
            m_sensor.stopStreamingData(); // Tell the Wnet to stop sending UDP packets
        } 
        catch (IOException exc) 
        {
            m_logger.log(Level.WARNING, "Exception requesting stop streaming data: {0}", exc.getMessage());
        }

        m_sensor.endCommunication();
    }
    
    /**
     * Sets the bias to "on" for the given sensor.
     * 
     * @param transducer The sensor to bias (0 to 5).
     * @throws CommSeveredException if the command cannot be sent for any reason.
     */
    public void biasSensor(int transducer) throws CommSeveredException 
    {
        m_sensor.sendTelnetCommand("BIAS " + (transducer + 1) + " ON", true); // Transducer is origin 1 to the user
    }
    
    /**
     * Sets the bias to "off" for the given sensor.
     * 
     * @param transducer The sensor to un-bias (0 to 5).
     * @throws CommSeveredException if the command cannot be sent for any reason.
     */
    public void unbiasSensor(int transducer) throws CommSeveredException
    {
        m_sensor.sendTelnetCommand("BIAS " + (transducer + 1) + " OFF", true); // Transducer is origin 1 to the user
    }

    /**
     * Starts a multiple-file upload thread.
     *
     * @param fs The list of file(s) to upload.
     * @param m_model
     * @throws FileNotFoundException If file(s) not found.
     * @throws IOException If file(s) cannot be parsed into byte-arrays.
     */
    public void startFileUpload(List<File> fs, final WirelessFTDemoModel m_model) throws FileNotFoundException, IOException 
    {
        m_fileUploadProgress.set(0.0);
        ArrayList<File> files = new ArrayList<>();
        for (File f : fs) 
        {
            files.add(f);
        }
        WriteFileTask writeTask = new WriteFileTask(files);
        new Thread(writeTask).start(); // start the file upload thread
        
        writeTask.stateProperty().addListener(new ChangeListener<Worker.State>() 
        {
            /**
             * Keeps track of the file-upload progress
             * and signals when the job is done.
             * 
             * @param ov The current observable state of the task.
             * @param t  The previous state of the task.
             * @param t1 The new state of the task.
             */
            @Override
            public void changed(ObservableValue<? extends Worker.State> ov, Worker.State t, Worker.State t1) 
            {
                if (t1 == Task.State.SUCCEEDED) // If File upload complete (2),
                {
//                  m_logger.log(Level.INFO, "File upload complete 2"); // (2)
                    m_flagFileUploadComplete.set(true);
                    
                    if (m_model.getNeedWnetReset())
                    {
                        m_controller.disconnectButtonPressed();
                    }
                }
            }
        });
    }
    
    /**
     * Verifies that the file upload task is complete.
     * 
     * @return Has the file-upload task finished?
     */
    public ObservableValue<Boolean> checkFileUploadStatus() 
    {
        return m_flagFileUploadComplete;
    }
    
    private boolean m_NeedWnetReset = false; // Did we need to reset the WNet?
    
    public boolean getNeedWnetReset()
    {
        return m_NeedWnetReset;
    }
    
    /**
     * Writing data to a file is pushed to a separate thread
     * to keep UI, network communication, and other activities
     * running smoothly.
     */
    public class WriteFileTask extends Task<Void> 
    {
        ArrayList<String> m_fileName  = new ArrayList<>(); // The name of the file(s), as stored on the WNet.
        ArrayList<byte[]> m_data      = new ArrayList<>(); // Raw byte-data from each read-in file.
        double            m_totalSize = 0.0;               // The total size of the upload job.
        
        /**
         * Gets the size (in bytes) of
         * any file in the upload list.
         * 
         * @param filename The name of the file.
         * @return The total size, in bytes,
         * of the file to be sent to the WNet.
         */
        public int getFileSize(String filename) 
        {
            int i = m_fileName.indexOf(filename);
            if (i != -1)
            {
                return m_data.get(i).length;
            }
            return -1; // File not found.
        }
        
        /**
         * Creates new WriteFileThread to upload file to WNet.
         *
         * @param fs The list of files write.
         * @throws FileNotFoundException If invalid file(s) passed.
         * @throws IOException If the given file(s) cannot be parsed
         * into byte-arrays.
         */
        public WriteFileTask(ArrayList<File> fs) throws FileNotFoundException, IOException 
        {
            for (File f : fs) 
            {
                byte[] fileToWrite;
                try (FileInputStream input = new FileInputStream(f)) 
                {
                    fileToWrite = new byte[(int) f.length()];
                    input.read(fileToWrite);
                }
                m_data.add(fileToWrite);
                
                String fname = f.getName();
                m_fileName.add(fname);
            }
            
            for (byte[] b : m_data)  // Get the total size of the files to upload.
            {
                m_totalSize += (double) b.length;
            }
        }
        
        /**
         * The actual task of writing the files to the WNet's
         * Serial Flash memory, performed when the thread starts.
         * 
         * @throws CommSeveredException if the
         * commands cannot be sent for any reason.
         */
        @Override
        public Void call() throws CommSeveredException 
        {
            final int chunksize = 106;
            double    bytesUp   = 0.0;
            
            m_NeedWnetReset = false;
            
            for (int file = 0; file < m_fileName.size(); file++)   // For each file in the list of files to upload,
            {
                String fileName = m_fileName.get(file);            // get the name of the file.
                int    fileLen  = m_data.get(file).length;
                
                sendCommandAndWaitForResponse("FDEL " + fileName); // Delete the file, just in case it already exists in the Wnet.
                    
                if (fileName.equals("appl.bin"))                   // If this file is appl.bin,
                {
                    m_NeedWnetReset = true;                        // set flag to do a reset when we are done uploading files.
                }
                
                for (int i = 0; i < fileLen; i += chunksize)       // For each chunk of data in this file,
                {
                    String chunkHex = "";
                    int    chunkLen = (i + chunksize) > fileLen ? fileLen - i : chunksize;
                    
                    for (int j = 0; j < chunkLen; j++)             // For each byte of data in this file chunk,
                    {
                        int    dataByte = 0xff & m_data.get(file)[i + j];
                        String byteHex  = String.format("%02x", dataByte);
                        chunkHex       += byteHex;
                    }
                    
                    sendCommandAndWaitForResponse("FHEX " + fileName + " " + chunkHex); // Send a chunk of datas.
                    
                    bytesUp += (double)chunkLen;                                        // Update the progress bar.
                    m_fileUploadProgress.set(bytesUp / m_totalSize);
                }
                
                sendCommandAndWaitForResponse("FCLOSE");
            }
            
            if (m_NeedWnetReset)                               // If appl.bin was one of the files that was uploaded,
            {
                try 
                {
                    m_sensor.sendTelnetCommand("RESET", true); // issue RESET to Wnet.
                } 
                catch (CommSeveredException ex)                // Communications are expected to be severed after a RESET,
                {                                              // so do nothing.
                }
            }
            
//          m_logger.log(Level.INFO, "File upload complete 1"); // (1)
            return null;
        }
    }
    
    /**
    * Reads WNet's current IP settings.
    *
    * @return The current IP settings.
    * @throws CommSeveredException if the
    * command cannot be sent for any reason.
    */
    public IPSettings readIPSettings() throws CommSeveredException 
    {
        String response = sendCommandAndWaitForResponse("IP");
        return new IPSettings(response);
    }
    
    /**
     * Writes new IP settings to WNet.
     * 
     * @param settings The settings to write.
     * @throws CommSeveredException if the
     * commands cannot be sent for any reason.
     */
    public void writeIPSettings(IPSettings settings) throws CommSeveredException
    {
        sendCommandAndWaitForResponse("MYIP "     + settings.IPAddress);
        sendCommandAndWaitForResponse("GATEIP "   + settings.DefaultGateway);
        sendCommandAndWaitForResponse("NETMASK "  + settings.SubnetMask);
        sendCommandAndWaitForResponse("SSID "     + settings.SSID);
        sendCommandAndWaitForResponse("NET DHCP " + settings.DHCP);
        
        String bandString; /* Parameter to band command. */
        switch (settings.Band)
        {
            case Spectrum2_4Ghz:
                bandString = "2.4";
                break;
            case Spectrum5Ghz:
                bandString = "5";
                break;
            default:
                throw new IllegalArgumentException("Unknown band setting " + settings.Band.toString());
        }
        sendCommandAndWaitForResponse("BAND " + bandString);            
        sendCommandAndWaitForResponse("SAVEALL");
    }

    /**
     * Sends a firmware command over Telnet (TCP port 23) and waits
     * for a full response (any response text up to the next prompt).
     * 
     * @param command The command to send.
     * @return The response string.
     * @throws CommSeveredException if the
     * command cannot be sent for any reason.
     */
    public String sendCommandAndWaitForResponse(String command) throws CommSeveredException 
    {
        String response = "";
        
        m_sensor.sendTelnetCommand(command, true);               // send command, clearing the input buffer first.
    
        while (!response.endsWith("\r\n>"))                      // wait for a prompt.
        {
            response = response + m_sensor.readTelnetData(true); // Blocks if no available data
        }
        
        return response;
    }
}
    