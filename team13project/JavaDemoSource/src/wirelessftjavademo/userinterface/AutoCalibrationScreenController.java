/*
 * Auto-Calibration Screen Controller
 */
package wirelessftjavademo.userinterface;

import java.io.IOException;
import java.net.URL;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.ResourceBundle;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.concurrent.Task;
import javafx.concurrent.WorkerStateEvent;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.RadioButton;
import javafx.scene.control.SingleSelectionModel;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.HBox;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftsensor.WirelessFTSensor;

/**
 *
 * @author jenwi
 */
public class AutoCalibrationScreenController implements Initializable
{
    
    @FXML
    ComboBox m_cmbAutoCalTrans;

    @FXML
    RadioButton m_rbAutoCal1;

    @FXML
    RadioButton m_rbAutoCal2;

    @FXML
    RadioButton m_rbAutoCal3;
    
    @FXML
    Button m_btnAutoCalibrate;
    
    @FXML
    Button m_btnExtCalibrate;
    
    @FXML
    private TextArea m_txtAreaInstructions;
    
    @FXML
    private Label m_labelDataEntry;
    
    @FXML
    private TextField m_textFieldDataEntry;
    
    @FXML
    private Button m_btnContinue;
    
    @FXML
    ProgressBar m_progressAutoCal; // Displays Auto Calibration progress.
    
    @FXML
    private HBox m_hBoxEntry;
        
    @FXML
    private Label m_labelInputRange;
        
    @FXML
    private TextField m_textFieldInputRange;
    
    /**
     * Auto-calibration constants and variables.
     */
    private static final int    SETTLE_TIME        = 150; // mS
    private static final int    BSEARCH_ITERS      = (16 - 1); // Binary search interations
    private static final double NOMINAL_VOLTAGE    = 2.5;
    private static final double NOMINAL_RESISTANCE = 25.0;

    private static double m_inputRangeMV = 250.0;
    
    private int m_transducer; // 0 to 5
    
    float m_progress;
    float m_progressGoal;
    
    private WirelessFTDemoModel                m_model;
    private WirelessFTDemoMainScreenController m_controller;
    
    private       RadioButton[] m_AutoCalRBs;
    private final ToggleGroup   m_AutoCalRBToggleGroup = new ToggleGroup();

    private static int[] m_GageData = new int[WirelessFTDemoModel.NUM_AXES];

    private int[]   m_zeroScaleGageData;
    private int[]   m_fullScaleGageData;
    
    private int[]   m_commonModeGageData;
    private int[]   m_vPlusGageData;
    
    private boolean m_stop = false;

    private int     m_extCalPhase = 0;
    private boolean m_calError    = false;
    
    private double  m_vCommon;
    private double  m_vPlus;
    
    private boolean m_forceTorqueSetting;
    
    private boolean  m_allResistancesGood;
    private final double[] m_resistances = {0, 0, 0, 0, 0, 0};

    
    /**
     * The active transducers as a list.
     */
    private final ObservableList<String> m_AutoCalTrans = FXCollections.observableArrayList();
    
    @Override
    public void initialize(URL location, ResourceBundle resources) // Called once when AutoCalibrationScreenController.fxml is loaded.
    {
    }
    
    public void OnCloseRequest()
    {
        m_stop = true;
    }
    
    private void SetUpForStart()
    {
        m_btnAutoCalibrate   .setDisable(false); // Enable the Auto-Calibrate button.
        m_btnExtCalibrate    .setDisable(false); // Enable the Ext -Calibrate button.
        m_labelInputRange    .setDisable(false); // Enable Input Range label.
        m_textFieldInputRange.setDisable(false); // Enable Input Range text field.
        m_textFieldInputRange.setText(String.format("%.3f", m_inputRangeMV));
    }
    
    // Runs after this screen is opened.
    public void initializeAutoCalibrationScreen(WirelessFTDemoModel model, WirelessFTDemoMainScreenController controller) throws IOException, WirelessFTSensor.CommSeveredException 
    {
        m_model      = model;
        m_controller = controller;
        m_AutoCalTrans.clear();
        
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
        {
            if (model.m_sensorActive[transducer]) 
            {
                m_AutoCalTrans.add("Transducer " + (transducer + 1)); // Transducer is origin 1 to the user
            } 
        }
        
        m_cmbAutoCalTrans.setItems(m_AutoCalTrans);
        
        if (m_AutoCalTrans.size() > 0) 
        {
            m_cmbAutoCalTrans.getSelectionModel().select(0);
        }
        
        m_AutoCalRBs = new RadioButton[] {m_rbAutoCal1, m_rbAutoCal2, m_rbAutoCal3};
        
        for (RadioButton button : m_AutoCalRBs) 
        {
            button.setToggleGroup(m_AutoCalRBToggleGroup);
            button.setDisable(false);
        }
        
        SetUpForStart();
    }
    
    /**
     * Set Gage Data.
     * 
     * This function is called by a listener whenever a new data packet
     * is received from the Wnet.
     * @param data
     */
    public void setGageData(int[][] data) // Called from the listener in WirelessFTDemoMainScreenController
    {
        if (m_transducer >= 0 && m_transducer < WirelessFTDemoModel.MAX_SENSORS)
        {
            m_GageData = data[m_transducer];             // Save the latest gage data locally.
        }
    }
    
    private int[] getGageData()
    {
        System.out.println(Arrays.toString(m_GageData));
        return m_GageData;
    }
    
    /**
     * Set Force Torque Setting.
     * 
     * This function is called by a listener whenever the FT or Gage
     * buttons are pressed on the main XY graph screen.
     * @param setting
     */
    public void setForceTorqueSetting(boolean setting)
    {
        m_forceTorqueSetting = setting;
    }
    
    /**
     * Read the selected transducer number from auto-calibration transducer combo box.
     */
    private int GetAutoCalTransducer()
    {
        SingleSelectionModel ssm = m_cmbAutoCalTrans.getSelectionModel(); // Get Filter/Calibration combo box

        if (ssm.isEmpty())                        // If no selection has been made,
        {
            return 0;
        }
    
        Object selection = ssm.getSelectedItem(); // Get the selected item

        if (selection == null)                    // If there is NO combo box selection,
        {
            return 0;
        }

        String trans = selection.toString();      // get the associated string.

        if (trans.contains("ALL"))                // If ALL transducers is set,
        {
            return WirelessFTDemoModel.MAX_SENSORS; // return 6.
        }
        else                                      // If one transducer is set,
        {
            String last = trans.substring(trans.length() - 1); // Get last character of string
            return Integer.parseInt(last) - 1;    // return the Transducer number // Transducer is origin 1 to the user
        }
    }
    
    /**
     * Read the selected Calibration number from auto-calibration transducer combo box.
     */
    private int GetAutoCalCalibration()
    {
        RadioButton button = (RadioButton) m_AutoCalRBToggleGroup.getSelectedToggle(); // Get RadioButton
        
        if (button == null)             // If there is no calibration selection
        {
            return 0;
        }
        
        String cal = button.getText();  // Get RadioButton label

        if (cal == null)                // If no selection has been made,
        {
            return 0;
        }
        
        if (cal.isEmpty())
        {
            return 0;
        }
    
        String last = cal.substring(cal.length() - 1); // Get last character of string
        return Integer.parseInt(last);  // return the Calibration number
    }

    /**
     * Runs when the auto-calibration transducer changes.
     * @param event
     */
    @FXML
    protected void m_cmbAutoCalTrans_Changed(ActionEvent event) 
    {
    }

    /**
     * Runs when the auto-calibration calibration number changes.
     * @param event
     */
    @FXML
    protected void m_rbAutoCalTransCalChanged(ActionEvent event) 
    {
    }
    
    private void ReportAnyCalibrationErrors()
    {
        m_txtAreaInstructions.setDisable(false); // Enable Instruction field

        if (m_calError)
        {
                m_txtAreaInstructions.setText("Calibration Error!\r\n" 
                                            + "Zero scale: " + Arrays.toString(m_zeroScaleGageData) + "\r\n"
                                            + "Full scale: " + Arrays.toString(m_fullScaleGageData));
        }
        else
        {
            m_txtAreaInstructions.setText("Success");
        }
        
        if (!m_allResistancesGood) // If any resistance was not read correctly,
        {                          // pop up a message.
            try
            {
                String message =   "At least one EEPOT resistance did not read correctly.\r\n" 
                                 + "Using nominal resistance of " + NOMINAL_RESISTANCE + " K-Ohms instead:";
                
                for (int gage = 0; gage < m_resistances.length; gage++)            // For all gages.
                {
                    message = message + "\r\n" + gage + " " + m_resistances[gage];
                }
            
                PopupDialogController.PopUpDialogBox("Auto Calibration", message);
            }
            catch (IOException ioe) 
            {
                ioe.printStackTrace();
            }
        }
    }
    
    /**
     * Runs when the auto-calibration button is pressed.
     * @param event
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     * @throws java.lang.InterruptedException
     */
    @FXML
    protected void m_btnAutoCalibratePressed(ActionEvent event) throws WirelessFTSensor.CommSeveredException, InterruptedException 
    {
        double value;
        
        m_calError = false;
        m_txtAreaInstructions.setText("");
        m_txtAreaInstructions.setDisable(true); // Disable Instruction field
        m_labelDataEntry     .setDisable(true); // Disable Data Entry label
        m_textFieldDataEntry .setDisable(true); // Disable Data Entry field
        m_btnContinue        .setDisable(true); // Disable Continue button
        // Note: TextArea.setFont does not exist in Java 7.
        
        try
        {
            value = Double.parseDouble(m_textFieldInputRange.getText());
        }
        catch (NumberFormatException nfe) // If the text is not a number,
        {
            m_textFieldInputRange.setText("BAD");
            return;
        }
        if (value < 30.0 || value > 250.0) // If the value is out of range,
        {
            m_textFieldDataEntry .setText(value < 30.0 ? "LOW" : "HIGH");
            return;
        }

        m_textFieldInputRange.setText(String.format("%.3f", value));
        System.out.println("inputRange = " + value + " mV");
        m_inputRangeMV = value;

        m_btnAutoCalibrate   .setDisable(true); // Disable Auto-Calibrate button.
        m_btnExtCalibrate    .setDisable(true); // Disable Ext -Calibrate button.
        m_labelInputRange    .setDisable(true); // Disable Input Range label.
        m_textFieldInputRange.setDisable(true); // Disable Input Range text field.

        AutoCalibrateTask autoCalibrateTask = new AutoCalibrateTask();
        
        autoCalibrateTask.setOnSucceeded(new EventHandler<WorkerStateEvent>() // This method runs when the autoCalibrateTask is complete.
        {
            @Override
            public void handle(WorkerStateEvent t) 
            {
                ReportAnyCalibrationErrors();
                SetUpForStart(); // Issue a CAL command and put the parameters somewhere.
            }
        });
        
        new Thread(autoCalibrateTask).start(); // Start the auto-calibration task.
    }
    
    /**
     * Runs when the external-calibration button is pressed.
     * @param event
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     * @throws java.lang.InterruptedException
     */
    @FXML
    protected void m_btnExtCalibratePressed(ActionEvent event) throws WirelessFTSensor.CommSeveredException, InterruptedException 
    {
        double value;
        
        try
        {
            value = Double.parseDouble(m_textFieldInputRange.getText());
        }
        catch (NumberFormatException nfe) // If the text is not a number,
        {
            SetUpExtCalPhase2();
            m_textFieldInputRange.setText("BAD");
            return;
        }
        if (value < 30.0 || value > 250.0) // If the value is out of range,
        {
            SetUpExtCalPhase2();
            m_textFieldDataEntry .setText(value < 30.0 ? "LOW" : "HIGH");
            return;
        }

        m_textFieldInputRange.setText(String.format("%.3f", value));
        System.out.println("inputRange = " + value + " mV");
        m_inputRangeMV = value;

        m_extCalPhase = 1;
        m_calError     = false;
        m_btnContinuePressed();
        m_btnAutoCalibrate   .setDisable(true); // Disable Auto-Calibrate button.
        m_btnExtCalibrate    .setDisable(true); // Disable Ext -Calibrate button.
        m_labelInputRange    .setDisable(true); // Disable Input Range label.
        m_textFieldInputRange.setDisable(true); // Disable Input Range text field.
    }
    
    private void SetUpExtCalPhase2()
    {
        m_extCalPhase = 2;
        
        m_txtAreaInstructions.setDisable(false); // Enable Instruction field
        m_txtAreaInstructions.setText(  "1. Connect "     + NOMINAL_VOLTAGE + " VDC to all channels of the selected transducer input.\r\n" 
                                      + "2. Measure the " + NOMINAL_VOLTAGE + " VDC source using a calibrated voltmeter and type it into the Value field below.\r\n" 
                                      + "3. Press Continue.");
        
        m_labelDataEntry     .setDisable(false); // Enable Data Entry label
        m_textFieldDataEntry .setDisable(false); // Enable Data Entry field
        m_textFieldDataEntry .setText(String.format("%.6f", NOMINAL_VOLTAGE));
        
        m_btnContinue        .setDisable(false); // Enable Continue button
    }
    
    private void SetUpExtCalPhase3()
    {
        double input   = m_inputRangeMV / 1000.0;
        double low     = NOMINAL_VOLTAGE + input * 0.9;
        double high    = NOMINAL_VOLTAGE + input * 1.1;
        String stLow   = String.format("%.6f", low);
        String stHigh  = String.format("%.6f", high);
        m_extCalPhase = 3;
        
        m_btnContinue        .setDisable(false); // Enable Continue button
        m_labelDataEntry     .setDisable(false); // Enable Data Entry label
        m_textFieldDataEntry .setDisable(false); // Enable Data Entry field
        m_txtAreaInstructions.setDisable(false); // Enable Instruction field
        m_txtAreaInstructions.setText(  "1. Connect [" + stLow + " -> " + stHigh + "] VDC (suggested) to the all channels of the selected transducer input.\r\n" 
                                      + "2. Measure the input voltage using a calibrated voltmeter and type it into the Value field below.\r\n"
                                      + "3. Press Continue.");
    }
    
    @FXML
    protected void m_btnContinuePressed()
    {
        double value;
        
        m_txtAreaInstructions.setText("");
        m_txtAreaInstructions.setDisable(true); // Disable Instruction field
        m_labelDataEntry     .setDisable(true); // Disable Data Entry label
        m_textFieldDataEntry .setDisable(true); // Disable Data Entry field
        m_btnContinue        .setDisable(true); // Disable Continue button
        m_labelInputRange    .setDisable(true); // Disable Input Range label.
        
        switch (m_extCalPhase)
        {
            case 1:
                ExtCalibrateTask1 extCalibrateTask1 = new ExtCalibrateTask1();

                extCalibrateTask1.setOnSucceeded(new EventHandler<WorkerStateEvent>() // This method runs when extCalibrateTask1 is complete.
                {
                    @Override
                    public void handle(WorkerStateEvent t) 
                    {
                        SetUpExtCalPhase2();
                    }
                });

                System.out.println("Starting ExtCalTask 1");
                new Thread(extCalibrateTask1).start();
                break;
                
            case 2:
                try
                {
                    value = Double.parseDouble(m_textFieldDataEntry.getText());
                }
                catch (NumberFormatException nfe) // If the text is not a number,
                {
                    SetUpExtCalPhase2();          // repeat this phase.
                    m_textFieldDataEntry.setText("BAD");
                    break;
                }
                if (value < 0.0 || value > 5.0)  // If the value is out of range,
                {
                    SetUpExtCalPhase2();         // repeat this phase.
                    m_textFieldDataEntry.setText(value < 0.0 ? "LOW" : "HIGH");
                    break;
                }
                
                m_textFieldDataEntry .setText(String.format("%.6f", value));
                System.out.println("vCommon    = " + value + " V" );
                m_vCommon = value;
                
                ExtCalibrateTask2 extCalibrateTask2 = new ExtCalibrateTask2();

                extCalibrateTask2.setOnSucceeded(new EventHandler<WorkerStateEvent>() // This method runs when extCalibrateTask2 is complete.
                {
                    @Override
                    public void handle(WorkerStateEvent t) 
                    {
                        SetUpExtCalPhase3();
                    }
                });

                System.out.println("Starting ExtCalTask 2");
                new Thread(extCalibrateTask2).start();
                break;
                
            case 3:
                try
                {
                    value = Double.parseDouble(m_textFieldDataEntry.getText());
                }
                catch (NumberFormatException nfe)    // If the text is not a number,
                {
                    SetUpExtCalPhase3();             // repeat this phase.
                    m_textFieldDataEntry.setText("BAD");
                    break;
                }
                if (value < 0.0 || value > 5.0) // If the value is out of range,
                {
                    SetUpExtCalPhase3();             // repeat this phase.
                    m_textFieldDataEntry.setText(value < 0.0 ? "LOW" : "HIGH");
                    break;
                }
                
                m_textFieldDataEntry.setText(String.format("%.6f", value));
                System.out.println("V+ = " + value + " V");
                m_vPlus = value;

                ExtCalibrateTask3 extCalibrateTask3 = new ExtCalibrateTask3();

                extCalibrateTask3.setOnSucceeded(new EventHandler<WorkerStateEvent>() // This method runs when extCalibrateTask3 is complete.
                {
                    @Override
                    public void handle(WorkerStateEvent t) 
                    {
                        ReportAnyCalibrationErrors();
                        SetUpForStart();
                        m_extCalPhase = 0;
                    }
                });

                System.out.println("Starting ExtCalTask 3");
                new Thread(extCalibrateTask3).start();
                break;
                
            default:
                break;
        }
    }

    /**
     * Settle Delay.
     * 
     * This allows time for the ADCs in the Wnet to settle into their new values after parameter changes.
     */
    private void SettleDelay() throws InterruptedException
    {
        Thread.sleep(SETTLE_TIME);
        m_progress++;
        m_progressAutoCal.setProgress(m_progress / m_progressGoal);
    }
    
    /**
     * Set gains.
     * 
     * Set the gains necessary for the Voltage-Calibration process.
     * 
     * Magic gain formula from David Fleissner:
     * "I derived the gain from the circuit in terms of the gain pot value and RAB.
     * I didn’t do any curve fitting here, this is what you get when you solve for
     * the voltage gain of the entire circuit in terms of the digital pot value, 
     * then have Wolfram alpha solve that equation for the digital pot value."
     * 
     * Original equation Wolfram alpha solved:
     * a == 4 (1 + (g r)/(1024 (10/11 + ((1024 - g) r)/(1024 (1 + ((1024 - g) r)/1024)))))
     */
    private void SetGains() throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        double   av          = 2000.0 / m_inputRangeMV;    // voltage gain of the whole amplifier circuit
        double[] resistances = getEepotResistanceValues(); // Get the calibrated resistance for each gage from the Wnet.
        
        for (int gage = 0; gage < WirelessFTDemoModel.NUM_AXES; gage++)
        {
            double rab  = resistances[gage];               // Individual EEPOT resistance for each gage in K-Ohms
            double temp =    5376 * rab * av  
                          + 11264 * rab * rab 
                          - 10240 * rab;
            double sqrt = Math.sqrt(44*rab*rab*(-5505024*av*rab - 2621440*av + 22020096*rab + 10485760) + temp*temp);
            int    gpv  = (int) ((temp - sqrt) / (22*rab*rab));

            m_model.sendCommandAndWaitForResponse("G " + gage + " " + gpv); // Set channel gain on the current Transducer
        }
    }
    
    /**
     * Get the EEPOT resistance values for all gages of the active Transducer
     * from the WNet.
     * 
     * @return the EEPOT resistance values from the WNet
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException 
     */
    public double[] getEepotResistanceValues() throws WirelessFTSensor.CommSeveredException
    {
        String   response;
        String[] lines;
        
        double   resistance;
        
        int      transducer;
        int      gage;
        
        response = m_model.sendCommandAndWaitForResponse("EEPOT");   // Issue the EEPOT command.
        lines    = response.split("\r\n");                           // Split response into individual lines.
        
        for (gage = 0; gage < m_resistances.length; gage++)
        {
            m_resistances[gage] = 0;
        }
        
        for (String line : lines)                                    // For each line of the response,
        {
            String[] fields = line.trim().split("\\s+");             // split this line into individual tokens.
            
            if (fields.length >= 22)                                 // If there are enough tokens in this line,
            {
                try
                {
                    transducer = Integer.parseInt(fields[0]) - 1;    // Get active Transducer number. // Transducer is origin 1 to the user
                    gage       = Integer.parseInt(fields[1]) * 2;    // Get EEPOT number, convert to Gage number.

                    if (   transducer == m_transducer                // If this is the active Transducer,
                        && gage       >= 0                           // and the Gage number is valid,
                        && gage       <  WirelessFTDemoModel.NUM_AXES)
                    {
                        resistance = Double.parseDouble(fields[21]); // Get the EEPOT's calibrated resistance.
                        m_resistances[gage    ] = resistance;        // set the resistance.
                        m_resistances[gage + 1] = resistance;        // Note that there are two Gages per EEPOT.
                    }
                }
                catch (NumberFormatException nfe)                    // If the is a number format issue, do nothing here.
                {                                                    // Any problems parsing actual resistance values will be dealt with in the loop
                }                                                    // below, which looks for any resistances which weren't set to a new value.
            }
        }
        
        m_allResistancesGood = true;                                 // All gages are assumed good until proven otherwise.
        
        for (gage = 0; gage < m_resistances.length; gage++)          // For all gages.
        {
            if (m_resistances[gage] == 0)                            // If this resistance was not read,
            {
                m_resistances[gage]  = NOMINAL_RESISTANCE;           // set nominal resistance value,
                m_allResistancesGood = false;                        // and set flag so that we pop up an error message.
            }
        }
        
        return m_resistances;
    }
    
    /**
     * Auto Zero Offset
     * 
     * This function goes through the process to automatically set the zero offsets.
     */
    private int[] AutoZeroOffsets() throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        int[] offset          = { 32768,  32768,  32768,  32768,  32768,  32768};
        int[] bestOffset      = { 32768,  32768,  32768,  32768,  32768,  32768};
        int[] minGageReadings = {500000, 500000, 500000, 500000, 500000, 500000};

        m_model.sendCommandAndWaitForResponse("O * 32768"); // Set all channel offsets to 32768
        SettleDelay();
        if (m_stop) 
        {
            return null;
        }
        
        for (int i = 0; i < BSEARCH_ITERS; i++) // Find the optimal 16-bit offset one bit at a time.
        {
            int gageData[] = getGageData();
            int shift      = Math.max(BSEARCH_ITERS - 1 - i, 0); // Start checking at the +/- 2**14 range
            int mask       = 1 << shift;
            
            for (int channel = 0; channel < WirelessFTDemoModel.NUM_AXES; channel++)
            {
                int value;
                int change;
                int output;
                
                value            = gageData[channel];
                change           = mask * Integer.signum(value);
                offset[channel] += change;
                output           = Math.min(offset[channel], 65535); // Saturate
                m_model.sendCommandAndWaitForResponse("O " + channel + " " + output);
            }
                      
            SettleDelay();
            if (m_stop) 
            {
                return null;
            }
            
            gageData = getGageData();

            for (int channel = 0; channel < WirelessFTDemoModel.NUM_AXES; channel++) // see if changed offsets improved readings
            {
                int value = gageData[channel];
                int output;
                        
                if (Math.abs(value) > Math.abs(minGageReadings[channel]))
                {
                    offset[channel] = bestOffset[channel];
                    output          = Math.min(offset[channel], 65535);
                    m_model.sendCommandAndWaitForResponse("O " + channel + " " + output);
                }
                else
                {
                    minGageReadings[channel] = value;
                    bestOffset     [channel] = offset[channel];
                }
            }

            SettleDelay();
            if (m_stop) 
            {
                return null;
            }
        }
        
        return bestOffset;
    }
    
    /**
     * average
     * 
     * This function returns the average value of the int array that is passed in.
     */
    private int average(int[] data)
    {
        long average = 0;
        long length  = data.length;
        
        for (int i = 0; i < length; i++)
        {
            average += data[i];
        }
        
        return (int) (average / length);
    }
    
    /**
     * Set DAC Test Signal Zero
     * 
     * This function goes through the process to automatically set the channel 6 DAC zero offset.
     */
    private int SetDacTestSignalZero() throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        int gageData[];
        
        int dacCh6     = 32768;
        int bestDacCh6 = 32768;
        int minGageAvg;
        
        m_model.sendCommandAndWaitForResponse("O 6 32768"); // Set channel 6 offset to 32768
        SettleDelay();
        if (m_stop) 
        {
            return 0;
        }
        
        gageData   = getGageData();
        minGageAvg = average(gageData);
        
        for (int i = 0; i < BSEARCH_ITERS; i++) // Find the optimal 16-bit offset one bit at a time.
        {
            int shift;
            int mask;
            int value;
            int change;
            int output;
            
            shift    = Math.max(BSEARCH_ITERS - 1 - i, 0); // Start checking at the +/- 2**14 range
            mask     = 1 << shift;
            gageData = getGageData();
            value    = average(gageData);
            change   = mask * -Integer.signum(value);
            dacCh6  += change;
            output   = Math.min(dacCh6, 65535); // Saturate
            m_model.sendCommandAndWaitForResponse("O 6 " + output);
            
            SettleDelay();
            if (m_stop) 
            {
                return 0;
            }
            
            gageData = getGageData();
            value    = average(gageData);
            
            if (Math.abs(value) > Math.abs(minGageAvg))
            {
                dacCh6 = bestDacCh6;
                output = Math.min(dacCh6, 65535);
                m_model.sendCommandAndWaitForResponse("O 6 " + output);
                SettleDelay();
                if (m_stop) 
                {
                    return 0;
                }
            }
            else
            {
                minGageAvg = value;
                bestDacCh6 = dacCh6;
                m_progress++;
                
                if (Math.abs(minGageAvg) < 1) // If we hit center point exactly,
                {
                    m_progress += 16 - i;
                    break;                    // return early.
                }
            }
        }
        
        return bestDacCh6;
    }
    
    /**
     * Set DAC Test Signal Cal
     */
    private void SetDacTestSignalCal(int dacCh6Zero) throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        int offset = dacCh6Zero + (int) ((65535.0 / 5.0) * (m_inputRangeMV / 1000.0) + 0.5);
        m_model.sendCommandAndWaitForResponse("O 6 " + offset);
        SettleDelay();
    }
    
    /**
     * Init Calibration
     * 
     * This method is used by both Auto and External Calibration.
     */
    private boolean InitCalibration(String calType) throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        int calibration;
        
        m_progressAutoCal.setProgress(0.0);                                           // Initialize the progress bar.
        m_progress    = 0;
            
        m_transducer  = GetAutoCalTransducer ();                                      // Get the selected Transducer  from the ComboBox
        calibration   = GetAutoCalCalibration();                                      // Get the selected Calibration from the RadioButtons
        
        System.out.println(calType + "-calibrate: Transducer " + (m_transducer + 1) + ", Calibration " + calibration);
        
        if (calibration < 1 || calibration > 3)                                       // If the Calibration is out of range,
        {
            return false;                                                             // we are done.
        }

        m_model.sendCommandAndWaitForResponse("D OFF");                               // Set Debug Dump off
        m_model.sendCommandAndWaitForResponse("RATE 30 32");                          // Set packet rate = 30 packets/second, sampling rate = 30 * 32 = 960 samples/second
        m_model.sendCommandAndWaitForResponse("FILTER " + (m_transducer + 1) + " MEAN 32"); // Set transducer filter to mean with 32 taps
        m_model.sendCommandAndWaitForResponse("BIAS "   + (m_transducer + 1) + " OFF");     // Set transducer bias off
        m_model.sendCommandAndWaitForResponse("TRANS "  + (m_transducer + 1));              // Make this the default Transducer
        m_model.sendCommandAndWaitForResponse("CALIB "  +  calibration);                    // Make this the default Calibration
        m_model.sendCommandAndWaitForResponse("CAL MULT OFF");

        SetGains();                                                                   // Set gains based on input range
        return true;
    }

    /**
     * Set Calibration Matrix
     * 
     * This method is used by both Auto and External Calibration.
     */
    private boolean SetCalibrationMatrix(double inputRangeMv, int[] zeroScaleGageData, int[] fullScaleGageData) throws WirelessFTSensor.CommSeveredException
    {
        double range = inputRangeMv * 10000.0;
        
        for (int channel = 0; channel < WirelessFTDemoModel.NUM_AXES; channel++)
        {
            String command   = "CAL MATRIX " + channel + " 0";
            double fullScale = fullScaleGageData[channel];
            double zeroScale = zeroScaleGageData[channel];
            double value;

            if (fullScale != zeroScale)
            {
                value = range / (fullScale - zeroScale);
            }
            else
            {
                value = 0.0;
                m_calError = true;
            }

            for (int i = 0; i < WirelessFTDemoModel.NUM_AXES; i++)
            {
                command = command + " " + ((i == channel) ? value : 0);
            }

            m_model.sendCommandAndWaitForResponse(command); // It wouldn't be a bad idea to verify the calibration matrix at this point.
        }

        m_model.sendCommandAndWaitForResponse("CAL MAX * " + inputRangeMv); // Set maxRatings fields for all gages to the calibrated voltage range in mV.
            
        if (m_calError)
        {
            System.out.println("Calibration error!");
        }
        
        return true;
    }

    /**
     * Finalize Calibration
     * 
     * This method is used by both Auto and External Calibration.
     */
    private void FinalizeCalibration() throws WirelessFTSensor.CommSeveredException, InterruptedException
    {
        DateFormat dateFormat  = new SimpleDateFormat("yyyy/MM/dd");
        Calendar   calendar    = Calendar.getInstance();
        Date       date        = new Date(calendar.getTimeInMillis());
        String     dateString  = dateFormat.format(date);
        m_model.sendCommandAndWaitForResponse("CAL DATE " + dateString);

        m_model.sendCommandAndWaitForResponse("CAL SERIAL Voltage");
        m_model.sendCommandAndWaitForResponse("CAL PART Voltage");
        m_model.sendCommandAndWaitForResponse("CAL FORCE 10000 mV");
        m_model.sendCommandAndWaitForResponse("CAL TORQUE 10000 mV");
      //m_model.sendCommandAndWaitForResponse("BIAS " + transducer + " ON");
      //m_model.sendCommandAndWaitForResponse("SAVEALL");  

        if (m_forceTorqueSetting) // If Force/Torque is selected on the main XY graph screen,
        {
            m_model.sendCommandAndWaitForResponse("CAL MULT ON");
        }
        else
        {
            m_model.sendCommandAndWaitForResponse("CAL MULT OFF");
        }
        
        m_controller.refreshCalibrationInformation();
        m_progressAutoCal.setProgress(1.0);
    }
    
    /**
     * Auto-calibration is done in a separate thread
     * to keep UI, network communication, and other activities
     * running smoothly.
     */
    private class AutoCalibrateTask extends Task<Void> 
    {
        @Override
        protected Void call() throws Exception 
        {
            boolean good;
            int     dacCh6;
            
            m_progressGoal = BSEARCH_ITERS * 2 + BSEARCH_ITERS * 2 + 1 + 1;               // Calculate total number of settling operations needed to auto-calibrate.
            
            good = InitCalibration("Auto");                                               // Init for auto-calibration
            
            if (!good)                                                                    // If the init was not successful,
            {
                return null;                                                              // we are done.
            }

            m_model.sendCommandAndWaitForResponse("TEST " + (m_transducer + 1) + " ZERO"); // Pass in 2.5V ref for the auto zero function

            AutoZeroOffsets();                                                            // Perform the auto zero function
            if (m_stop) 
            {
                return null;
            }

            m_model.sendCommandAndWaitForResponse("TEST " + (m_transducer + 1) + " DAC"); 

            dacCh6            = SetDacTestSignalZero();                                   // Find zero point for DAC ch 6
            if (m_stop) 
            {
                return null;
            }
            m_zeroScaleGageData = getGageData();                                            // Record ADC readings from DAC ch 6

            SetDacTestSignalCal(dacCh6);                                                  // Apply input range signal using DAC ch 6
            if (m_stop) 
            {
                return null;
            }
            m_fullScaleGageData = getGageData();                                            // Record ADC readings from DAC input range signal

            m_model.sendCommandAndWaitForResponse("TEST " + (m_transducer + 1) + " OFF");   // Restore normal operation

            System.out.println("Zero scale: " + Arrays.toString(m_zeroScaleGageData));
            System.out.println("Full scale: " + Arrays.toString(m_fullScaleGageData));
                    
            good = SetCalibrationMatrix(m_inputRangeMV, m_zeroScaleGageData, m_fullScaleGageData);
           
            if (!good)                                                                    // If setting the calibration matrix was not successful,
            {
                return null;                                                              // we are done.
            }

            FinalizeCalibration();
            return null;
        }
    }

    /**
     * Ext-calibration Step 1.
     * This is done in a separate thread to keep UI, network
     * communication, and other activities running smoothly.
     */
    private class ExtCalibrateTask1 extends Task<Void> 
    {
        @Override
        protected Void call() throws Exception 
        {
            m_progressGoal = BSEARCH_ITERS * 2 + 2; // Calculate total number of settling operations needed to ext-calibrate.
            InitCalibration("Ext");                 // init for ext-calibration
            return null;
        }
    }

    /**
     * Ext-calibration Step 2.
     * This is done in a separate thread to keep UI, network
     * communication, and other activities running smoothly.
     */

    private class ExtCalibrateTask2 extends Task<Void> 
    {
        @Override
        protected Void call() throws Exception 
        {
            AutoZeroOffsets();                  // Perform the auto zero function
            m_commonModeGageData = getGageData(); // record ADC readings from common mode voltage
            return null;
        }
    }

    /**
     * Ext-calibration Step 3.
     * This is done in a separate thread to keep UI, network
     * communication, and other activities running smoothly.
     */
    private class ExtCalibrateTask3 extends Task<Void> 
    {
        @Override
        protected Void call() throws Exception 
        {
            boolean good;
            
            m_vPlusGageData = getGageData(); // record ADC readings from V+ voltage
            System.out.println("Common: " + Arrays.toString(m_commonModeGageData));
            System.out.println("V+:     " + Arrays.toString(m_vPlusGageData));
            
            good = SetCalibrationMatrix((m_vPlus - m_vCommon) * 1000.0, m_commonModeGageData, m_vPlusGageData);
           
            if (!good)                           // If setting the calibration matrix was not successful,
            {
                return null;                     // we are done.
            }

            FinalizeCalibration();
            return null;
        }
    }
}    
