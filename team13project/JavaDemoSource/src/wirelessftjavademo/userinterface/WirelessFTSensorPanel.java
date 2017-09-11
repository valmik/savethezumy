/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;


import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftjavademo.WirelessFTDemoProfile;
import wirelessftsensor.WirelessFTSample;
import wirelessftsensor.WirelessFTSensor.CommSeveredException;

/**
 * FXML Controller class
 *
 * @author Sam Skuce, Chris Collins
 */
public class WirelessFTSensorPanel extends AnchorPane {

    /**
     * The default packet send rate.
     */
    private final int DEFAULT_RATE = 125;
    
    /**
     * Display new data at this rate.
     */
    private final double UI_UPDATE_HZ = 25.0;
    
    /**
     * Displays the transducer and serial numbers for a device.
     */
    @FXML
    Label m_txtTitle;
    
    /**
     * Force and torque unit labels.
     */
    @FXML
    Label m_lblFUnits;
    @FXML
    Label m_lblTUnits;
    
    /**
     * Axis Buttons.
     */
    @FXML
    ToggleButton m_btnTitleFx;
    @FXML
    ToggleButton m_btnTitleFy;
    @FXML
    ToggleButton m_btnTitleFz;
    @FXML
    ToggleButton m_btnTitleTx;
    @FXML
    ToggleButton m_btnTitleTy;
    @FXML
    ToggleButton m_btnTitleTz;
    
    /**
     * The axis title buttons.
     */
    ToggleButton[] m_btnAxisTitles;
    
    /**
     * The VBox that groups the data values.
     */
    @FXML
    VBox m_vboxDataDisplay;
    
    /**
     * The toggle group for controlling graph views.
     */
    @FXML
    ToggleGroup m_toggleGroupPower;
    
    /**
     * Turns the display on and off.
     */
    @FXML
    ToggleButton m_btnDisplay;
    
    /**
     * The auto-scale check box.
     */
    @FXML
    CheckBox m_chkAutoscale;
    
    /**
     * The x-axis of the chart.
     */
    @FXML
    NumberAxis m_xAxisForc;
    
    @FXML
    NumberAxis m_xAxisTorq;
    
    /**
     * The y-axis of the chart.
     */
    @FXML
    NumberAxis m_yAxisForc;
    
    @FXML
    NumberAxis m_yAxisTorq;
    
    /**
     * The line chart that displays the data over time.
     */
    @FXML
    LineChart m_lineChartForc; // Force chart
    
    @FXML
    LineChart m_lineChartTorq; // Torque chart
    
    /**
     * The FX label.
     */
    @FXML
    Label m_lblFx;
    
    /**
     * The FY label.
     */
    @FXML
    Label m_lblFy;
    
    /**
     * The FZ label.
     */
    @FXML
    Label m_lblFz;
    
    /**
     * The TX label.
     */
    @FXML
    Label m_lblTx;
    
    /**
     * The TY label.
     */
    @FXML
    Label m_lblTy;
    
    /**
     * The TZ label.
     */
    @FXML
    Label m_lblTz;
    
    /**
     * The labels that display the current data.
     */
    Label[] m_ftLabels;
    
    /**
     * The WNet profile which applies to all transducers.
     */
    WirelessFTDemoProfile m_profile = new WirelessFTDemoProfile();
    
    /**
     * The sensor within the WNet for which
     * this panel is displaying data.
     */
    private int m_sensorIndex;
    
    /**
     * The number of UI update cycles.
     */
    private int m_uiTicks = 0;
    
    /**
     * The total graph history displayed.
     */
    private final double m_historySeconds = 10.0;
    
    /**
     * The current F/T or gage data.
     */
    private double[] m_currentData = new double[WirelessFTDemoModel.NUM_AXES];
    
    /**
     * The current power setting.
     */
    private final int m_powerSetting = 0;
    
    /**
     * Conversion factors.
     */
    private final double CONVERT_FORCE_POUND_LBF      =    1;
    private final double CONVERT_FORCE_KILOPOUND_KLBF = 1000;
    private final double CONVERT_FORCE_NEWTON_N       =    4.448222;
    private final double CONVERT_FORCE_KILONEWTON_KN  =    0.004448222;
    private final double CONVERT_FORCE_GRAM_G         =  453.5924;
    private final double CONVERT_FORCE_KILOGRAM_KG    =    0.4535924;
    
    private final double CONVERT_TORQUE_POUND_INCHES_LBFIN       =   1;
    private final double CONVERT_TORQUE_POUND_FEET_LBFFT         =   0.0833333;
    private final double CONVERT_TORQUE_NEWTON_METER_NM          =   0.1129848;
    private final double CONVERT_TORQUE_NEWTON_MILLIMETER_NMM    = 112.984829;
    private final double CONVERT_TORQUE_KILOGRAM_CENTIMETER_KGCM =   1.15212462;
    private final double CONVERT_TORQUE_KILONEWTON_METER         =   0.000112985;
    
    //private final double[] forceConversionFactors = 
    //{
    //    CONVERT_FORCE_POUND_LBF,     CONVERT_FORCE_KILOPOUND_KLBF, CONVERT_FORCE_NEWTON_N, 
    //    CONVERT_FORCE_KILONEWTON_KN, CONVERT_FORCE_GRAM_G,         CONVERT_FORCE_KILOGRAM_KG
    //};
    
    //private final double[] torqueConversionFactors = 
    //{
    //    CONVERT_TORQUE_POUND_INCHES_LBFIN,    CONVERT_TORQUE_POUND_FEET_LBFFT,         CONVERT_TORQUE_NEWTON_METER_NM, 
    //    CONVERT_TORQUE_NEWTON_MILLIMETER_NMM, CONVERT_TORQUE_KILOGRAM_CENTIMETER_KGCM, CONVERT_TORQUE_KILONEWTON_METER
    //};
    
    private String m_forceUnits  = "N";
    private String m_torqueUnits = "N-m";
    
    private double m_forceConversionFactor  = 1;
    private double m_torqueConversionFactor = 1;
    
    /**
     * Are we displaying secondary axis?
     */
    private boolean m_forceTorqueButton;
    
    /**
     * ActionEvent raised when user presses Bias button on a panel.
     */
    private final ObjectProperty<EventHandler<ActionEvent>> m_propertyOnBiasRequested = new SimpleObjectProperty<>();
    
    /**
     * ActionEvent raised when user presses Unbias button on a panel.
     */
    private final ObjectProperty<EventHandler<ActionEvent>> m_propertyOnUnbiasRequested = new SimpleObjectProperty<>();
    
    /**
     * Used to monitor bias requests.
     * 
     * @return The property to listen to for user bias requests.
     */
    public final ObjectProperty<EventHandler<ActionEvent>> onBiasRequestedProperty() 
    {
        return m_propertyOnBiasRequested;
    }
    /**
     * Used to monitor un-bias requests.
     * 
     * @return The property to listen to for user un-bias requests.
     */
    public final ObjectProperty<EventHandler<ActionEvent>> onUnbiasRequestedProperty() 
    {
        return m_propertyOnUnbiasRequested;
    }
    
    /**
     * Set the action to take when the user requests a bias.
     *
     * @param handler The handler for user bias requests.
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     */
    public final void setOnBiasRequested(EventHandler<ActionEvent> handler) throws CommSeveredException 
    {
        m_propertyOnBiasRequested.set(handler);
    }
    /**
     * Set the action to take when the user requests an un-bias.
     *
     * @param handler The handler for user bias requests.
     * @throws wirelessftsensor.WirelessFTSensor.CommSeveredException
     */
    public final void setOnUnbiasRequested(EventHandler<ActionEvent> handler) throws CommSeveredException 
    {
        m_propertyOnUnbiasRequested.set(handler);
    }
    
    /**
     * Gets the handler for bias requests.
     * 
     * @return The handler for user bias requests.
     */
    public final EventHandler<ActionEvent> getOnBiasRequested() 
    {
        return m_propertyOnBiasRequested.get();
    }
    /**
     * Gets the handler for un-bias requests.
     * 
     * @return The handler for user un-bias requests.
     */
    public final EventHandler<ActionEvent> getOnUnbiasRequested() 
    {
        return m_propertyOnUnbiasRequested.get();
    }
    
    /**
     * Gets the current power setting for this transducer.
     * 
     * @return The requested power setting: 0 = on, 1 = off, 2 = warm/idle.
     */
    public int getPowerSetting() 
    {
        return m_powerSetting;
    }
    
    /**
     * The F/T data displayed in the LineChart. You'd think I could have just
     * done this as a fixed size array - except that gives me a "Generic array
     * creation" error. Rather than doing gymnastics per the accepted answer at
     * http://stackoverflow.com/questions/529085/java-how-to-generic-array-creation,
     * I decided to just use an ArrayList which I will never actually change the
     * size of.
     */
    private final List<XYChart.Series<Number, Number>> m_ftSeries = new ArrayList<>(WirelessFTDemoModel.NUM_AXES);

    /**
     * Called when the bias button is pressed.
     */
    @FXML
    protected void biasButtonPressed() 
    {
        m_propertyOnBiasRequested.get().handle(new ActionEvent(this, null));
    }
    
    /**
     * Called when the clear button is pressed.
     */
    @FXML
    protected void unbiasButtonPressed() 
    {
        m_propertyOnUnbiasRequested.get().handle(new ActionEvent(this, null));
    }
    
    /**
     * Set the sample data to which this sensor corresponds,
     * then apply transformations (if applicable).
     * 
     * @param sample The data to display.
     */
    public void setSensorData(WirelessFTSample sample) 
    {
        int gageData[] = sample.getFtOrGageData()[m_sensorIndex];

        for (int i = 0; i < WirelessFTDemoModel.NUM_AXES; i++) // For each data channel,
        {
            m_currentData[i] = gageData[i];
        }
        
        // Apply transformations for this transducer, if there were any.
        m_currentData = matrixMult(m_profile.getTransformationMatrix(m_sensorIndex), m_currentData);
    }
    
    /**
     * Called when the auto-scale check-box is pressed.
     */
    @FXML
    protected void autoscaleChecked() 
    {
        boolean autoRanging = m_chkAutoscale.isSelected();
        m_yAxisForc.setAutoRanging(autoRanging);
        m_yAxisTorq.setAutoRanging(autoRanging);
    }

    /**
     * Checks to see how many seconds have passed
     * since this panel was first drawn.
     * 
     * @return The current timestamp in seconds based on the UI tick count.
     */
    private double timeInSeconds() 
    {
        return m_uiTicks / UI_UPDATE_HZ;
    }
    
    private void SetForceTorqueOrRawCounts(boolean forceTorque)
    {
        int i;
            
        m_lineChartForc.getData().clear();
        m_lineChartTorq.getData().clear();
        m_lineChartTorq.setVisible(forceTorque);           // If displaying in force/torque only, show the secondary axis
        
        if (forceTorque)                                   // If displaying in force/torque only,
        {
            for (i = 0; i < WirelessFTDemoModel.NUM_AXES / 2; i++) // Initialize Force series.
            {
                XYChart.Series series = m_ftSeries.get(i); // The current force series.
                m_lineChartForc.getData().add(series);
            }

            for (; i < WirelessFTDemoModel.NUM_AXES; i++)  // Initialize Torque series.
            {
                XYChart.Series series = m_ftSeries.get(i); // The current torque series.
                m_lineChartTorq.getData().add(series);
            }
        }
        else                                               // If we are only displaying the primary X axis,
        {
            for (i = 0; i < WirelessFTDemoModel.NUM_AXES; i++) // initialize raw count series.
            {
                XYChart.Series series = m_ftSeries.get(i); // The current raw counts series.
                m_lineChartForc.getData().add(series);
            }
        }
    }

    /**
     * Creates new WirelessFTSensorPanel user control by loading FXML,
     * initializing series data, and starting its animation.
     * 
     * @param profile  The WNet profile for units and transformations.     
     */
    public WirelessFTSensorPanel(WirelessFTDemoProfile profile) 
    {
        int i;
        
        m_profile = profile;
        
        // Load FXML.
        URL        url        = getClass().getResource("WirelessFTSensorPanel.fxml");
        FXMLLoader fxmlLoader = new FXMLLoader(url);
        fxmlLoader.setRoot      (this);
        fxmlLoader.setController(this);
        
        try 
        {
            fxmlLoader.load();
        } 
        catch (IOException exc) 
        {
            throw new RuntimeException(exc);
        }
       
        for (i = 0; i < WirelessFTDemoModel.NUM_AXES; i++)  // Initialize plot series.
        {
            XYChart.Series<Number, Number> series = new XYChart.Series<>();
            m_ftSeries.add(i, series);
            series.getData().clear();
        }
        
        SetForceTorqueOrRawCounts(true); // Turn on the primary and secondary x axes.
        
        m_lineChartForc.setCreateSymbols           (false);
        m_lineChartForc.setAnimated                (false);
        m_lineChartForc.setVerticalGridLinesVisible(false);
        
        m_lineChartTorq.setCreateSymbols           (false);
        m_lineChartTorq.setAnimated                (false);
        m_lineChartTorq.setVerticalGridLinesVisible(false);
        
        double tickMarkLineLength = (1 / UI_UPDATE_HZ) * (Integer.parseInt(m_profile.m_rate) / DEFAULT_RATE);

        m_xAxisForc.setForceZeroInRange (false);
        m_xAxisForc.setAutoRanging      (false);
        m_xAxisForc.setTickLength       (tickMarkLineLength);
        m_xAxisForc.setTickLabelsVisible(false);
        m_xAxisForc.setTickMarkVisible  (false);
        m_xAxisForc.setLowerBound       (timeInSeconds());
        m_xAxisForc.setUpperBound       (timeInSeconds() + m_historySeconds);

        m_yAxisForc.setAutoRanging      (true);
        
        m_xAxisTorq.setForceZeroInRange (false);
        m_xAxisTorq.setAutoRanging      (false);
        m_xAxisTorq.setTickLength       (tickMarkLineLength);
        m_xAxisTorq.setTickLabelsVisible(false);
        m_xAxisTorq.setTickMarkVisible  (false);
        m_xAxisTorq.setLowerBound       (timeInSeconds());
        m_xAxisTorq.setUpperBound       (timeInSeconds() + m_historySeconds);
        
        m_yAxisTorq.setAutoRanging      (true);
        
        // Set up labels.
        m_btnAxisTitles = new ToggleButton[] {m_btnTitleFx, m_btnTitleFy, m_btnTitleFz, m_btnTitleTx, m_btnTitleTy, m_btnTitleTz};
        m_ftLabels      = new Label[]        {m_lblFx,      m_lblFy,      m_lblFz,      m_lblTx,      m_lblTy,      m_lblTz};
    }

    private String[] colors = new String[] {"#e41a1c",  // red 
                                            "#ff7f00",  // orange
                                            "#c2c200",  // yellow
                                            "#4daf4a",  // green 
                                            "#377eb8",  // blue
                                            "#984ea3"}; // purple
    /**
     * Displays the most-recently read data.
     */
    public void updatePlot() 
    {
        boolean voltage = m_forceUnits.toLowerCase().equals("mv");
        
        double forceCF;
        double torqueCF;
        
        String numberFormat;
        
        if (!m_forceTorqueButton)                        // If displaying in gage,
        {
            forceCF      = 1.0;
            torqueCF     = 1.0;
            numberFormat = "%12.0f";
            m_lblFUnits.setText("Raw counts");
            m_lblTUnits.setText("");
        }
        else if (voltage)                                // If displaying in voltage,
        {
            forceCF      = m_forceConversionFactor;
            torqueCF     = m_torqueConversionFactor;
            numberFormat = "%12.4f";
            m_lblFUnits.setText("Voltage (" + m_forceUnits  + ")");
            m_lblTUnits.setText("");
        }
        else                                             // If displaying in force/torque,
        {
            forceCF      = m_forceConversionFactor;
            torqueCF     = m_torqueConversionFactor;
            numberFormat = "%12.4f";
            m_lblFUnits.setText("Force ("   + m_forceUnits  + ")");
            m_lblTUnits.setText("Torque ("  + m_torqueUnits + ")");
        }
        
        m_yAxisForc.setLabel(m_lblFUnits.getText());
        m_yAxisTorq.setLabel(m_lblTUnits.getText());
        
        m_yAxisForc.setForceZeroInRange(m_forceTorqueButton);
        m_yAxisTorq.setForceZeroInRange(m_forceTorqueButton);
       
        double timeNow           = timeInSeconds();            // Note that this is the current JVM time, not the Wnet packet generation time.
        double minimumLowerBound = timeNow - m_historySeconds; // Timestamp of the oldest data we should be displaying.
        
        // Add new data to chart series, and erase old data if necessary.
        for (int gage = 0; gage < WirelessFTDemoModel.NUM_AXES; gage++) // For each gage on this Transducer,
        {
            XYChart.Series series = m_ftSeries.get(gage);           // Get the current F/T series.

            if (m_btnAxisTitles[gage].isSelected())                 // If this gage is selected,
            {
                double units  = (gage < WirelessFTDemoModel.NUM_AXES / 2) ? forceCF : torqueCF; // Pick the correct units
                double value  = (double) m_currentData[gage] * units; // Get current gage data, convert to units
                String number = String.format(numberFormat, value); // get data value to be printed left of graph
                m_ftLabels[gage].setText(number);                   // Print the number on the graph

                series.getNode().setStyle("-fx-stroke: " + colors[gage] + ";");         // Set data point color
                series.getData().add(new XYChart.Data<Number, Number>(timeNow, value)); // Add new data point to the series
            }
            
            // Erase old data that is no longer being displayed.
            // I'm doing this search for all old data instead of just removing a single
            // element from the beginning of the list whenever we have exceeded a certain
            // run time because the user can change the amount of history displayed. 
            // If this becomes a performance bottleneck, we can put in some
            // synchronization/resetting logic when doing that resize so that we could
            // guarantee that we would only ever have to remove a single point at a time
            // in this function.
            LinkedList<XYChart.Data<Number, Number>> oldData = new LinkedList<>();  // This will hold the old data to be removed.
            List      <XYChart.Data<Number, Number>> allData = series.getData();    // Get all current data points.

            for       (XYChart.Data<Number, Number> point : allData)                // For each current data point,
            {
                double x = point.XValueProperty().get().doubleValue();              // get the x coordinate.
                
                if (x < minimumLowerBound)                                          // if this point is too old,
                {
                    oldData.add(point);                                             // add it to the list of old data points.
                }
                else                                                                // If this point is in range,
                {
                    break;                                                          // we are done.
                }
            }

            series.getData().removeAll(oldData); // Remove all of the old data points from the series.
        }
        
        if (m_xAxisForc.getLowerBound() < minimumLowerBound) // Convert force units to display according to the user selection.
        {
            m_xAxisForc.setLowerBound(minimumLowerBound);
            m_xAxisForc.setUpperBound(timeNow);
        }
        
        if (m_xAxisTorq.getLowerBound() < minimumLowerBound) // Convert torque units to display according to the user selection.
        {
            m_xAxisTorq.setLowerBound(minimumLowerBound);
            m_xAxisTorq.setUpperBound(timeNow);
        }
        
        m_uiTicks++;
    }

    /**
     * Gets the index of the sensor
     * represented by this panel.
     * 
     * @return The zero-based index of the
     * sensor within the Wireless F/T to use.
     */
    public int getSensorIndex() 
    {
        return m_sensorIndex;
    }

    /**
     * Sets which sensor in the WNet
     * to display on this panel.
     *
     * @param transducer The zero-based index of the
     * sensor we wish to use on the wireless F/T.
     */
    public void setSensorIndex(int transducer) 
    {
        this.m_sensorIndex = transducer;
    }
    
    /**
     * Changes the title on this graph panel.
     * 
     * @param title The title
     */
    public void setTitle(String title) 
    {
        m_txtTitle.setText(title);
    }
    
    /**
     * Sets the force conversion factors.
     * 
     * @param forceConv  The force  conversion.
     * @param torqueConv The torque conversion.
     */
    public void setConversions(double forceConv, double torqueConv)
    {
        m_forceConversionFactor  = forceConv;
        m_torqueConversionFactor = torqueConv;
    }
    
    public void setForceTorqueUnits(String fUnits, String tUnits)
    {
        m_forceUnits  = fUnits;
        m_torqueUnits = tUnits;
    }
    
    /**
     * Sets the axis labeling.
     * 
     * @param forceTorqueButton
     */
    public void setDataDisplay(Boolean forceTorqueButton) 
    {
        m_forceTorqueButton = forceTorqueButton;

        boolean voltage = m_forceUnits.toLowerCase().equals("mv");
        boolean ft      = m_forceTorqueButton && !voltage; // If force/torque button is pressed and we are not displaying in mV, display in force/torque.
        
        String[] labelText; // The axis titles.
        
        if (ft) // If force/torque,
        {
            labelText = new String[] {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
        } 
        else    // If voltage or gage,
        {
            labelText = new String[] {"G0", "G1", "G2", "G3", "G4", "G5"};
        }
        
        for (int i = 0; i < WirelessFTDemoModel.NUM_AXES; i++) 
        {
            String label = labelText[i];
            m_btnAxisTitles[i].setText(label);
        }
        
        SetForceTorqueOrRawCounts(ft);
    }
    
    /**
     * Multiplies the matrices representing transformations
     * and raw gage data, respectively.
     * 
     * @param trans The transformations for this transducer.
     * @param gage The raw gage data for this transducer.
     * @return The resulting transformed matrix that will be displayed
     * to the user after unit conversion.
     */
    private static double[] matrixMult(double trans[][], double[] gage)
    {
        if (trans[0].length != gage.length) return null; // invalid dims

        double ans[] = new double[WirelessFTDemoModel.NUM_AXES];

        for (int i = 0; i < WirelessFTDemoModel.NUM_AXES; i++) 
        {
            double sum = 0.0;
            
            for (int j = 0; j < WirelessFTDemoModel.NUM_AXES; j++) 
            {
                sum += trans[i][j] * gage[j];
            }
            
            ans[i] = sum;
        }
                 
        return ans;
    }
}