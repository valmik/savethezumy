/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.ResourceBundle;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.prefs.Preferences;
import javafx.animation.Animation;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.application.Platform;
import javafx.beans.property.SimpleObjectProperty;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.ListView;
import javafx.scene.control.MenuItem;
import javafx.scene.control.TextField;
import javafx.scene.control.Toggle;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import javafx.util.Duration;
import wirelessftjavademo.Calibration;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftjavademo.WirelessFTDemoProfile;
import wirelessftsensor.WirelessFTSample;
import wirelessftsensor.WirelessFTSensor.CommSeveredException;

/**
 * FXML Controller class
 *
 * @author Sam Skuce, Chris Collins
 */
public class WirelessFTDemoMainScreenController implements Initializable 
{
    /**
     * Displays the battery LED status.
     */
    @FXML
    Circle m_batteryLED;
    
    /**
     * Displays the external power LED status.
     */
    @FXML
    Circle m_externalPowerLED;
    
    /**
     * Displays the Wi-fi LED status.
     */
    @FXML
    Circle m_wifiLED;
    
    /**
     * Displays the transducer 1 LED status.
     */
    @FXML
    Circle m_transducer1LED;
    
    /**
     * Displays the transducer 2 LED status.
     */
    @FXML
    Circle m_transducer2LED;
    
    /**
     * Displays the transducer 3 LED status.
     */
    @FXML
    Circle m_transducer3LED;
    
    /**
     * Displays the transducer 4 LED status.
     */
    @FXML
    Circle m_transducer4LED;
    
    /**
     * Displays the transducer 5 LED status.
     */
    @FXML
    Circle m_transducer5LED;
    
    /**
     * Displays the transducer 6 LED status.
     */
    @FXML
    Circle m_transducer6LED;
    
    /**
     * Transducer 1 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer1;
    
    /**
     * Transducer 2 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer2;
    
    /**
     * Transducer 3 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer3;
    
    /**
     * Transducer 4 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer4;
    
    /**
     * Transducer 5 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer5;
    
    /**
     * Transducer 6 Label, including saturation.
     */
    @FXML
    Label m_lblTransducer6;
    
    /**
     * Contains the path to the data to collect to file.
     */
    @FXML
    TextField m_txtFilePath;
    
    /**
     * The button that the user presses to start and stop collecting data.
     */
    @FXML
    Button m_btnCollectData;
    
    /**
     * The selectors for gage or FT data.
     */
    @FXML
    ToggleGroup m_toggleGroupGageOrFT;
    
    /**
     * radio button to select gage data.
     */
    @FXML
    ToggleButton m_toggleGageData;
    
    /**
     * radio button to select F/T data.
     */
    @FXML
    ToggleButton m_toggleFTData;
    
    /**
     * Contains the 2 sets of graph panels.
     */
    @FXML
    GridPane m_graphGrid;
    
    /**
     * A container for the first three dynamically created sensor panels.
     */
    @FXML
    VBox m_sensorPanelBox1;
    
    /**
     * A second container for additional dynamically created sensor panels.
     * It will be used if the user has more than 3 transducers active.
     */
    @FXML
    VBox m_sensorPanelBox2;
    
    /**
     *
     */
    @FXML
    Label m_DisplayPackets;
    
    /**
     *
     */
    @FXML
    Label m_DisplayRate;
    
    /**
     *
     */
    @FXML
    Label m_DisplayLatency;
    
    /**
     *
     */
    @FXML
    Label m_DisplayDrops;
    
    /**
     *
     */
    @FXML
    Label m_DisplayPktDrops;
    
    /**
     *
     */
    @FXML
    Label m_DisplayDropRate;
    
    /**
     *
     */
    @FXML
    Label m_DisplayOutOfOrders;
    
    /**
     *
     */
    @FXML
    Label m_DisplayDups;
    
    /**
     * The text field where the user enters the address of the Wireless FT.
     */
    @FXML
    TextField m_txtFieldSensorHostName;
    
    /**
     * The button pressed to detect Wireless F/Ts.
     */
    @FXML
    Button m_btnDiscover;
    
    /**
     * The button pressed to connect to the sensor.
     */
    @FXML
    Button m_btnConnect;
    
    /**
     * Brings up a file selector for the CollectData file.
     */
    @FXML
    Button m_btnDots;
    
    /**
     * The on-screen log of events.
     */
    @FXML
    ListView m_listLogEvents;
    
    /**
     * The first anchor pane
     */
    @FXML
    AnchorPane m_anchorPaneTop;
    
    /**
     * The second anchor pane
     */
    @FXML
    AnchorPane m_anchorPaneTwo;
    
    /**
     * The VBox containing all the data on the main screen
     */
    @FXML
    VBox m_dataVBox;
    
    /**
     * Drop-down menu options.
     */
    @FXML
    MenuItem m_menuItemSD;
    @FXML
    MenuItem m_menuItemClose;
    @FXML
    MenuItem m_menuItemCalibration;
    @FXML
    MenuItem m_menuItemAutoCalibration;
    @FXML
    MenuItem m_menuItemFirmwareUpgrade;
    @FXML
    MenuItem m_menuItemAbout;
    
    @FXML
    TextField m_applyRate;
    
    @FXML
    CheckBox m_chkShowPacketStats;
    @FXML
    VBox     m_vBoxPacketStats;
    
    /**
     * The WNet profile for this demo run.
     */
    private WirelessFTDemoProfile m_profile = new WirelessFTDemoProfile();
    
    /**
     * Should profile changes be saved on the WNet?
     */
    private boolean m_saveProfile = false;
    
    /**
     * Whether or not we are successfully connected to a WNet.
     */
    boolean m_connected = false;
    
    /**
     * Labels for the transducers.
     */
    private Label[] m_transLabels;
    
    /**
     * The thread for streaming UDP data.
     */
//   Thread m_dataCollectorThread;
    
    /**
     * Updates the data display.
     */
    private final Timeline m_animation = new Timeline();
    
    /**
     * Keeps track of the most-recently received Wireless F/T Sample.
     */
    private WirelessFTSample m_lastSample;
    
    /**
     * This is updated when new
     * samples are sent from the WNet.
     */
    private final SimpleObjectProperty<WirelessFTSample> m_sampleProperty = new SimpleObjectProperty<>(null);
    
    /**
     * Used to synchronize access to m_bufferedWriter.
     */
    private final Object m_bufferedWriterSynchroLock = new Object();
    
    /**
     * Writes collected data to file.
     */
    private BufferedWriter m_bufferedWriter = null;
    
    /**
     * Are we reading records?
     */
    private boolean m_readingRecords = false;
    
    /**
     * Are we collecting data to file?
     */
    private boolean m_collectingData = false;
    
    /**
     * Is the CollectDataThread is currently active?
     */
    public static boolean m_threadActive = false;
    
    private static long  m_drops          = 0; // The number of packet drop occurrences thus far, regardless of size.
    private static long  m_packets        = 0; // total number of packets received
    private static long  m_missedPackets  = 0; // The number of individual packets that have been missed.
    private final  float m_rxedPacketsTc  = 10000;
    private        float m_rxedPacketsAcc = 0;
    private static long  m_LastSequence   = 0;
    private static long  m_LastPacketTime = System.currentTimeMillis();
    private static float m_timeAcc        = 0; // The number of individual packets that have been missed.
    private final  float m_timeTc         = 500;
    private static long  m_OutOfOrders    = 0;
    private static long  m_Duplicates     = 0;
    
    /**
     * The most-recently connected IP address.
     */
    private static String m_latestIP;
    
    /**
     * Keeps track of exception/error info.
     */
    private static final Logger m_logger = Logger.getLogger("wirelessft");
    
    /**
     * Panels to display F/T data from up to the first
     * three sensors in the system.
     */
    private List<WirelessFTSensorPanel> m_sensorPanels1 = new ArrayList<>();
    
    /**
     * Panels to display F/T data from up to 3 additional sensors.
     */
    private List<WirelessFTSensorPanel> m_sensorPanels2 = new ArrayList<>();
    
    /**
     * The application model.
     */
    private final WirelessFTDemoModel m_model = new WirelessFTDemoModel();
    
    /**
     * Keep track of user preferences.
     */
    protected static Preferences m_prefsRoot;
    
    /**
     * The preferred display startup (raw data or not).
     */
    public static final String PREF_GAGE_OR_FT = "ftOrGageData"; 
    
    /**
     * The preferences user root node pathname.
     */
    public static final String PREF_USER_ROOT_NODE_PATHNAME = "com.FTDemo.preference.Settings";
    
    /**
     * The latest file-selection window for data collection, a preference keyword.
     */
    public static final String PREF_DATA_COLLECTION_DIRECTORY = "latestCollectionDir";
    
    /**
     * The latest IP setting, a preference keyword
     */
    public static final String PREF_LATEST_IP = "latestIPConnectAttempt";
    
    /**
     * The latest window width setting, a preference keyword
     */
    public static final String PREF_LAST_WINDOW_WIDTH = "latestWindowWidth";
    
    /**
     * The latest window height setting, a preference keyword
     */
    public static final String PREF_LAST_WINDOW_HEIGHT = "latestWindowHeight";
    
    /**
     * The latest x window position
     */
    public static final String PREF_LAST_X_POSITION = "latestXWindowPosition";
    
    /**
     * The latest y window position
     */
    public static final String PREF_LAST_Y_POSITION =  "latestYWIndowPosition";
    
    /**
     * The last force unit selected
     */
    public static final String PREF_FORCE_UNIT = "forceUnitConversion";
    
    /**
     * The last torque unit selected
     */
    public static final String PREF_TORQUE_UNIT = "torqueUnitConversion";
    
    /**
     * The last displacement unit selected
     */
    public static final String PREF_DISPLACEMENT_UNIT = "displacementUnits";
    
    /**
     * The last rotations unit selected
     */
    public static final String PREF_ROTATIONS_UNIT = "rotationsUnits";
    
    /**
     * Rotation/Displacement string prefixes (loop to use one for each sensor).
     */
    public static final String PREF_DISPLACEMENT_X = "displacementX";
    public static final String PREF_DISPLACEMENT_Y = "displacementY";
    public static final String PREF_DISPLACEMENT_Z = "displacementZ";
    
    public static final String PREF_ROTATIONS_X    = "rotationsX";
    public static final String PREF_ROTATIONS_Y    = "rotationsY";
    public static final String PREF_ROTATIONS_Z    = "rotationsZ";
    
    /**
     * The last IP settings.
     */
    public static final String PREF_BAND            = "band";
    public static final String PREF_DHCP            = "dhcp";
    public static final String PREF_SSID            = "ssid";
    public static final String PREF_IP_ADDRESS      = "ipAddress";
    public static final String PREF_DEFAULT_GATEWAY = "defaultGateway";
    public static final String PREF_SUBNET_MASK     = "subnetMask";
    
    /**
     * The preferred window width
     */
    public static final double PREFERRED_WINDOW_WIDTH = 912;
    
    /**
     * The preferred window height
     */
    public static final double PREFERRED_WINDOW_HEIGHT = 500;
    
    /**
     * The preferred graph width
     */
    public static final double PREFERRED_GRAPH_WIDTH = 510;
    
    /**
     * The preferred graph height
     */
    public static final double PREFERRED_GRAPH_HEIGHT = 494;
    
    /**
     * Graph set upper anchor
     */
    public static final double UPPER_ANCHOR = 21;
    
    /**
     * Graph set lower anchor
     */
    public static final double LOWER_ANCHOR = 261;
    
    /**
     * Graph set 1's left anchor
     */
    public static final double BOX1_LEFT_ANCHOR = 232;
    
    /**
     * Graph set 2's left anchor
     */
    public static final double BOX2_LEFT_ANCHOR = 741;
    
    /**
     * Graph set 1's right anchor
     */
    public static final double BOX1_RIGHT_ANCHOR = 509;
    
    /**
     * Graph set 2's right anchor
     */
    public static final double BOX2_RIGHT_ANCHOR = 0;
    
    /**
     * Display new data at this rate.
     */
    private final double UI_UPDATE_HZ = 30.0;
    
    /**
     * Processes new data from wireless FT system.
     */
    private final ChangeListener<? super WirelessFTSample> m_sampleChangeListener = new ChangeListener<WirelessFTSample>() 
    {
        /**
         * Handles changes in the most-recently read WNet data sample.
         * 
         * @param observable The current sample.
         * @param oldValue   The previous sample.
         * @param newValue   The new sample.
         */
        @Override
        public void changed(ObservableValue<? extends WirelessFTSample> observable, WirelessFTSample oldValue, WirelessFTSample newValue) 
        {
            if (newValue != null) 
            {
                m_lastSample = newValue;                                  // Save the newest sample block.
            }

            for (WirelessFTSensorPanel panel : m_sensorPanels1) 
            {
                panel.setSensorData(m_lastSample);
            }

            if (m_sensorPanelBox2.isVisible()) 
            {
                for (WirelessFTSensorPanel panel : m_sensorPanels2) 
                {
                    panel.setSensorData(m_lastSample);
                }
            }
                        
            if (autoCalController != null)
            {
                int[][] data = m_lastSample.getFtOrGageData();
                autoCalController.setGageData(data);
            }
        }
    };
    
    /**
     * Processes user request to change type of data coming from sensor.
     */
    private final ChangeListener<? super Toggle> m_ftOrGageChangeListener = new ChangeListener<Toggle>() 
    {
        /**
         * Handles changes in the data view mode.
         * 
         * @param ov       The current view mode's toggle button.
         * @param oldValue The previous toggle button.
         * @param newValue The new toggle button.
         */
        @Override
        public void changed(ObservableValue<? extends Toggle> ov, Toggle oldValue, Toggle newValue) 
        {
            boolean forceTorqueButton = (newValue == m_toggleFTData); // True if the user selected F/T data type.
            
            if (forceTorqueButton) 
            {
                m_toggleFTData  .setSelected(true);
                m_toggleGageData.setSelected(false);
            } 
            else 
            {
                m_toggleFTData  .setSelected(false);
                m_toggleGageData.setSelected(true);
            }
            
            changeGageFT(forceTorqueButton);

            if (autoCalController != null)
            {
                autoCalController.setForceTorqueSetting(forceTorqueButton);
            }
        }
    };
    
    /**
     * Sets the WNet profile prior to initialization.
     * 
     * @param profile The profile from the startup window.
     * @param save Should these changes be saved to the WNet
     * itself at connect-time?
     */
    public void setProfile(WirelessFTDemoProfile profile, boolean save) 
    {
        m_profile     = profile;
        m_saveProfile = save;
        m_applyRate.setText(m_profile.m_rate); // Put current rate into the Apply Rate text field.
        m_vBoxPacketStats.setVisible(false);
    }

    /**
     * Initializes the controller class.
     * 
     * @param url Not used.
     * @param rb  Not used.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) 
    {
        m_sampleProperty.addListener(m_sampleChangeListener);
        m_toggleGroupGageOrFT.selectedToggleProperty().addListener(m_ftOrGageChangeListener);
        
        m_logger.addHandler(new Handler() 
        {
            @Override
            public void publish(LogRecord record) 
            {
                Platform.runLater(new UpdateErrorLog(record.getLevel().toString() + " - " + record.getMessage() + "(" + (new Date()).toString() + ")"));
            }

            @Override
            public void flush() 
            {
            }

            @Override
            public void close() throws SecurityException 
            {
            }
        });
        
        m_prefsRoot = Preferences.userRoot(); //set the user root
        // Set the user root node for the FT Demo in the registery. The absolute path name is
        // regedit\HKEY_CURRENT_USER\Software\JavaSoft\Prefs
        Preferences prefs = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        m_latestIP = prefs.get(PREF_LATEST_IP, ""); // Retrieve the latest IP Address the user accesed.
        prefs.put(PREF_LATEST_IP, m_latestIP); // Assign the preferences latest ip keyword to the latest IP
        
        double lastWidth  = prefs.getDouble(PREF_LAST_WINDOW_WIDTH,  m_anchorPaneTop.getPrefWidth());  //Retrieve the latest window width
        double lastHeight = prefs.getDouble(PREF_LAST_WINDOW_HEIGHT, m_anchorPaneTop.getPrefHeight()); //Retrieve the latest window height
        double lastX      = prefs.getDouble(PREF_LAST_X_POSITION,    100); //Retrieve the latest window x position
        double lastY      = prefs.getDouble(PREF_LAST_Y_POSITION,    100); //Retrieve the latest window y position
        
        prefs.putDouble(PREF_LAST_WINDOW_WIDTH,  lastWidth);  //Assign the preferences last width  keyword to the latest width
        prefs.putDouble(PREF_LAST_WINDOW_HEIGHT, lastHeight); //Assign the preferences last height keyword to the latest height
        prefs.putDouble(PREF_LAST_X_POSITION,    lastX);      //Assign the preferences last x keyword the latest x window position
        prefs.putDouble(PREF_LAST_Y_POSITION,    lastY);      //Assign the preferences last y keyword the latest y window position
        
        m_txtFieldSensorHostName.setText(prefs.get(PREF_LATEST_IP, m_latestIP)); //The latest ip address attempted is set as the label text each time the program is opened
        m_btnConnect            .setPrefWidth(Double.MAX_VALUE);
        m_btnDiscover           .setPrefWidth(Double.MAX_VALUE);
        m_btnCollectData        .setPrefWidth(Double.MAX_VALUE);
        
        m_txtFieldSensorHostName.textProperty().addListener(new ChangeListener<String>()
        {
            /**
             * Handles changes in the connection address and
             * whether or not the connect button should be active.
             * 
             * @param ov The current text value.
             * @param t  The previous text value.
             * @param t1 The new text value.
             */
            @Override
            public void changed(ObservableValue<? extends String> ov, String t, String t1) 
            {
                if (   m_readingRecords 
                    && m_txtFieldSensorHostName.getText().equals(Preferences.userRoot().node(PREF_USER_ROOT_NODE_PATHNAME).get(PREF_LATEST_IP, null))) 
                {
                    m_btnConnect.setDisable(true);
                } 
                else 
                {
                    m_btnConnect.setDisable(false);
                }
            }
        });
        
        m_transLabels = new Label[] {m_lblTransducer1, m_lblTransducer2, m_lblTransducer3, m_lblTransducer4, m_lblTransducer5, m_lblTransducer6};
    }
    
    /**
     * Opens window to extract .DAT files
     * from a connected MicroSD and rewrite
     * them locally as .CSV files.
     */
    @FXML
    protected void openSDWindow() 
    {
        try 
        {
            URL        url    = getClass().getResource("FileExtractScreen.fxml");
            FXMLLoader loader = new FXMLLoader(url);
            AnchorPane page   = (AnchorPane) loader.load();
            Scene      scene  = new Scene(page);
            Stage      stage  = new Stage();
            
            stage.setTitle("Extract File(s)");
            stage.setScene(scene);
            stage.setResizable(false);
            stage.showAndWait();
        } 
        catch (IOException e) 
        {
            m_logger.log(Level.WARNING, "Exception loading SD Data Extraction screen: {0}", e.getMessage());
        }
    }
    
    /**
     * Opens the window to download files to the WNet.
     */
    @FXML
    protected void openFileUploadWindow() 
    {
        try 
        {
            URL                        url        = getClass().getResource("FileUploadScreen.fxml");
            FXMLLoader                 loader     = new FXMLLoader(url);
            AnchorPane                 page       = (AnchorPane) loader.load();
            FileUploadScreenController controller = (FileUploadScreenController) loader.getController();
            
            controller.setDemoModel(m_model);
            
            Scene scene = new Scene(page);
            Stage stage = new Stage();
            
            stage.setTitle("Upload File(s)");
            stage.setScene(scene);
            stage.setResizable(false);
            stage.showAndWait();
        } 
        catch (IOException e)
        {
            m_logger.log(Level.WARNING, "Exception loading file upload screen: {0}", e.getMessage());
        }
    }
    
    /**
     * Opens the calibration view window.
     */
    @FXML
    protected void openCalibrationWindow() 
    {
        try 
        {
            URL                                       url        = getClass().getResource("WirelessFTCalibrationInfoScreen.fxml");
            FXMLLoader                                loader     = new FXMLLoader(url);
            AnchorPane                                page       = (AnchorPane) loader.load();
            WirelessFTCalibrationInfoScreenController controller = loader.getController();

            try 
            {
                controller.initializeCalibrationController(m_model, this);
            } 
            catch (CommSeveredException cse) 
            {
                disconnectButtonPressed();
            }
            
            Scene scene = new Scene(page);
            Stage stage = new Stage();
            
            stage.setTitle("Calibration Settings View");
            stage.setScene(scene);
            stage.setResizable(false);
            stage.showAndWait();
        } 
        catch(IOException e) 
        {
            m_logger.log(Level.WARNING, "Exception loading calibration screen: {0}", e.getMessage());
        }
    }
    
    /**
     * Opens the About window.
     */
    @FXML
    protected void openAboutWindow()
    {
        try 
        {
            URL                   url        = getClass().getResource("AboutScreen.fxml");
            FXMLLoader            loader     = new FXMLLoader(url);
            AnchorPane            page       = (AnchorPane) loader.load();
            AboutScreenController controller = loader.getController();

            try 
            {
                controller.initializeAboutScreen(m_model, m_connected);
            } 
            catch (CommSeveredException cse) 
            {
                disconnectButtonPressed();
            }
           
            Scene scene = new Scene(page);
            Stage stage = new Stage();
            
            stage.setTitle("About");
            stage.setScene(scene);
            stage.setResizable(false);
            stage.showAndWait();
        } 
        catch(IOException e) 
        {
            m_logger.log(Level.WARNING, "Exception loading About screen: {0}", e.getMessage());
        }
    }
    
    private AutoCalibrationScreenController autoCalController;
    
    /**
     * Opens the Auto-Cal window.
     */
    @FXML
    protected void openAutoCalWindow()
    {
        try 
        {
            URL        url    = getClass().getResource("AutoCalibrationScreen.fxml");
            FXMLLoader loader = new FXMLLoader(url);
            AnchorPane page   = (AnchorPane) loader.load();
            autoCalController =              loader.getController();

            try 
            {
                autoCalController.initializeAutoCalibrationScreen(m_model, this);
            } 
            catch (CommSeveredException cse) 
            {
                disconnectButtonPressed();
            }
            
            boolean forceTorque = m_toggleFTData.isSelected();    // Get Force/Torque setting from screen.
            autoCalController.setForceTorqueSetting(forceTorque); // Tell the voltage-calibration screen the force/torque setting.
           
            Scene scene = new Scene(page);
            Stage stage = new Stage();
            
            stage.setTitle("Voltage-Calibration");
            stage.setScene(scene);
            stage.setResizable(false);
            
            stage.setOnCloseRequest(new EventHandler<WindowEvent>() // Listen for the voltage-calibration screen to close.
            {
                @Override
                public void handle(WindowEvent t) 
                {
                    autoCalController.OnCloseRequest();
                }
            });

            stage.showAndWait();
        } 
        catch(IOException e) 
        {
            m_logger.log(Level.WARNING, "Exception loading AutoCalibrationScreen screen: {0}", e.getMessage());
        }
    }
    
    /**
     * Get conversion factors.
     */
    private void getConversionFactors(WirelessFTSensorPanel panel, int transducer) throws CommSeveredException
    {
        Calibration cal    = m_model.readActiveCalibration(); // Send CAL command to get calibration information.
        String      serial = cal.getSerialNumber();           // Get calibration serial number to label the graph with.
        String      date   = cal.getCalibrationDate();        // Get calibration date
        String      title  = "Transducer " + (transducer + 1) + " | " + serial; // Transducer is origin 1 to the user
        
        if (date.contains("1970/01"))
        {
            title = title + " UNCALIBRATED";
        }

        panel.setTitle(title); // Label the graph with the Transducer number and the calibration serial number.
        
        String forceUnits  = m_profile.getForceUnits ().toLowerCase(); // Get profile force  units
        String torqueUnits = m_profile.getTorqueUnits().toLowerCase(); // Get profile torque units
        
        if (forceUnits.equals("default"))                // If the force units are "default",
        {
            forceUnits = cal.getForceUnits();            // get the force units from the current calibration.
        }
        
        if (torqueUnits.equals("default"))               // If the force units are "default",
        {
            torqueUnits = cal.getTorqueUnits();          // get the force units from the current calibration.
        }
        
        double[] conversions = {1, 1, 1, 1, 1, 1};
        
        try
        {
            conversions = cal.getForceTorqueConversionFactors(forceUnits, torqueUnits, transducer);
        }
        catch (IllegalArgumentException e)
        {
            m_logger.log(new LogRecord(Level.WARNING, "Exception getting output F/T conversion factors, using factor of 1: " + e.getMessage()));
        }
        
        double forceConv  = conversions[0] / cal.getCountsPerUnitForce (); // force
        double torqueConv = conversions[3] / cal.getCountsPerUnitTorque(); // torque
        
        panel.setConversions     (forceConv,  torqueConv);
        panel.setForceTorqueUnits(forceUnits, torqueUnits);
    }
    
    /**
     * Refreshes information about active calibrations.
     */
    public void refreshCalibrationInformation() 
    {
        try 
        {
            boolean userMode   = m_model.SetTechnicianMode();            // Set technician mode
            int     transducer = 0;

            if (!m_sensorPanels1.isEmpty())                              // If the left-hand graph panel is in use,
            {
                for (int i = 0; i < m_sensorPanels1.size() && transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                {
                    if (m_model.m_sensorActive[transducer])              // If this transducer is active,
                    {
                        WirelessFTSensorPanel panel = m_sensorPanels1.get(i++);
                        m_model.setActiveSensor    (transducer);         // issue TRANS n command.
                        getConversionFactors(panel, transducer);         // Issue CAL command, parse the results and issue error messages as needed.
                    }
                }
            }
           
            if (!m_sensorPanels2.isEmpty())                              // If the right-hand graph panel is in use,
            {
                for (int i = 0; i < m_sensorPanels2.size() && transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                {
                    if (m_model.m_sensorActive[transducer])              // If this transducer is active,
                    {
                        WirelessFTSensorPanel panel = m_sensorPanels2.get(i++);
                        m_model.setActiveSensor    (transducer);         // issue TRANS n command.
                        getConversionFactors(panel, transducer);         // Issue CAL command, parse the results and issue error messages as needed.
                    }
                }
            }

            m_model.RestoreWnetMode(userMode);                           // restore Wnet mode
        } 
        catch (CommSeveredException exc) 
        {
            disconnectButtonPressed();
        }
    }
    
    /**
     * Connects to WNet when the enter key is pressed in the host text field.
     * 
     * @param ke The key pressed.
     */
    @FXML
    protected void txtFieldSensorHostNameEnter(KeyEvent ke) 
    {
        if (ke.getCode() == KeyCode.ENTER && !m_btnConnect.disabledProperty().get()) 
        {
            connectButtonPressed();  
        }
    }
    
    /**
     * Opens the Discovery Results window,
     * which continuously searches for active Wireless F/Ts
     * in range using the @RTADiscoveryProtocol package.
     * The window contains a list of results to choose from.
     */
    @FXML
    protected void discoverButtonPressed() 
    {
        try 
        { 
            // Open the Discovery Results window.
            URL        url    = getClass().getResource("DiscoveryResults.fxml");
            FXMLLoader loader = new FXMLLoader(url);
            AnchorPane page   = (AnchorPane) loader.load();
                  
            // Gives the Discovery Controller access to the text field on the main screen so results selected can be sent to it.
            final DiscoveryResultsController discoveryController;
            discoveryController = (DiscoveryResultsController) loader.getController();
            discoveryController.retrieveHostTextBox(m_txtFieldSensorHostName);
            discoveryController.DiscoverButtonPressedCommon(page);
        } 
        catch (IOException e) 
        {
            m_logger.log(Level.WARNING, "Could not display results: {0}", e.getMessage());
        }
    }    

    private final int MIN_UDP_RATE =    5;
    private final int MAX_UDP_RATE = 4000;
    
    @FXML
    protected void m_ApplyRateChanged()
    {
        m_btnApplyRatePressed();
    }
    
    @FXML
    protected Button m_btnApplyRate;
    
    @FXML
    protected void m_btnApplyRatePressed()
    {
        int packetRate;
        
        try 
        {
            packetRate = (int) Double.parseDouble(m_applyRate.getText()); // Get rate from screen
        } 
        catch (NumberFormatException n) 
        {
            packetRate = MIN_UDP_RATE;
        }

        if (packetRate > MAX_UDP_RATE) 
        {
            packetRate = MAX_UDP_RATE;
        }
            
        if (packetRate < MIN_UDP_RATE) 
        {
            packetRate = MIN_UDP_RATE;
        }

        m_applyRate.setText("" + packetRate); // Copy rate back to screen
        m_profile.setRate  (     packetRate); // Copy rate to profile
        
        try 
        {
            m_model.applyRate(packetRate);    // Copy rate to WNet unit
        } 
        catch (CommSeveredException ex)       // If the WNet is not connected,
        {                                     // do not do anything rash.
        }
    }

    // The possible LED color combinations.
    private final Color[] colors = new Color[] {Color.BLACK, 
                                                Color.web("#ff1400"),  // brighter red   than Color.RED
                                                Color.web("#14ff00"),  // brighter green than Color.GREEN
                                                Color.web("#ff7f00")}; // orange
    
    private final Color SATRED = Color.web("#cc0000"); // Saturation red
                        
    /**
     * Called when the Show Packet Statistics CheckBox changes.
     */
    @FXML
    protected void ShowPacketStatsChanged()
    {
        boolean selected = m_chkShowPacketStats.isSelected();
        m_vBoxPacketStats.setVisible(selected);
    }
    
    /**
     * Connects to the WNet.
     */
    @FXML
    protected void connectButtonPressed() 
    {
        if (!m_connected) // If we are not now connected,
        {
            try           // try to connect.
            {
                // Save the IP address for future attempts.
                Preferences.userRoot().node("com.FTDemo.preference.Settings").put(PREF_LATEST_IP, m_txtFieldSensorHostName.getText());
                String ip = m_txtFieldSensorHostName.getText();
                m_model.connect(ip, m_profile, m_saveProfile, this);
                
                m_readingRecords = true;
                new Thread(new CollectDataThread()).start(); // Start collecting records from the WNet.
               
                setupPanels(); // Create panels for each sensor.
                refreshCalibrationInformation();
                
                // Set Gage or FT data based on previous run (Gage if no previous run).
                Preferences prefs  = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
                boolean     ftData = prefs.getBoolean(PREF_GAGE_OR_FT, false);

                setOnlineMode(true, ftData); // Switch to online UI mode and set data type.
                changeGageFT(ftData);
                
                // Set up the logger.
                m_logger.log(Level.INFO, "{0} established a connection.", Preferences.userRoot().node(PREF_USER_ROOT_NODE_PATHNAME).get(PREF_LATEST_IP, ""));
               
                //FetchCalInfoTask fetchTask = new FetchCalInfoTask(); // Issue CAL commands and get responses
                //m_dataCollectorThread      = new Thread(fetchTask);
                //m_dataCollectorThread.start();
               
                m_btnConnect.setText("Disconnect"); // We have a connection, change the button's functionality.
                m_connected = true;
               
                m_animation.getKeyFrames().clear(); // Begin animating.
                
                m_animation.getKeyFrames().add(new KeyFrame(Duration.millis(1000 / UI_UPDATE_HZ), new EventHandler<ActionEvent>() 
                {
                    /**
                     * Handles changes in the LEDs, their labels,
                     * and the graphs for each transducer based
                     * on the newest sensor packet data.
                     * 
                     * @param actionEvent Not used.
                     */
                    @Override
                    public void handle(ActionEvent actionEvent) 
                    {
                        if (m_lastSample == null)
                        {
                            return;
                        }
                        
                        int status1 = m_lastSample.getStatusCode1();
                        int status2 = m_lastSample.getStatusCode2();
                       
                        m_transducer1LED  .setFill(colors[status1       & 0x3]); // Update LEDs
                        m_transducer2LED  .setFill(colors[status1 >>  2 & 0x3]);
                        m_transducer3LED  .setFill(colors[status1 >>  4 & 0x3]);
                        m_wifiLED         .setFill(colors[status1 >>  6 & 0x3]);
                        m_externalPowerLED.setFill(colors[status1 >>  8 & 0x3]);
                        m_batteryLED      .setFill(colors[status1 >> 10 & 0x3]);
                        
                        m_transducer4LED  .setFill(colors[status2       & 0x3]);
                        m_transducer5LED  .setFill(colors[status2 >>  2 & 0x3]);
                        m_transducer6LED  .setFill(colors[status2 >>  4 & 0x3]);
                                
                        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) // Update saturation flags
                        {
                            int     status    = (transducer < 3) ? status1 : status2;
                            boolean saturated = (status >> (24 + transducer % 3) & 0x1) == 1;
                            Color   color;
                            String  suffix;
                            
                            if (saturated)                     // If the Transducer is saturated,
                            {
                                color  = SATRED;               // Set color  to Saturation red
                                suffix = " SAT";               // Set suffix to SAT
                            } 
                            else                               // If the Transducer is not saturated,
                            {
                                color  = Color.BLACK;          // Set color  to black
                                suffix = "    ";               // Set suffix to blank
                            }
                            
                            m_transLabels[transducer].setTextFill(color);
                            m_transLabels[transducer].setText("Transducer " + (transducer + 1) + suffix); // Transducer is origin 1 to the user
                        }
                        
                        for (WirelessFTSensorPanel m_sensorPanels11 : m_sensorPanels1) // Update graphs(s)
                        {
                            m_sensorPanels11.updatePlot();
                        }
                        
                        if (m_sensorPanelBox2.isVisible()) 
                        {
                            for (WirelessFTSensorPanel m_sensorPanels21 : m_sensorPanels2) 
                            {
                                m_sensorPanels21.updatePlot();
                            }
                        }
                        
                        float avgMissed  =  100.0f * m_rxedPacketsAcc / m_rxedPacketsTc; // Update statistics.
                        float packetRate = 1000.0f * m_timeTc         / m_timeAcc;

                        m_DisplayPackets    .setText(String.format("%10d",   m_packets));
                        m_DisplayRate       .setText(String.format("%10.0f", packetRate));
                        m_DisplayLatency    .setText(String.format("%10d",   m_lastSample.getLatency()));
                        m_DisplayDrops      .setText(String.format("%10d",   m_drops));
                        m_DisplayPktDrops   .setText(String.format("%10d",   m_missedPackets));
                        m_DisplayDropRate   .setText(String.format("%10.2f", avgMissed));
                        m_DisplayOutOfOrders.setText(String.format("%10d",   m_OutOfOrders));
                        m_DisplayDups       .setText(String.format("%10d",   m_Duplicates));
                    }  
                }));
                
                m_animation.setCycleCount(Animation.INDEFINITE);
                m_animation.play();
                
                m_LastPacketTime = System.currentTimeMillis();
                m_packets        = 0;
                m_drops          = 0;
                m_missedPackets  = 0;
                m_rxedPacketsAcc = 0;
                m_OutOfOrders    = 0;
                m_Duplicates     = 0;
            } 
            catch (IOException | CommSeveredException e) 
            {
                e.printStackTrace();
                
                try // Suggest using the Discovery button.
                {
                    PopupDialogController.PopUpDialogBox("Connect timed out", 
                        "Could not connect to IP address: " 
                        + m_txtFieldSensorHostName.getText() 
                        + ". If DHCP is on, the device's IP address may have changed; consider using the \"Discover\" button.");
                } 
                catch (IOException ioe) 
                {
                    m_logger.log(Level.SEVERE, "Exception connecting to WNET at IP Address {0} from main screen: {1}", new Object[]{m_txtFieldSensorHostName.getText(), e.getMessage()});
                }           
            }
        } 
        else 
        { // We are already connected, the user clicked "Disconnect".
            disconnectButtonPressed();
        }
    }
    
    /**
     * Changes active UI controls based on whether
     * or not we've connected with a WNet.
     * 
     * @param connected Are we connected to the WNet?
     * @param ftData    If we're connected, are we displaying
     *                  F/T data or gage data?
     */
    private void setOnlineMode(boolean connected, boolean ftData) 
    {
        m_btnDiscover            .setDisable( connected);
        m_btnCollectData         .setDisable(!connected);
        m_btnApplyRate           .setDisable(!connected);
        m_applyRate              .setDisable(!connected);
        m_toggleFTData           .setDisable(!connected);
        m_toggleGageData         .setDisable(!connected);
        m_menuItemSD             .setDisable( connected);
        m_menuItemFirmwareUpgrade.setDisable(!connected);
        m_menuItemCalibration    .setDisable(!connected);
        m_menuItemAutoCalibration.setDisable(!connected);
        
        if (connected) 
        {
            m_toggleFTData  .setSelected( ftData);
            m_toggleGageData.setSelected(!ftData);
        }
    }
    
    /**
     * Dynamically creates panels to maximize space
     * when displayed on the main screen.
     */
    private void setupPanels() 
    {
        int sensors = 0;
        
        for (String m_xpwr : m_profile.m_xpwr) // For each Transducer,
        {
            if (m_xpwr.equals("ON"))           // that is active,
            {
                sensors++;                     // count it.
            }
        }
        
        m_sensorPanelBox1.setVisible(false);
        m_sensorPanelBox2.setVisible(false);
        
        if(!m_graphGrid.getChildren().contains(m_sensorPanelBox1)) 
        {
            m_graphGrid.getChildren().add     (m_sensorPanelBox1);
        }
        
        if(!m_graphGrid.getChildren().contains(m_sensorPanelBox2)) 
        {
            m_graphGrid.getChildren().add     (m_sensorPanelBox2);
        }
        
        m_sensorPanelBox1.getChildren().clear();
        m_sensorPanelBox2.getChildren().clear();
        
        GridPane.setColumnSpan(m_sensorPanelBox1, 1);

        WirelessFTSensorPanel panel;
        
        int max  = 9999;
        int rows = (sensors == 4) ? 2 : 3;
        
        // Create graphs for each Transducer and add them to the screen.
        for (int transducer = 0; transducer < m_profile.m_xpwr.length; transducer++) // For each Transducer,
        {
            if (m_profile.m_xpwr[transducer].equals("ON"))              // that is active,
            {
                panel = CreatePanel();                                  // create a panel for it,
                panel.setSensorIndex(transducer);                       // and save its index.
                panel.setPrefHeight(max);
            
                if (m_sensorPanelBox1.getChildren().size() < rows)      // If there is room on the left side,
                {
                    m_sensorPanels1.add(panel);                         // setup the data source,
                    m_sensorPanelBox1.getChildren().add(panel);         // and put the panel there,
                } 
                else if (m_sensorPanelBox2.getChildren().size() < rows) // else if there is room on the right side,
                {
                    m_sensorPanels2.add(panel);                         // setup the data source,
                    m_sensorPanelBox2.getChildren().add(panel);         // and put the panel there.
                } 
            }
        }
        
        if (sensors >= 1 && sensors <= 3)        // If there are 1 to 3 transducers,
        {
            m_graphGrid.getChildren().remove(1); // remove the right side panel from the screen.
        }
        
        m_sensorPanelBox1.setVisible(true);
        m_sensorPanelBox2.setVisible(true);
        
        m_graphGrid.layout(); // Redraw the graphs.
    }
    
    /**
     * Create a panel, and handle its Bias and Un-bias buttons.
     */
    private WirelessFTSensorPanel CreatePanel()
    {
        WirelessFTSensorPanel panel;
        panel = new WirelessFTSensorPanel(m_profile);
            
        try 
        {
            panel.setOnBiasRequested(new EventHandler<ActionEvent>() 
            {
                @Override
                public void handle(ActionEvent e) 
                {
                    try 
                    {
                        m_model.biasSensor(((WirelessFTSensorPanel) e.getSource()).getSensorIndex());
                    } 
                    catch (CommSeveredException cse) 
                    {
                        disconnectButtonPressed();
                    }
                }
            });
                
            panel.setOnUnbiasRequested(new EventHandler<ActionEvent>() 
            {
                @Override
                public void handle(ActionEvent n) 
                {
                    try 
                    {
                        m_model.unbiasSensor(((WirelessFTSensorPanel) n.getSource()).getSensorIndex());
                    } 
                    catch (CommSeveredException ex) 
                    {
                        disconnectButtonPressed();
                    }
                }
            });
        } 
        catch (CommSeveredException cse) 
        {
            disconnectButtonPressed();
        }
            
        return panel;
    }
    
    /**
     * Close the UDP stream and TCP connection to the
     * current device, then clears the graphs from the GUI.
     */
    public void disconnectButtonPressed() 
    {
        stopCollectingData();
        m_readingRecords = false;
        
        if (m_threadActive) 
        {
            try 
            {
                Thread.sleep(1000);
            } 
            catch (InterruptedException ie) 
            {
                m_logger.log(Level.SEVERE, "Exception closing connection: {0}", ie.getMessage());
            }
        }
       
        m_model.disconnect();        // Close TCP/UDP connections to the sensor.
       
        try
        {
            setOnlineMode(false, false); // Switch to offline UI. // This function really should be called from a different thread
        }
        catch (IllegalStateException e)  // kludge
        {
        }
       
        m_animation.stop();          // Clear graphs.
        m_sensorPanels1.clear();
        m_sensorPanels2.clear();
        
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
        {
            m_transLabels[transducer].setTextFill(Color.BLACK);
        }
        
        // Set all LEDs to black.
        Circle[] statusLEDs1 = new Circle[]{m_transducer1LED, m_transducer2LED, m_transducer3LED, m_wifiLED, m_externalPowerLED, m_batteryLED};
        Circle[] statusLEDs2 = new Circle[]{m_transducer4LED, m_transducer5LED, m_transducer6LED};
        
        for (Circle statusLEDs11 : statusLEDs1) 
        {
            statusLEDs11.setFill(Color.BLACK);
        }
        
        for (Circle statusLEDs21 : statusLEDs2) 
        {
            statusLEDs21.setFill(Color.BLACK);
        }
        
        Platform.runLater(new Thread()  // Make UI functions thread-safe.
        {
            @Override
            public void run() 
            {
                m_graphGrid.getChildren().remove(m_sensorPanelBox1);
                m_graphGrid.getChildren().remove(m_sensorPanelBox2);
                m_btnConnect.setText("Connect");
            }
        });
        
        m_connected = false;
    }
    
    /**
     * Called when user presses button to
     * choose a data collection file.
     */
    @FXML
    protected void chooseDataCollectionFilePressed() 
    {
        Preferences prefs       = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname    = prefs.get(PREF_DATA_COLLECTION_DIRECTORY, null);
        
        File        collectFile;
        FileChooser chooser     = new FileChooser();
        
        chooser.setTitle("Choose File to Save Data");
        chooser.getExtensionFilters().add(new ExtensionFilter("CSV Files",  "*.csv"));
        chooser.getExtensionFilters().add(new ExtensionFilter("Text Files", "*.txt"));
        
        if (pathname != null) 
        {
            try 
            {
                chooser.setInitialDirectory(new File(pathname));
                collectFile = chooser.showSaveDialog(((Node)m_btnDots).getScene().getWindow());
            } 
            catch(Exception e) 
            {
                // User's last preferred directory was deleted.
                chooser.setInitialDirectory(null);
                collectFile = chooser.showSaveDialog(((Node)m_btnDots).getScene().getWindow());
            }
        } 
        else 
        {
            // No preferred directory (first use).
            chooser.setInitialDirectory(null);
            collectFile = chooser.showSaveDialog(((Node)m_btnDots).getScene().getWindow());
        }
        
        if (collectFile != null) 
        {
            String filename = collectFile.getAbsolutePath();
            if (   !filename.toLowerCase().endsWith(".csv") 
                && !filename.toLowerCase().endsWith(".txt"))
            {
                filename += ".csv";
            }

            m_txtFilePath.setText(filename);
            
            if (collectFile.getParentFile().exists()) 
            {
                prefs.put(PREF_DATA_COLLECTION_DIRECTORY, collectFile.getParentFile().getAbsolutePath());
            }
        }
    }

    /**
     * Called when button to collect data is pressed.
     */
    @FXML
    protected void collectDataButtonPressed() 
    {
        if (m_collectingData) 
        {
            stopCollectingData();
        } 
        else 
        {
            String filename;
            String extension;
            String date;
            int    extIndex;
            
            try 
            {
                // Append date/time to prevent name collisions.
                filename = m_txtFilePath.getText();
                extIndex = filename.lastIndexOf("."); // Find last . (if any)

                if (extIndex < 0)
                {
                    extension = ".csv";
                }
                else
                {
                    extension = filename.substring(extIndex, filename.length());
                    filename  = filename.substring(0,        extIndex);
                }
                
                date     = Calendar.getInstance().getTime().toString().replace(':', '-');
                filename = filename + "(" + date + ")" + extension;
                
                startCollectingData(filename);
            } 
            catch (IllegalStateException | IOException e) 
            {
                String message = "Exception starting data collection: " + e.getMessage();
                
                try 
                {
                    PopupDialogController.PopUpDialogBox("Data collection failed", message);
                } 
                catch(IOException ioe) 
                {
                    m_logger.log(Level.WARNING, message);
                }
            }
        }

        /* Update the text in a separate if-statement in case there was an exception
         * while starting or stoppping data collection which prevented us from 
         * actually transitioning to the opposite state.
         */
        if (m_collectingData) 
        {
            m_btnCollectData.setText("Stop");
        } 
        else 
        {
            m_btnCollectData.setText("Collect Data");
        }
    }
    
    /**
     * Begins writing data to a file by setting the collect data flag
     * and inserting the header into a new .csv or .txt file.
     * 
     * @param filePath The path of the file in which we collect data.
     * @throws IllegalStateException If data collection is already in progress.
     * @throws IOException if there is an error opening the specified file for
     * writing.
     */
    public void startCollectingData(String filePath) throws IllegalStateException, IOException 
    {
        if (m_collectingData) 
        {
            throw new IllegalStateException("Data collection already in progress.");
        }
        synchronized (m_bufferedWriterSynchroLock) 
        {
            m_bufferedWriter = Files.newBufferedWriter(Paths.get(filePath), Charset.forName("US-ASCII"));
            /* Write column headers. */
            m_bufferedWriter.write("Time, Mask, Bat, Sts1, Sts2, Seq #");
            String[] channelNames = new String[]{"FX", "FY", "FZ", "TX", "TY", "TZ"};
            String[] gageNames    = new String[]{"G0", "G1", "G2", "G3", "G4", "G5"};
            
            for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
            {
                if (m_model.m_sensorActive[transducer])
                {
                    for (int axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++)
                    {
                        m_bufferedWriter.write(",T" 
                                + (transducer + 1) // Transducer is origin 1 to the user
                                + (m_toggleFTData.isSelected() ? channelNames[axis] : gageNames[axis]) );
                    }
                }
            }
            m_bufferedWriter.write("\n");
        }
        m_collectingData = true;
    }
    
    /**
     * Continuously reads samples from the WNet and (optionally)
     * writes them to a .txt or .csv file.
     */
    private class CollectDataThread implements Runnable 
    {
        /**
         * Separate from the UDP receive timeout, 
         * this is the longest time the Demo is
         * allowed to run with no new data.
         */
        private final long THRESHOLD = 60000;
       
        private boolean warningShown = false; // Did we fail to retrieve the last sample buffer?
        private boolean reconnecting = false; // Are we trying to re-establish communication?
        
        /**
         * Begin reading records.
         * If the time between records
         * exceeds the 1-minute timeout threshold, attempt to
         * handle a disconnect scenario.
         * If the file-write flag is set, also record each of
         * these records to the user-specified .csv or .txt file.
         */
        @Override
        public void run() 
        {
            m_threadActive   = true;
            Date lastSuccess = new Date();
            
            while (m_readingRecords) 
            {
                try 
                {
                    ArrayList<WirelessFTSample> samples = m_model.m_sensor.readStreamingSamples(); // Get all packets from next UDP data block

                    warningShown = false;                 // Response recieved, reset the message flag.
                    
                    for (WirelessFTSample s : samples)    // For each sample block in the UDP data block,
                    {
                        CalculateDataBlockStatistices(s); // calculate statistics.
                        
                        m_sampleProperty.set(s);          // Put the data where the listener can plot it.
                        
                        if (m_collectingData) 
                        {
                            synchronized (m_bufferedWriterSynchroLock) 
                            {
                                WriteDataBlockToFile(s);
                            }
                        }
                    }
                    
                    if (reconnecting) 
                    {
                        reconnecting = false;
                        m_logger.log(Level.INFO, "Connection re-established.");
                    }
                    
                    lastSuccess = new Date();
                } 
                catch (IOException exc)  // UDP socket timeout.
                {
                    reconnecting     = true;
                    Date currentTime = new Date();
                    
                    if (currentTime.getTime() - lastSuccess.getTime() > THRESHOLD) 
                    {
                        m_logger.log(Level.SEVERE, "Could not re-establish connection: {0}", exc.getMessage());
                        try
                        {
                            disconnectButtonPressed(); // This function really should be called from the FX application thread
                        }
                        catch (Exception e) // kludge
                        {
                        }
                        break;
                    } 
                    else 
                    {
                        if (!warningShown) 
                        {
                            m_logger.log(Level.WARNING, "Connection lost, attempting to re-establish UDP ...");
                            warningShown = true;
                        }
                    }
                }
            }
            
            m_threadActive = false;
        }
    }
    
    private void CalculateDataBlockStatistices(WirelessFTSample s)
    {
        long thisSequence = s.getSequence();                      // Get the new sequence number
            
        if (thisSequence > m_LastSequence)                        // If this packet is in order,
        {         
            m_packets++;                                          // Count the packet
            long missed      = thisSequence - m_LastSequence - 1; // Calculate number of missed packets (should be 0)
            m_missedPackets += missed;                            // Calculate total number of missed packets
            if (missed > 0)                                       // If any packets were missed,
            {
                m_drops++;                                        // count it as a drop event.
                for (int i = 0; i < missed; i++)
                {
                    m_rxedPacketsAcc += 1.0f - (m_rxedPacketsAcc / m_rxedPacketsTc); // single-pole IIR filter
                }
            }
            m_rxedPacketsAcc += 0.0f - (m_rxedPacketsAcc / m_rxedPacketsTc); // single-pole IIR filter
                            
            long currentTime  = System.currentTimeMillis();       // Calculate packet rate
            long deltaTime    = currentTime - m_LastPacketTime;
            m_LastPacketTime  = currentTime;
            m_timeAcc        += deltaTime - (m_timeAcc / m_timeTc); // single-pole IIR filter
        }
        else if (thisSequence < m_LastSequence)                   // If this packet is out-of-order,
        {
            m_OutOfOrders++;                                      // count it.
        }
        else                                                      // If the packet was a duplicate,
        {
            m_Duplicates++;                                       // count it.
        }

        m_LastSequence = thisSequence;                            // Save sequence number for next time
    }

    private void WriteDataBlockToFile(WirelessFTSample s) throws IOException
    {
      //boolean FTSelected = m_toggleFTData.isSelected();
        
        m_bufferedWriter.write(String.format("%d, %2x, %d, %8x, %8x, %d",
                         s.getTimeStamp(), 
                         s.getSensorMask(),
                         (int) s.getBatteryLevel(),
                         s.getStatusCode1(), 
                         s.getStatusCode2(),
                         s.getSequence()));
                                
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) // For each Transducer,
        {
            if (m_model.m_sensorActive[transducer])                              // If this Transducer is active,
            {
                double[] data = matrixMult(m_profile.getTransformationMatrix(transducer), s.getFtOrGageData()[transducer]);
                                        
                for (int axis = 0; axis < WirelessFTDemoModel.NUM_AXES; axis++)  // For each channel,
                {
                    double value = data[axis];                                   // get the data value.
                                               
                    m_bufferedWriter.write(", " + Double.toString(value));       // Save converted value to .csv file.
                }
            }
        }
        
        m_bufferedWriter.write("\n");
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
    protected static double[] matrixMult(double trans[][], int[] gage)
    {
        if (trans[0].length != gage.length) return null; // invalid dims

        double ans[] = new double[WirelessFTDemoModel.NUM_AXES];

        for (int i = 0; i < WirelessFTDemoModel.NUM_AXES; i++) 
        {
            for (int j = 0; j < WirelessFTDemoModel.NUM_AXES; j++) 
            {
                ans[i] += trans[i][j] * gage[j];
            }
        }
                 
        return ans;
    }
    
    /**
     * Closes the file to which data is being
     * collected and stops writing data.
     */
    public void stopCollectingData() 
    {
        m_collectingData = false;
        m_LastPacketTime = System.currentTimeMillis();
        m_packets        = 0;
        m_drops          = 0;
        m_missedPackets  = 0;
        m_rxedPacketsAcc = 0;
        m_OutOfOrders    = 0;
        m_Duplicates     = 0;
        
        if (m_bufferedWriter != null) 
        {
            synchronized (m_bufferedWriterSynchroLock) 
            {
                try 
                {
                    m_bufferedWriter.close();
                } 
                catch (IOException e) 
                {
                    m_logger.log(Level.WARNING, "Exception closing data file: {0}", e.getMessage());
                }
            }
        }
    }
    
    /**
     * Change data between Gage and FT displays.
     * 
     * @param forceTorqueButton true if forceTorque button is selected
     */
    private void changeGageFT(boolean forceTorqueButton) 
    {
        try 
        {
            m_model.selectGageOrFTData(forceTorqueButton);
            
            for (WirelessFTSensorPanel panel : m_sensorPanels1) 
            {
                panel.setDataDisplay(forceTorqueButton);
            }
            
            if (!m_sensorPanels2.isEmpty())
            {
                for (WirelessFTSensorPanel panel : m_sensorPanels2) 
                {
                    panel.setDataDisplay(forceTorqueButton);
                }
            }
        } 
        catch (CommSeveredException cse) 
        {
            disconnectButtonPressed();
        }

        // Change the preferred setting.
        Preferences prefs = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        prefs.putBoolean(PREF_GAGE_OR_FT, forceTorqueButton);
    }
    
    /**
     * Updates error log on main screen. Intended to be used with
     * Platform.runLater to make it thread safe.
     */
    private class UpdateErrorLog implements Runnable 
    {
        /**
         * The message to log.
         */
        private final String m_message;

        public UpdateErrorLog(String message) 
        {
            m_message = message;
        }

        @Override
        public void run() 
        {
            m_listLogEvents.getItems().add(0, m_message);
        }
    }
    
    /**
     * Disconnects from the WNet automatically
     * when the application is closed.
     */
    @FXML
    public void OnCloseRequest() 
    {
        if (m_connected) 
        {
            disconnectButtonPressed();
        }
    }
}
