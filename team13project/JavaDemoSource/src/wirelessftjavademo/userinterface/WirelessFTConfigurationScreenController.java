/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.File;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.SingleSelectionModel;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import org.w3c.dom.DOMException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftjavademo.WirelessFTDemoProfile;

/**
 * FXML Controller class
 *
 * @author Sam Skuce, Chris Collins
 */
public class WirelessFTConfigurationScreenController implements Initializable 
{
    // ---------- CONSTANTS ---------- \\
    
    private final int MIN_UDP_RATE =    5;
    private final int MAX_UDP_RATE = 4000;
    
    /**
     * The largest valid value for oversampling.
     */
    private final int MAX_OVERSAMPLING = 32;
    
    /**
     * The maximum amount of averaging samples
     * allowed per transducer.
     */
    private final int MAX_TAPS = 32;
    
    /**
     * The maximum IIR time constant
     * allowed per transducer.
     */
    private final int MAX_TIME_CONSTANT = 32767;
    
    private final String[] m_forceLabels  = {"Default", "lbf",    "klbf",   "N",   "kN",   "g",     "kg"  }; // Force  ComboBox entries
    private final String[] m_torqueLabels = {"Default", "lbf-in", "lbf-ft", "N-m", "N-mm", "kg-cm", "kN-m"}; // Torque ComboBox entries
    
    private final int FIRST_PAGE = 0;
    private final int MODEL_PAGE = 1;
    private final int NTP_PAGE   = 2;
    private final int FCU_PAGE   = 3;
    private final int TOOL_PAGE  = 4;
    private final int MAX_PAGE   = 5; // The largest page index possible.
    
    // ---------- Fields ---------- \\
    
    /**
     * The current wizard page index.
     */
    private int m_pageIndex = 0;
    
    /**
     * The file to edit/create.
     */
    private File m_profileXML;
    
    /**
     * This dummy profile represents the
     * state of the Wizard's UI and is
     * used to write the XML for the new
     * profile when the Finish button is
     * pressed.
     */
    private WirelessFTDemoProfile m_wizardProfile = new WirelessFTDemoProfile();
    
    /**
     * The active transducers as a list.
     */
    private final ObservableList<String> m_FilCalTrans = FXCollections.observableArrayList();
    private final ObservableList<String> m_ToolTrTrans = FXCollections.observableArrayList();
    
    /**
     * The force units as a list.
     */
    private final ObservableList<String> m_forceUnits = FXCollections.observableArrayList();
    
    /**
     * The torque units as a list.
     */
    private final ObservableList<String> m_torqueUnits = FXCollections.observableArrayList();
    
    /**
     * The active transducer and calibrations for which
     * settings are being changed.
     */
    private int m_filCalTransducer;
    
    /**
     * The active transducer for which
     * transformation settings are being changed.
     */
    private int m_ToolTransformTransducer;
    
    // ---------- FXML Controls ---------- \\
    
    // Navigation Controls
    @FXML
    Button m_btnCancel;
    
    @FXML
    Button m_btnBack;
    
    @FXML
    Button m_btnNext;
    
    @FXML
    Button m_btnFinish;
   
    @FXML
    AnchorPane m_paneWelcome; // Welcome page
   
    @FXML
    AnchorPane m_paneModel;   // WNet Model page
    
    @FXML
    AnchorPane m_paneNTP;
    
    @FXML
    ToggleButton m_wnet3Toggle;
    
    @FXML
    ToggleButton m_wnet6Toggle;
    
    @FXML
    ToggleButton m_t1Toggle;
    
    @FXML
    ToggleButton m_t2Toggle;
    
    @FXML
    ToggleButton m_t3Toggle;
    
    @FXML
    ToggleButton m_t4Toggle;
    
    @FXML
    ToggleButton m_t5Toggle;
    
    @FXML
    ToggleButton m_t6Toggle;
    
    @FXML
    ComboBox m_cmbForce;
    
    @FXML
    ComboBox m_cmbTorque;
    
    @FXML
    TextField m_txtRate;
    
    @FXML
    Label m_udpRateLabel;
    
    @FXML
    TextField m_txtOversamp;
    
    @FXML
    CheckBox m_chkSD;
    
    @FXML
    AnchorPane m_paneFilCal; // Filter/Calibration Controls
    
    @FXML
    ComboBox m_cmbFilCalTrans;
    
    @FXML
    RadioButton m_rbNone;
    
    @FXML
    RadioButton m_rbMean;
    
    @FXML
    RadioButton m_rbMedian;
    
    @FXML
    RadioButton m_rbIIR;
    
    @FXML
    Label m_lblAveraging;
    
    @FXML
    TextField m_txtAveraging;
    
    @FXML
    RadioButton m_rbDefault;
    
    @FXML
    RadioButton m_rb1;
    
    @FXML
    RadioButton m_rb2;
    
    @FXML
    RadioButton m_rb3;
   
    @FXML
    AnchorPane m_paneTT; // Tool Transform Controls
    
    @FXML
    ToggleButton m_toggleButtonIN;
            
    @FXML
    ToggleButton m_toggleButtonFT;
            
    @FXML
    ToggleButton m_toggleButtonM;
            
    @FXML
    ToggleButton m_toggleButtonCM;
            
    @FXML
    ToggleButton m_toggleButtonMM;
    
    @FXML
    TextField m_txtDisplacementX;
    
    @FXML
    TextField m_txtDisplacementY;
    
    @FXML
    TextField m_txtDisplacementZ;
    
    @FXML
    ToggleButton m_toggleButtonDEGREES;
    
    @FXML
    ToggleButton m_toggleButtonRADIANS;
    
    @FXML
    TextField m_txtRotationsX;
    
    @FXML
    TextField m_txtRotationsY;
    
    @FXML
    TextField m_txtRotationsZ;
    
    @FXML
    ComboBox m_cmbTTTrans;
    
    @FXML
    Button m_btnReset;
    
    @FXML
    AnchorPane m_paneReady; // Completion Controls
    
    @FXML
    CheckBox m_chkDefault;
    
    @FXML
    CheckBox m_chkUseNtp;
    
    @FXML
    CheckBox m_chkUseDst;
    
    @FXML
    TextField m_txtNtpServer;
    
    @FXML
    TextField m_txtZoneHours;
    
    @FXML
    TextField m_txtZoneMinutes;
    
    @FXML
    TextArea m_Notes;
    
    // ---------- FXML Control Groups ---------- \\
    
    private       AnchorPane[]     m_pages;
    
    private final ToggleGroup      m_wnetToggleGroup = new ToggleGroup();
    
    private       ToggleButton[]   m_transducers;
    
    private       RadioButton[][]  m_filCalRBs;
    
    private final ToggleGroup[]    m_filCalRBToggleGroup = {new ToggleGroup(), new ToggleGroup()};
    
    private       ToggleButton[][] m_toolTransformUnits;
    
    private final ToggleGroup[]    m_toolTransformSettingsToggleGroup = {new ToggleGroup(), new ToggleGroup()};
    
    private       TextField[][]    m_toolTransformAxisNumbers;
    
    private       StartupScreenController m_parent = null;
    
    // ---------- METHODS ---------- \\
    
    /**
     * Sets the profile for this
     * controller to modify.
     * 
     * @param xmlFile The xml file that
     * corresponds to the profile.
     * @param readFile Should the wizard profile's
     * contents be populated from this file?
     * @param parent The Startup Screen which
     * created this wizard. Used EXCLUSIVELY to
     * set the makeDefaultProfile flag.
     * @throws Exception if a profile cannot be
     * created from the given file to read.
     */
    public void setProfile(File xmlFile, boolean readFile, StartupScreenController parent) throws Exception 
    {
        m_parent     = parent;
        m_profileXML = xmlFile;
        
        if (readFile) // If there is an existing profile,
        {
            m_wizardProfile = new WirelessFTDemoProfile(xmlFile); // read it.
        }

        setupWizard(); // Copy the settings to the working variables.
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
        m_pages = new AnchorPane[] {m_paneWelcome, m_paneModel, m_paneNTP, m_paneFilCal, m_paneTT, m_paneReady};
        
        for (AnchorPane a : m_pages) // Set all pages not visible
        {
            a.setVisible(false);
        }
        
        m_paneWelcome.setVisible(true); // Set the Welcome panel visible
    
        m_wnet3Toggle.setToggleGroup(m_wnetToggleGroup);
        m_wnet6Toggle.setToggleGroup(m_wnetToggleGroup);
        
        m_transducers = new ToggleButton[] 
        {
            m_t1Toggle, m_t2Toggle, m_t3Toggle, m_t4Toggle, m_t5Toggle, m_t6Toggle
        };
    
        m_filCalRBs = new RadioButton[][] 
        {
            {m_rbNone,    m_rbMean, m_rbMedian, m_rbIIR}, 
            {m_rbDefault, m_rb1,    m_rb2,      m_rb3}
        };
    
        m_toolTransformUnits = new ToggleButton[][] 
        {
            {m_toggleButtonIN,      m_toggleButtonFT, m_toggleButtonM, m_toggleButtonCM, m_toggleButtonMM},
            {m_toggleButtonDEGREES, m_toggleButtonRADIANS}
        };

        m_toolTransformAxisNumbers = new TextField[][] 
        {
            {m_txtDisplacementX, m_txtDisplacementY, m_txtDisplacementZ},
            {m_txtRotationsX,    m_txtRotationsY,    m_txtRotationsZ}
        };
        
        m_forceUnits .addAll(m_forceLabels); 
        m_torqueUnits.addAll(m_torqueLabels);
        
        m_cmbForce .setItems(m_forceUnits);  // Add entries to the Force  ComboBox
        m_cmbTorque.setItems(m_torqueUnits); // Add entries to the Torque ComboBox
        
        for (int i = 0; i < m_filCalRBs.length; i++) // For each RadioButton group in m_filCalRBs,
        {
            for (RadioButton item : m_filCalRBs[i])  // For each RadioButton in this group,
            {
                item.setToggleGroup(m_filCalRBToggleGroup[i]);
            }
        }
        
        for (int i = 0; i < m_toolTransformUnits.length; i++) 
        {
            for (ToggleButton item : m_toolTransformUnits[i]) 
            {
                item.setToggleGroup(m_toolTransformSettingsToggleGroup[i]);
            }
        }
        
        activeTransducersChanged();
    }
    
    /**
     * Set the state of the wizard UI to match that of the provided WNet profile,
     * i.e. copy the settings from the XML file to the working variables.
     */
    private void setupWizard() 
    {
        // Set active transducers.
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
        {
            String xpwr = m_wizardProfile.m_xpwr[transducer];

            if (xpwr.contains("ON")) 
            {
                  m_transducers[transducer].setSelected(true);
            } 
            else 
            {
                  m_transducers[transducer].setSelected(false);
            }
        }

        // Apply general settings from the XML file to the working copies.
        m_cmbForce .getSelectionModel().select(m_wizardProfile.getForceUnits ());
        m_cmbTorque.getSelectionModel().select(m_wizardProfile.getTorqueUnits());
        
        m_txtRate    .setText                 (m_wizardProfile.m_rate);
        m_txtOversamp.setText                 (m_wizardProfile.m_oversampling);
        m_chkSD      .setSelected             (m_wizardProfile.m_sd.equals("ON"));
        m_Notes      .setText                 (m_wizardProfile.m_Notes);
        
        if (m_wizardProfile.m_Wnet == 6)
        {
            m_wnet3Toggle.setSelected(false);
            m_wnet6Toggle.setSelected(true);
        }
        else
        {
            m_wnet3Toggle.setSelected(true);
            m_wnet6Toggle.setSelected(false);
        }
        
        m_chkUseNtp.setSelected (m_wizardProfile.m_ntpUse);
        m_chkUseDst.setSelected (m_wizardProfile.m_ntpDst);
        m_txtNtpServer  .setText(m_wizardProfile.m_ntpServer);
        m_txtZoneHours  .setText(m_wizardProfile.m_ntpOffsetHours   + "");
        m_txtZoneMinutes.setText(m_wizardProfile.m_ntpOffsetMinutes + "");
 
        activeTransducersChanged(); // Apply per-transducer settings.
        ShowCorrectTransducers();
    }
    
    // ---------- WNet Model ---------- \\
    
    /**
     * Changes which transducers can be selected
     * based on the type of WNet model being used.
     */
    @FXML
    protected void ShowCorrectTransducers() 
    {
        if (m_wnet3Toggle.isSelected())
        {
            m_t4Toggle.setSelected(false);
            m_t5Toggle.setSelected(false);
            m_t6Toggle.setSelected(false);
            
            m_t4Toggle.setVisible (false);
            m_t5Toggle.setVisible (false);
            m_t6Toggle.setVisible (false);
        } 
        else 
        {
            m_t4Toggle.setVisible(true);
            m_t5Toggle.setVisible(true);
            m_t6Toggle.setVisible(true);
        }
        
        activeTransducersChanged();
    }
    
    // ---------- General Settings ---------- \\
    
    @FXML
    protected void notesChanged() // Executes whenver a key is released within the Notes Box.
    {
        m_wizardProfile.m_Notes = m_Notes.getText(); // Get text box contents
    }
    
    /**
     * Read the selected transducer number from filter/calibration transducer combo box.
     */
    private int GetFilCalTransducer()
    {
        SingleSelectionModel ssm = m_cmbFilCalTrans.getSelectionModel(); // Get Filter/Calibration combo box

        if (ssm.isEmpty())                        // If no selection has been made,
        {
            return -1;
        }
    
        Object selection = ssm.getSelectedItem(); // Get the selected item

        if (selection == null)                    // If there is NO combo box selection,
        {
            return -1;
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
     * Modifies the list from which transducer
     * combo-boxes get their available transducers.
     * Called when the transducer toggles on the
     * main settings page are modified.
     */
    @FXML
    protected void activeTransducersChanged() 
    {
        m_FilCalTrans.clear();
        m_ToolTrTrans.clear();
        
        for (int transducer = 0; transducer < m_transducers.length; transducer++) 
        {
            if (m_transducers[transducer].isSelected()) 
            {
                String label = m_transducers[transducer].getText();
                m_FilCalTrans.add(label);
                m_ToolTrTrans.add(label);
                m_wizardProfile.m_xpwr[transducer] = "ON";
            } 
            else 
            {
                m_wizardProfile.m_xpwr[transducer] = "OFF";
            }
        }
        
        m_FilCalTrans.add("ALL Transducers");
        m_cmbFilCalTrans.setItems(m_FilCalTrans);
        m_cmbTTTrans    .setItems(m_ToolTrTrans);
        
        if (m_FilCalTrans.size() > 0) 
        {
            m_cmbFilCalTrans.getSelectionModel().select(0);
            populateFilterValue();
            populateCalibration();
        }
        
        if (m_ToolTrTrans.size() > 0)
        {
            m_cmbTTTrans.getSelectionModel().select(0);
            populateToolTransform(m_ToolTransformTransducer - 1);
        }
    }
    
    /**
     * Sets the display force units for
     * the WNet profile. Called when the
     * combo-box for force units changes.
     */
    @FXML
    protected void m_cmbForce_Changed() 
    {
        Object selection = m_cmbForce.getSelectionModel().getSelectedItem();
        if (selection != null) 
        {
            m_wizardProfile.setForceUnits(selection.toString());
        }
    }
    
    /**
     * Sets the display torque units for
     * the WNet profile. Called when the
     * combo-box for torque units changes.
     */
    @FXML
    protected void m_cmbTorque_Changed() 
    {
        Object selection = m_cmbTorque.getSelectionModel().getSelectedItem();
        if (selection != null) 
        {
            m_wizardProfile.setTorqueUnits(selection.toString());
        }
    }
    
    /**
     * Reflects text field changes on the slider bar.
     */
    @FXML
    protected void m_txtRate_Changed() 
    {
        int packetRate;

        try 
        {
            packetRate = (int) Double.parseDouble(m_txtRate.getText()); // Get rate from screen
            packetRate = Math.max(packetRate, MIN_UDP_RATE);
            packetRate = Math.min(packetRate, MAX_UDP_RATE);
        } 
        catch (NumberFormatException n) 
        {
            packetRate = MIN_UDP_RATE;
        }
        
        m_txtRate.setText("" +  packetRate); // Copy rate back to screen
        m_wizardProfile.setRate(packetRate); // Copy rate to profile
    }
    
    /**
     * Reflects changes in the oversample rate text.
     */
    @FXML
    protected void m_txtOversamp_Changed()
    {
        try 
        {
            int oversampling;

            oversampling = Integer.parseInt(m_txtOversamp.getText());
            oversampling = Math.max(oversampling, 1);
            oversampling = Math.min(oversampling, MAX_OVERSAMPLING);
            
            m_txtOversamp  .setText("" + oversampling);
            m_wizardProfile.setOversampling(oversampling);
        } 
        catch (NumberFormatException nfe) 
        {   // Do nothing.
        }
    }
    
    /**
     * Reflects changes in the MicroSD check box.
     */
    @FXML
    protected void m_chkSD_Changed() 
    {
        m_wizardProfile.setSD(m_chkSD.isSelected());
    }
    
    // ---------- Filter/Calibration Settings ---------- \\
    
    /**
     * Convenience method to enable/disable the filter controls.
     * 
     * @param on Should the controls be on?
     */
    private void enableFilterControls(boolean on) 
    {
        for (RadioButton rb : m_filCalRBs[0]) 
        {
            rb.setDisable(!on);
        }
        m_txtAveraging.setDisable(!on);
    }
    
    /**
     * Handles changes in the filter/calibration transducer combo box.
     */
    @FXML
    protected void m_cmbFilCalTrans_Changed()    // Filter/Calibration combo box changed
    {
        int transducer  = GetFilCalTransducer(); // Get transducer from Filter/Calibration combo box
        
        if (transducer >= 0)                     // If any or all transducers are selected,
        {
            enableFilterControls(true);
            enableCalControls   (true);
        
            populateFilterValue   ();
            populateCalibration   ();
            UpdateFilterTypeScreen();
        }
        else                                    // If no transducer is selected,
        {
            enableFilterControls(false);
            enableCalControls   (false);
        }
    }
    
    /**
     * Populates the filter values based on the selected transducer.
     */
    private void populateFilterValue() 
    {
        int transducer  = GetFilCalTransducer();
        
        if (transducer >= 0)  // If any or all transducers are selected,
        {
            if (transducer == WirelessFTDemoModel.MAX_SENSORS) 
            {
                transducer =  0;
            }
            
            for (RadioButton rb : m_filCalRBs[0]) 
            {
                if (m_wizardProfile.getFilterType(transducer).equals(rb.getText())) 
                {
                    rb.setSelected(true);
                } 
                else 
                {
                    rb.setSelected(false);
                }
            }

            m_txtAveraging.setText(m_wizardProfile.getFilterValue(transducer));
        }
    }
    
    /**
     * Called when a filter Filtering type RadioButton is pressed.
     */
    @FXML
    protected void filterTypeChanged() 
    {
        UpdateFilterTypeScreen();
    }
    
    private void UpdateFilterTypeScreen()
    {
        int transducer = GetFilCalTransducer();
        
        if (transducer >= 0)                                              // If any or all transducers are selected,
        {
            ToggleGroup group  = m_filCalRBToggleGroup[0];                // Get Filter Type ToggleGroup
            RadioButton button = (RadioButton) group.getSelectedToggle(); // Get RadioButton
            String      label  = button.getText();                        // Get RadioButton label
            
            if (transducer == WirelessFTDemoModel.MAX_SENSORS)            // If ALL Transducers are requested, set All selected transducers to the requested filter type.
            {
                for (transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                {
                    if (m_transducers[transducer].isSelected())
                    {
                        m_wizardProfile.setFilterType(transducer, label);
                    }
                }
            }
            else                 // If only one Transducer is requested, set this transducer to the requested filter type.
            {
                        m_wizardProfile.setFilterType(transducer, label);
            }
            
            switch (label)
            {
                case "No Filtering":
                    m_lblAveraging.setVisible(false);
                    m_txtAveraging.setVisible(false);
                    break;
                
                case "IIR":
                    m_lblAveraging.setText("Time Constant:");
                    m_txtAveraging.setPromptText("1 - 32767");
                    m_lblAveraging.setVisible(true);
                    m_txtAveraging.setVisible(true);
                    break;
                
                case "Running Mean":
                case "Running Median":
                default:
                    m_lblAveraging.setText("Number of samples:");
                    m_txtAveraging.setPromptText("1 - 32");
                    m_lblAveraging.setVisible(true);
                    m_txtAveraging.setVisible(true);
                    break;
            }
        }
    }
    
    /**
     * Called when the number of samples/time constant
     * text field value is changed.
     */
    @FXML
    protected void m_txtAveraging_Changed() 
    {
        int transducer = GetFilCalTransducer();             // Get transducer number from the Filter Calibration combo box.
        
        if (transducer >= 0)                                    // If any or all transducers are selected,
        {
            int min;
            int max;
            int value;
        
            try 
            {
                if (m_rbIIR.isSelected())                       // If the IIR radio button is selected,
                {
                    min = 1;                                    // IIR time constant range: 1-32767
                    max = MAX_TIME_CONSTANT;
                } 
                else                                            // else
                {
                    min = 1;                                    // Mean/Median taps range: 1-32
                    max = MAX_TAPS;
                } 
                
                value = Integer.parseInt(m_txtAveraging.getText()); // Get time constant or taps field.
                value = Math.max(value, min);                   // Clip time constant or taps to the valid range.
                value = Math.min(value, max);
                m_txtAveraging.setText("" + value);             // Store time constant or taps field.
                
                if (transducer == WirelessFTDemoModel.MAX_SENSORS) // If ALL transducers are requested, set all selected transducers to the requested value.
                {
                    for (transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                    {
                        if (m_transducers[transducer].isSelected())
                        {
                            m_wizardProfile.setFilterValue(transducer, value); 
                        }
                    }
                }
                else                                            // If only one transducer is requested, set this transducer to the requested value.
                {
                            m_wizardProfile.setFilterValue(transducer, value);
                }
            }
            catch (NumberFormatException nfe)                   // If the number format is bad,
            {                                                   // change nothing.
            }
        }
    }
    
    /**
     * Convenience method to enable/disable
     * the calibration controls.
     * 
     * @param on Should the controls be on?
     */
    private void enableCalControls(boolean on) 
    {
        for (RadioButton rb : m_filCalRBs[1]) 
        {
            rb.setDisable(!on);
        }
    }
    
    /**
     * Populates the calibration radio buttons
     * based on the selected transducer.
     */
    private void populateCalibration() 
    {
        int transducer  = GetFilCalTransducer();
        
        if (transducer >= 0)                         // If any or all transducers are selected,
        {
            if (transducer == WirelessFTDemoModel.MAX_SENSORS) 
            {
                transducer =  0;
            }

            try 
            {
                int calIndex = Integer.parseInt(m_wizardProfile.getActiveCalibration(transducer));
                
                for (RadioButton rb : m_filCalRBs[1]) // For all Calibration RadioButtons
                {
                    String label = rb.getText();
                    int    rbCal = label.contains("Default") ? 0 : Integer.parseInt(label);
                    rb.setSelected(calIndex == rbCal);
                }

                m_txtAveraging.setText(m_wizardProfile.getFilterValue(transducer));
            } 
            catch (NumberFormatException nfe)        // If no valid calibration found,
            {
                m_filCalRBs[0][0].setSelected(true); // pick calibration Default.
            }
        }
    }
    
    /**
     * Called when a calibration
     * RadioButton is pressed.
     */
    @FXML
    protected void calibrationIndexChanged() 
    {
        int transducer = GetFilCalTransducer(); // Get current Transducer from the Filter/Calibration ComboBox
        
        if (transducer >= 0)                        // If any or all transducers are selected,
        {
            ToggleGroup group       = m_filCalRBToggleGroup[1];                // Get Calibration ToggleGroup
            RadioButton button      = (RadioButton) group.getSelectedToggle(); // Get RadioButton
            String      label       = button.getText();                        // Get RadioButton label
            int         calibration = label.contains("Default") ? 0 : Integer.parseInt(label);

            if (transducer == WirelessFTDemoModel.MAX_SENSORS) // If ALL Transducers are requested, set all selected Transducers to the requested calibration.
            {
                for (transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)
                {
                    if (m_transducers[transducer].isSelected())
                    {
                        m_wizardProfile.setCal(transducer, calibration);
                    }
                }
            }
            else                 // If only one Transducer is requested, set this transducer to the requested calibration.
            {
                        m_wizardProfile.setCal(transducer, calibration);
            }
        }
    }
    
    // ---------- Tool Transform Settings ---------- \\
    
    /**
     * Handles changes in the tool transform
     * transducer combo box.
     */
    @FXML
    protected void m_cmbTTTrans_Changed() 
    {
        SingleSelectionModel ssm = m_cmbTTTrans.getSelectionModel(); // Get Tool Transform Combo Box
        
        if (!ssm.isEmpty()) 
        {
            enableTTControls(true);
            Object selection = ssm.getSelectedItem(); // Get the index from the seleted string and use that as our index.

            if (selection != null) 
            {
                String trans              = selection.toString();
                m_ToolTransformTransducer = Integer.parseInt(trans.charAt(trans.length() - 1) + "");
                populateToolTransform(m_ToolTransformTransducer - 1);
            }
        } 
        else 
        {
            enableTTControls(false);
        }
    }
    
    /**
     * Convenience method to enable/disable
     * the transformation controls.
     * 
     * @param on Should the controls be on?
     */
    private void enableTTControls(boolean on) 
    {
        for (ToggleButton[] tba : m_toolTransformUnits) 
        {
            for (ToggleButton tb : tba) 
            {
                tb.setDisable(!on);
            }
        }
        
        for (TextField[] tfa : m_toolTransformAxisNumbers) 
        {
            for (TextField tf : tfa) 
            {
                tf.setDisable(!on);
            }
        }
        
        m_btnReset.setDisable(!on);
    }
    
    /**
     * Populates the Tool Transform text fields with previous rotation
     * info for the selected transducer, if there is any data stored.
     * @param transducer The currently selected transducer 0 to 5
     */
    private void populateToolTransform(int transducer) 
    {
        if (transducer >= 0) // If any or all transducers are selected,
        {
            final int xyz = 3; // The 3 axes.

            // Update displacement units.
            for (ToggleButton item : m_toolTransformUnits[0]) 
            {
                if (m_wizardProfile.getDisplacementUnits(transducer).equals(item.getText())) 
                {
                    item.setSelected(true);
                } 
                else 
                {
                    item.setSelected(false);
                }
            }

            // Update displacement values.
            String[] displacements = m_wizardProfile.getDisplacementValues(transducer).split("|");
            
            for (int i = 0; i < xyz; i++)  
            {
                try 
                {
                    double disp = Double.parseDouble(displacements[i]);
                    m_toolTransformAxisNumbers[0][i].setText("" + disp);
                } 
                catch(NumberFormatException nfe) // The profile's value was not a valid number.
                {
                    m_toolTransformAxisNumbers[0][i].setText("0.0");
                }
            }

            // Update rotation units.
            for (ToggleButton item : m_toolTransformUnits[1]) 
            {
                if (m_wizardProfile.getRotationUnits(transducer).equals(item.getText())) 
                {
                    item.setSelected(true);
                } 
                else 
                {
                    item.setSelected(false);
                }
            }

            // Update rotation values.
            String[] rotations = m_wizardProfile.getDisplacementValues(transducer).split("|");
            
            for (int i = 0; i < xyz; i++)  
            {
                try 
                {
                    double rot = Double.parseDouble(rotations[i]);
                    m_toolTransformAxisNumbers[1][i].setText("" + rot);
                } 
                catch(NumberFormatException nfe)  // The profile's value was not a valid number.
                {
                    m_toolTransformAxisNumbers[1][i].setText("0.0");
                }
            }
        }
    }
    
    /**
     * Stores any changed tool transform data
     * to the temp profile. Called whenever a
     * Tool Transform control changes.
     */
    @FXML
    protected void updateToolTransform() 
    {
        if (m_ToolTransformTransducer > 0) 
        {
            // Check displacement and rotation units.
            ToggleButton displacementUnitSelected = (ToggleButton) m_toolTransformSettingsToggleGroup[0].getSelectedToggle();
            ToggleButton rotationUnitSelected     = (ToggleButton) m_toolTransformSettingsToggleGroup[1].getSelectedToggle();

            if (displacementUnitSelected != null) 
            {
                m_wizardProfile.setDisplacementUnits(m_ToolTransformTransducer - 1, displacementUnitSelected.getText());
            } 
            else 
            {
                m_wizardProfile.setDisplacementUnits(m_ToolTransformTransducer - 1, WirelessFTDemoProfile.DEFAULT_DISPLACEMENT_UNITS);
            }

            if (rotationUnitSelected != null) 
            {
                m_wizardProfile.setRotationUnits(m_ToolTransformTransducer - 1, rotationUnitSelected.getText());
            } 
            else 
            {
                m_wizardProfile.setRotationUnits(m_ToolTransformTransducer - 1, WirelessFTDemoProfile.DEFAULT_ROTATION_UNITS);
            }

            // Check displacement and rotation values.
            for (int i = 0; i < m_toolTransformAxisNumbers.length; i++) 
            {
                for (int j = 0; j < m_toolTransformAxisNumbers[0].length; j++) 
                {
                    double delta;
                    
                    try 
                    {
                        delta = Double.parseDouble(m_toolTransformAxisNumbers[i][j].getText());
                    } 
                    catch(NumberFormatException nfe) 
                    {
                        delta = 0.0;
                    }

                    m_toolTransformAxisNumbers[i][j].setText("" + delta);
                    m_wizardProfile.m_xforms[m_ToolTransformTransducer - 1][i][j] = "" + delta;
                }
            }
        }
    }
    
    /**
     * Clears the transformation from the current transducer.
     */
    @FXML
    protected void m_btnResetPressed() 
    {
        if (m_ToolTransformTransducer > 0) 
        {
            for (int i = 0; i < m_toolTransformAxisNumbers.length; i++) 
            {
                for (int j = 0; j < m_toolTransformAxisNumbers[i].length; j++) 
                {
                    m_toolTransformAxisNumbers[i][j].setText("0.0");
                    m_wizardProfile.m_xforms[m_ToolTransformTransducer - 1][i][j] = "0.0";
                }
            }
        }
    }
    
    // ---------- Navigation Controls ---------- \\
    
    /**
     * Closes the form.
     */
    @FXML
    protected void m_btnCancelPressed() 
    {
        close();
    }
    
    /**
     * Navigates back one page, if possible.
     */
    @FXML
    protected void m_btnBackPressed()               // If the Back button is pressed,
    {
        FinalizeTextFields(m_pageIndex);
        UpdateFilterTypeScreen();
        ShowCorrectTransducers();
        
        if (m_pageIndex > FIRST_PAGE)               // If we are not at the first page,
        {
            m_pages[m_pageIndex].setVisible(false); // make the current page invisible,
            m_pageIndex--;                          // decrement the page number,
            if (m_pageIndex == TOOL_PAGE) { m_pageIndex--; } // Temporarily skip the Tool Transform page.
            m_pages[m_pageIndex].setVisible(true);  // and make the new page visibile.
        }
       
        if (m_pageIndex == FIRST_PAGE)              // If we are now at the first page,
        {
            m_btnBack.setDisable(true);             // disable the Back button.
        } 
        else                                        // If we are not at the first page,
        {
            m_btnBack.setDisable(false);            // enable the Back button, and
            m_btnNext.setDisable(false);            // enable the Next button.
        }
    }
    
    /**
     * Navigates forward one page, if possible.
     */
    @FXML
    protected void m_btnNextPressed()               // If the Next button is pressed,
    {
        FinalizeTextFields(m_pageIndex);
        UpdateFilterTypeScreen();
        ShowCorrectTransducers();
        
        if (m_pageIndex < MAX_PAGE)                 // If we are not at the last page,
        {
            m_pages[m_pageIndex].setVisible(false); // make the current page invisible,
            m_pageIndex++;                          // increment the page number,
            if (m_pageIndex == TOOL_PAGE) { m_pageIndex++; } // Temporarily skip the Tool Transform page.
            m_pages[m_pageIndex].setVisible(true);  // and make the new page visible.
        }
       
        if (m_pageIndex == MAX_PAGE)                // If we are now at the last page,
        {
            m_btnNext.setDisable(true);             // disable the Next button.
        } 
        else                                        // If we are not at the last page,
        {
            m_btnBack.setDisable(false);            // enable the Back button, and
            m_btnNext.setDisable(false);            // enable the next button.
        }
    }
    
    /**
     * Saves changes to an XML file,
     * and closes the window.
     */
    @FXML
    protected void m_btnFinishPressed() 
    {
        FinalizeTextFields(m_pageIndex);
        WriteXmlFile();
        close();
    }
    
    /**
     * Finalize all text fields.
     */
    private void FinalizeTextFields(int page)
    {
        switch (page)
        {
            case MODEL_PAGE:
                m_txtRate_Changed();
                m_txtOversamp_Changed();
                break;
                
            case NTP_PAGE:
                UpdateNtpServer();
                UpdateZoneHours();
                UpdateZoneMinutes();
                break;
                
            case FCU_PAGE:
                m_txtAveraging_Changed();
                break;
                
            case TOOL_PAGE:
                updateToolTransform();
                break;
                
            default:
                break;
        }
    }
    
    /**
     * Saves changes to an XML file.
     * Creates a WNet profile with the provided settings.
     */
    private void WriteXmlFile()
    {
        try 
        {
            // Build an XML document for the settings profile.
            DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder        docBuilder = docFactory.newDocumentBuilder();
            
            // Root element
            Document doc     = docBuilder.newDocument();
            Element  profile = doc.createElement("profile");
            doc.appendChild(profile);
            
            { // General settings
                Element general = doc.createElement("general");
                profile.appendChild(general);
                
                { // Transmit rate
                    Element rate = doc.createElement("rate");
                    rate.appendChild(doc.createTextNode(m_wizardProfile.m_rate));
                    general.appendChild(rate);
                }
                { // Oversampling rate
                    Element oversamp = doc.createElement("oversamp");
                    oversamp.appendChild(doc.createTextNode(m_wizardProfile.m_oversampling));
                    general.appendChild(oversamp);
                }
                { // MicroSD Recording
                    Element sd = doc.createElement("sd");
                    sd.appendChild(doc.createTextNode(m_wizardProfile.m_sd));
                    general.appendChild(sd);
                }
                { // Force units
                    Element force = doc.createElement("force");
                    force.appendChild(doc.createTextNode(m_wizardProfile.m_force));
                    general.appendChild(force);
                }
                { // Torque units
                    Element torque = doc.createElement("torque");
                    torque.appendChild(doc.createTextNode(m_wizardProfile.m_torque));
                    general.appendChild(torque);
                }
                { // Active transducers
                    Element transducers = doc.createElement("transducers");
                    general.appendChild(transducers);
                    
                    for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
                    {
                        Element t = doc.createElement("t" + (transducer + 1));
                        t.appendChild(doc.createTextNode(m_wizardProfile.m_xpwr[transducer]));
                        transducers.appendChild(t);
                    }
                }
                { // Write Notes field to the XML file.
                    Element notes = doc.createElement("notes");
                    notes.appendChild(doc.createTextNode(m_wizardProfile.m_Notes));
                    general.appendChild(notes);
                }
                { // Write wnet field to the XML file.
                    Element wnet = doc.createElement("wnet");
                    wnet.appendChild(doc.createTextNode(m_wnet3Toggle.isSelected() ? "3" : "6"));
                    general.appendChild(wnet);
                }        
                
                { // Write NTP on/off field to the XML file.
                    Element ntpUse = doc.createElement("NtpOnOff");
                    ntpUse.appendChild(doc.createTextNode(m_wizardProfile.m_ntpUse ? "ON" : "OFF"));
                    general.appendChild(ntpUse);
                }        
                { // Write NTP server field to the XML file.
                    Element ntpServer = doc.createElement("NtpServer");
                    ntpServer.appendChild(doc.createTextNode(m_wizardProfile.m_ntpServer));
                    general.appendChild(ntpServer);
                }        
                { // Write NTP hours field to the XML file.
                    Element hours = doc.createElement("NtpOffsetHours");
                    hours.appendChild(doc.createTextNode("" + m_wizardProfile.m_ntpOffsetHours));
                    general.appendChild(hours);
                }        
                { // Write NTP minutes field to the XML file.
                    Element minutes = doc.createElement("NtpOffsetMinutes");
                    minutes.appendChild(doc.createTextNode("" + m_wizardProfile.m_ntpOffsetMinutes));
                    general.appendChild(minutes);
                }        
                { // Write NTP DST field to the XML file.
                    Element dst = doc.createElement("NtpDstOnOff");
                    dst.appendChild(doc.createTextNode(m_wizardProfile.m_ntpDst ? "ON" : "OFF"));
                    general.appendChild(dst);
                }        
            }
            
            { // Filters (averaging) for each transducer
                Element filters = doc.createElement("filters");
                profile.appendChild(filters);

                for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
                {
                    Element t = doc.createElement("t" + (transducer + 1));
                    { // Filtering type
                        Element type = doc.createElement("type");
                        type.appendChild(doc.createTextNode(m_wizardProfile.m_filters[transducer][0]));
                        t.appendChild(type);
                    }
                    { // Averaging value (samples or time constant)
                        Element val = doc.createElement("val");
                        val.appendChild(doc.createTextNode(m_wizardProfile.m_filters[transducer][1]));
                        t.appendChild(val);
                    }
                    filters.appendChild(t);
                }
            }
            
            { // Calibration index for each transducer
                Element calibration = doc.createElement("calibration");
                profile.appendChild(calibration);

                for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
                {
                    Element t = doc.createElement("t" + (transducer + 1));
                    t.appendChild(doc.createTextNode(m_wizardProfile.m_cals[transducer]));
                    calibration.appendChild(t);
                }
            }
            
            { // Transformation for each transducer
                Element transformation = doc.createElement("transformation");
                profile.appendChild(transformation);

                for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++) 
                {
                    Element t = doc.createElement("t" + (transducer + 1));
                    { // Displacement
                        Element displacement = doc.createElement("displacement");
                        { // Units
                            Element units = doc.createElement("units");
                            units.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][0]));
                            displacement.appendChild(units);
                        }
                        { // Values
                            Element xvalue = doc.createElement("xvalue");
                            xvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][1]));
                            displacement.appendChild(xvalue);
                            
                            Element yvalue = doc.createElement("yvalue");
                            yvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][2]));
                            displacement.appendChild(yvalue);
                            
                            Element zvalue = doc.createElement("zvalue");
                            zvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][3]));
                            displacement.appendChild(zvalue);
                        }
                        t.appendChild(displacement);
                    }
                    { // Rotation
                        Element rotation = doc.createElement("rotation");
                        { // Units
                            Element units = doc.createElement("units");
                            units.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][1][0]));
                            rotation.appendChild(units);
                        }
                        { // Values
                            Element xvalue = doc.createElement("xvalue");
                            xvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][1]));
                            rotation.appendChild(xvalue);
                            
                            Element yvalue = doc.createElement("yvalue");
                            yvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][2]));
                            rotation.appendChild(yvalue);
                            
                            Element zvalue = doc.createElement("zvalue");
                            zvalue.appendChild(doc.createTextNode(m_wizardProfile.m_xforms[transducer][0][3]));
                            rotation.appendChild(zvalue);
                        }
                        t.appendChild(rotation);
                    }
                    transformation.appendChild(t);
                }
                profile.appendChild(transformation);
            }

            // Write XML file.
            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer        transformer        = transformerFactory.newTransformer();
            DOMSource          source             = new DOMSource(doc);
            StreamResult       result             = new StreamResult(m_profileXML);

            transformer.transform(source, result);
            
            if (m_chkDefault.isSelected()) 
            {
                m_parent.setDefaultFlag();
            }
        } 
        catch (ParserConfigurationException | DOMException | TransformerException e) 
        {
            e.printStackTrace();
        }
    }
    
    /**
     * Closes the window.
     */
    private void close() // Executes when Cancel button on WNet Profile Wizard is pressed.
    {
        /**
         * Nothing special about the Cancel button;
         * we could get the active scene/window through any control.
         */
        Stage stage = (Stage) m_btnCancel.getScene().getWindow();
        stage.close();
    }
    
    public void UpdateUseNtp()
    {
        m_wizardProfile.m_ntpUse = m_chkUseNtp.isSelected();
    }
    
    public void UpdateNtpServer()
    {
        m_wizardProfile.m_ntpServer = m_txtNtpServer.getText();
    }
    
    public void UpdateUseDst()
    {
        m_wizardProfile.m_ntpDst = m_chkUseDst.isSelected();
    }
    
    public void UpdateZoneHours()
    {
        int hours;
        
        try 
        {
            hours = Integer.parseInt(m_txtZoneHours.getText());
            hours = Math.max(hours, -12); // UTC time offsets range from UTC-12:00 in the west
            hours = Math.min(hours,  14); // to UTC+14:00 in the east.
            m_txtZoneHours.setText("" + hours);
            m_wizardProfile.m_ntpOffsetHours = hours;
        }
        catch(NumberFormatException nfe) 
        { // Do nothing.
        }
    }
    
    public void UpdateZoneMinutes()
    {
        int minutes;
        
        try 
        {
            minutes = Integer.parseInt(m_txtZoneMinutes.getText());
            minutes = Math.max(minutes,  0); // UTC offset minutes range from 0
            minutes = Math.min(minutes, 59); // to 59.
            m_txtZoneMinutes.setText("" + minutes);
            m_wizardProfile.m_ntpOffsetMinutes = minutes;
        } 
        catch(NumberFormatException nfe) 
        { // Do nothing.
        }
    }
}