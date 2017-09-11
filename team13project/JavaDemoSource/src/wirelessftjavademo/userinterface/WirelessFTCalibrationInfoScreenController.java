/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.fxml.Initializable;
import javafx.scene.control.Accordion;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.Tab;
import javafx.scene.control.TitledPane;
import javafx.scene.control.Toggle;
import javafx.scene.control.ToggleGroup;
import javafx.stage.Stage;
import wirelessftjavademo.Calibration;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftsensor.WirelessFTSensor.CommSeveredException;

/**
 * FXML Controller class
 *
 * @author Sam Skuce
 */
public class WirelessFTCalibrationInfoScreenController implements Initializable {
    
    @FXML
    Tab m_tabPaneMatrix;
    
    @FXML
    Label m_lblCal1Serial;

    @FXML
    Label m_lblCal1PartNumber;

    @FXML
    Label m_lblCal1Date;

    @FXML
    Label m_lblCal1Force;

    @FXML
    Label m_lblCal1Torque;

    @FXML
    Label m_lblCal2Serial;

    @FXML
    Label m_lblCal2PartNumber;

    @FXML
    Label m_lblCal2Date;

    @FXML
    Label m_lblCal2Force;

    @FXML
    Label m_lblCal2Torque;

    @FXML
    Label m_lblCal3Serial;

    @FXML
    Label m_lblCal3PartNumber;

    @FXML
    Label m_lblCal3Date;

    @FXML
    Label m_lblCal3Force;

    @FXML
    Label m_lblCal3Torque;
    
    @FXML
    Label m_lblfxGain;
    
    @FXML
    Label m_lblfyGain;
    
    @FXML
    Label m_lblfzGain;
    
    @FXML
    Label m_lbltxGain;
    
    @FXML
    Label m_lbltyGain;
    
    @FXML
    Label m_lbltzGain;
    
    @FXML
    Label m_lblfxOffset;
    
    @FXML
    Label m_lblfyOffset;
    
    @FXML
    Label m_lblfzOffset;
    
    @FXML
    Label m_lbltxOffset;
    
    @FXML
    Label m_lbltyOffset;
    
    @FXML
    Label m_lbltzOffset;
    
    @FXML
    Label m_lblfxg0;
    
    @FXML
    Label m_lblfyg0;
    
    @FXML
    Label m_lblfzg0;
    
    @FXML
    Label m_lbltxg0;
    
    @FXML
    Label m_lbltyg0;
    
    @FXML
    Label m_lbltzg0;
    
    @FXML
    Label m_lblfxg1;
    
    @FXML
    Label m_lblfyg1;
    
    @FXML
    Label m_lblfzg1;
    
    @FXML
    Label m_lbltxg1;
    
    @FXML
    Label m_lbltyg1;
    
    @FXML
    Label m_lbltzg1;
    
    @FXML
    Label m_lblfxg2;
    
    @FXML
    Label m_lblfyg2;
    
    @FXML
    Label m_lblfzg2;
    
    @FXML
    Label m_lbltxg2;
    
    @FXML
    Label m_lbltyg2;
    
    @FXML
    Label m_lbltzg2;
    
    @FXML
    Label m_lblfxg3;
    
    @FXML
    Label m_lblfyg3;
    
    @FXML
    Label m_lblfzg3;
    
    @FXML
    Label m_lbltxg3;
    
    @FXML
    Label m_lbltyg3;
    
    @FXML
    Label m_lbltzg3;
    
    @FXML
    Label m_lblfxg4;
    
    @FXML
    Label m_lblfyg4;
    
    @FXML
    Label m_lblfzg4;
    
    @FXML
    Label m_lbltxg4;
    
    @FXML
    Label m_lbltyg4;
    
    @FXML
    Label m_lbltzg4;
    
    @FXML
    Label m_lblfxg5;
    
    @FXML
    Label m_lblfyg5;
    
    @FXML
    Label m_lblfzg5;
    
    @FXML
    Label m_lbltxg5;
    
    @FXML
    Label m_lbltyg5;
    
    @FXML
    Label m_lbltzg5;
    
    @FXML
    Label m_tabLabelTransducer;
    
    @FXML
    Button m_btnOkay;
    
    @FXML
    Button m_btnApplyRefresh;
    
    @FXML
    RadioButton m_radioT1Cal1;
    
    @FXML
    RadioButton m_radioT1Cal2;
    
    @FXML
    RadioButton m_radioT1Cal3;
    
    @FXML
    RadioButton m_radioT2Cal1;
    
    @FXML
    RadioButton m_radioT2Cal2;
    
    @FXML
    RadioButton m_radioT2Cal3;
    
    @FXML
    RadioButton m_radioT3Cal1;
    
    @FXML
    RadioButton m_radioT3Cal2;
    
    @FXML
    RadioButton m_radioT3Cal3;
    
    @FXML
    RadioButton m_radioT4Cal1;
    
    @FXML
    RadioButton m_radioT4Cal2;
    
    @FXML
    RadioButton m_radioT4Cal3;
    
    @FXML
    RadioButton m_radioT5Cal1;
    
    @FXML
    RadioButton m_radioT5Cal2;
    
    @FXML
    RadioButton m_radioT5Cal3;
    
    @FXML
    RadioButton m_radioT6Cal1;
    
    @FXML
    RadioButton m_radioT6Cal2;
    
    @FXML
    RadioButton m_radioT6Cal3;
    
    @FXML
    TitledPane m_AccTitledPane1;
    
    @FXML
    TitledPane m_AccTitledPane2;
    
    @FXML
    TitledPane m_AccTitledPane3;
    
    @FXML
    TitledPane m_AccTitledPane4;
    
    @FXML
    TitledPane m_AccTitledPane5;
    
    @FXML
    TitledPane m_AccTitledPane6;
    
    @FXML
    Accordion m_accordionTransducer;
    
    private WirelessFTDemoModel m_model;
    
    Label[][] calibrationLabels;

    RadioButton[][] transducerRadioButtons;

    Label[][] gainOffsetLabels;
    
    Label[][] matrixLabels;
    
    Calibration[][] calibrations;
    
    private TitledPane[] accordionTitledPanes;
        
    private WirelessFTDemoMainScreenController m_controller;

    private void setCalLabels(int transducer, int calibration)
    {
        Calibration cal = calibrations[transducer][calibration];
        
        calibrationLabels[0][calibration].setText(cal.getSerialNumber   ());
        calibrationLabels[1][calibration].setText(cal.getPartNumber     ());
        calibrationLabels[2][calibration].setText(cal.getCalibrationDate());
        calibrationLabels[3][calibration].setText(cal.getMaxRangeForce  () + " " + cal.getForceUnits ());
        calibrationLabels[4][calibration].setText(cal.getMaxRangeTorque () + " " + cal.getTorqueUnits());
    }
    
    private void DisplayAllTransducerCalibrations(int transducer)
    {
        m_tabLabelTransducer.setText("Transducer " + (transducer + 1)); // Transducer is origin 1 to the user

        for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++) // For all possible Calibrations
        {
            setCalLabels(transducer, calibration); //set text for calibration information labels
        }
    }
    
    private void DisplayCalibrationData(int transducer, int calibration)
    {
        Calibration   cal = calibrations[transducer][calibration];
        setMatrix    (cal);
        setGainOffset(cal);
        m_tabPaneMatrix.setText("F/T Matrix Transducer " + (transducer + 1) + " Calibration " + (calibration + 1)); // Transducer is origin 1 to the user
    }
    
    private void TransducerAccordionPaneOpened()
    {
        String string     = m_accordionTransducer.getExpandedPane().getText(); // Get accordion pane label
        String target     = "Transducer ";                                     // Extract just the number after "Transducer"
        int    index      = string.indexOf(target) + target.length();
        String found      = string.substring(index);
        int    transducer = Integer.parseInt(found) - 1;                       // Transducer is origin 1 to the user
        DisplayAllTransducerCalibrations(transducer);                          // Display all Calibrations for this Transducer
                    
        for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++)
        {
            if (transducerRadioButtons[transducer][calibration].selectedProperty().get()) // If this calibration is selected,
            {
                DisplayCalibrationData(transducer, calibration);                          // display the gain, offset, and matrix data.
            }
        }
    }
    
    // Runs when this screen is opened.
    public void initializeCalibrationController (WirelessFTDemoModel model, WirelessFTDemoMainScreenController controller) throws IOException, CommSeveredException 
    {
        m_model      = model;
        m_controller = controller;
        
        calibrationLabels = new Label[][] 
        {
            {m_lblCal1Serial,     m_lblCal2Serial,     m_lblCal3Serial},
            {m_lblCal1PartNumber, m_lblCal2PartNumber, m_lblCal3PartNumber},
            {m_lblCal1Date,       m_lblCal2Date,       m_lblCal3Date},
            {m_lblCal1Force,      m_lblCal2Force,      m_lblCal3Force},
            {m_lblCal1Torque,     m_lblCal2Torque,     m_lblCal3Torque}
        };

        transducerRadioButtons = new RadioButton[][]
        {
            {m_radioT1Cal1, m_radioT1Cal2, m_radioT1Cal3},
            {m_radioT2Cal1, m_radioT2Cal2, m_radioT2Cal3},
            {m_radioT3Cal1, m_radioT3Cal2, m_radioT3Cal3},
            {m_radioT4Cal1, m_radioT4Cal2, m_radioT4Cal3},
            {m_radioT5Cal1, m_radioT5Cal2, m_radioT5Cal3},
            {m_radioT6Cal1, m_radioT6Cal2, m_radioT6Cal3}
        };

        ToggleGroup[] transducerCalibrationRadioGroups = 
        {
            new ToggleGroup(), new ToggleGroup(), 
            new ToggleGroup(), new ToggleGroup(), 
            new ToggleGroup(), new ToggleGroup()
        };

        matrixLabels = new Label[][]
        {
            {m_lblfxg0, m_lblfyg0, m_lblfzg0, m_lbltxg0, m_lbltyg0, m_lbltzg0},
            {m_lblfxg1, m_lblfyg1, m_lblfzg1, m_lbltxg1, m_lbltyg1, m_lbltzg1},
            {m_lblfxg2, m_lblfyg2, m_lblfzg2, m_lbltxg2, m_lbltyg2, m_lbltzg2},
            {m_lblfxg3, m_lblfyg3, m_lblfzg3, m_lbltxg3, m_lbltyg3, m_lbltzg3},
            {m_lblfxg4, m_lblfyg4, m_lblfzg4, m_lbltxg4, m_lbltyg4, m_lbltzg4},
            {m_lblfxg5, m_lblfyg5, m_lblfzg5, m_lbltxg5, m_lbltyg5, m_lbltzg5}
        };

        gainOffsetLabels = new Label[][]
        {
            {m_lblfxGain, m_lblfxOffset},
            {m_lblfyGain, m_lblfyOffset},
            {m_lblfzGain, m_lblfzOffset},
            {m_lbltxGain, m_lbltxOffset},
            {m_lbltyGain, m_lbltyOffset},
            {m_lbltzGain, m_lbltzOffset}
        };
        
        accordionTitledPanes = new TitledPane[]
        {
            m_AccTitledPane1, m_AccTitledPane2, m_AccTitledPane3, m_AccTitledPane4, m_AccTitledPane5, m_AccTitledPane6
        };
        
        calibrations = new Calibration[WirelessFTDemoModel.MAX_SENSORS][WirelessFTDemoModel.MAX_CALIBRATIONS];
         
        boolean userMode = m_model.SetTechnicianMode();                                              // Set technician mode

        // Find and enable only powered Transducers in the Accordion
        boolean[] poweredTransducers = m_model.getPoweredTransducers();
        
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // Apply Wnet powered Transducers to the Transducer Accordion panes.
        {
            boolean powered = poweredTransducers[transducer];
            accordionTitledPanes[transducer].setDisable(!powered);
        }

        // Find and set selected options for calibration radio buttons in the Accordion
        int[] activeCalibrations = m_model.getActiveCalibrations();                                  // Get the active calibrations from the Wnet.

        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // Apply Wnet active calibrations to the screen radio buttons.
        {
            for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++)
            {
                transducerRadioButtons[transducer][calibration].selectedProperty().set(false);
            }
            
            if (poweredTransducers[transducer])                                                      // If this Transducer is powered,
            {
                int calibration = activeCalibrations[transducer];
                transducerRadioButtons[transducer][calibration].selectedProperty().set(true);
            }
        }
        
        // Get all Calibration data for all powereed Transducers.
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // For each possible Transducer
        {
            if (poweredTransducers[transducer])                                                      // If this Transducer is powered,
            {
                m_model.setActiveSensor(transducer);                                                 // select it in the Wnet.
            
                for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++) // For each possible Calibration
                {
                    m_model.setActiveCalibration(calibration);                                       // select it in the Wnet.
                    calibrations[transducer][calibration] = m_model.readActiveCalibration();         // Send CAL command, get calibration information.
                }
            }
        }

        m_model.RestoreWnetMode(userMode);                                                           // restore Wnet mode
         
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // For each possible Transducer
        {
            for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++) // For each possible Calibration
            {
                ToggleGroup t = transducerCalibrationRadioGroups[transducer];
                transducerRadioButtons[transducer][calibration].setToggleGroup(t);
            }
        }
        
        // Pick the first powered Transducer to have its Accordion pane opened at start up.
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // For each possible Transducer
        {
            if (poweredTransducers[transducer])                                                      // If this Transducer is powered,
            {
                m_accordionTransducer.expandedPaneProperty().set(accordionTitledPanes[transducer]);
                TransducerAccordionPaneOpened();
                break;
            }            
        }

        // Runs when a TitledPane is expanded within the Accordion.
        m_accordionTransducer.expandedPaneProperty().addListener(new ChangeListener<TitledPane>() 
        {
            @Override
            public void changed(ObservableValue<? extends TitledPane> ov, TitledPane old_val, TitledPane new_val)
            {
                if (new_val != null) 
                {
                    TransducerAccordionPaneOpened();
                }
            } 
        });
        
        // Runs when a Radio Button is pressed within the Transducer 1 Accordion TitledPane.
        transducerCalibrationRadioGroups[0].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(0, calibration);
            }
        });
        
        // Runs when a Radio Button is pressed within the Transducer 2 Accordion TitledPane.
        transducerCalibrationRadioGroups[1].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(1, calibration);
            }
        });
        
        // Runs when a Radio Button is pressed within the Transducer 3 Accordion TitledPane.
        transducerCalibrationRadioGroups[2].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(2, calibration);
            }
        });
        
        // Runs when a Radio Button is pressed within the Transducer 4 Accordion TitledPane.
        transducerCalibrationRadioGroups[3].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(3, calibration);
            }
        });
        
        // Runs when a Radio Button is pressed within the Transducer 5 Accordion TitledPane.
        transducerCalibrationRadioGroups[4].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(4, calibration);
            }
        });
        
        // Runs when a Radio Button is pressed within the Transducer 6 Accordion TitledPane.
        transducerCalibrationRadioGroups[5].selectedToggleProperty().addListener(new ChangeListener<Toggle>() 
        {
            @Override
            public void changed(ObservableValue<? extends Toggle> ov, Toggle t, Toggle t1) 
            {
                RadioButton button      = (RadioButton) t1;
                int         calibration = Integer.parseInt(button.getText()) - 1;
                DisplayCalibrationData(5, calibration);
            }
        });
    }
    
    @Override
    public void initialize(URL url, ResourceBundle rb) 
    {
    }
    
    @FXML
    public void applyButtonAction() throws IOException, CommSeveredException 
    {
        //Find which buttons are selected and send commands
        for (int transducer = 0; transducer < WirelessFTDemoModel.MAX_SENSORS; transducer++)         // For all possible Transducers
        {
            for (int calibration = 0; calibration < WirelessFTDemoModel.MAX_CALIBRATIONS; calibration++) // For all possible Calibrations
            {
                if (transducerRadioButtons[transducer][calibration].isSelected())                    // If this combination is selected,
                {
                    m_model.setActiveSensor     (transducer);                                        // select it in the Wnet.
                    m_model.setActiveCalibration(calibration);
                }
            }
        }
        
        m_controller.refreshCalibrationInformation();
    }
    
    @FXML
    public void okayButtonAction() throws Exception 
    {
        applyButtonAction();
        close();
    }
    
    @FXML
    public void cancelButtonAction() throws Exception 
    {
        close();
    }
    
    private void close()
    {
        /* Nothing special about m_btnOkay, we could get the active scene/window
         * through any control.
         */
        ((Stage)m_btnOkay.getScene().getWindow()).close();
    }

    private void setMatrix(Calibration cal) 
    {
        float [][] matrix = cal.getMatrix();
        
        for (int i = 0; i < matrixLabels.length; i++)        // For each row
        {
            for (int j = 0; j < matrixLabels[0].length; j++) // for each column
            {
                matrixLabels[i][j].setText(Float.toString(matrix[i][j])); // put in the matrix value.
            }
        }
    }
    
    private void setGainOffset(Calibration cal)
    {
        int[][] offsetGain = cal.getGainOffset();
        
        for (int gage = 0; gage < gainOffsetLabels.length; gage++) // For each Gage
        {
            for (int label = 0; label < gainOffsetLabels[0].length; label++)         // For each label (gain or offset)
            {
                gainOffsetLabels[gage][label].setText(Integer.toString(offsetGain[gage][label])); // put in the value.
            }
        }
    }
}
