/*
 * Copyright 2014
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.fxml.Initializable;
import wirelessftjavademo.WirelessFTDemoModel;
import wirelessftsensor.WirelessFTSensor;

/**
 *
 * @author jenwi
 */
public class AboutScreenController implements Initializable
{
    @FXML
    private Label m_versionDemo;

    @FXML
    private Label m_versionFirmware;
    
    @FXML
    private Label m_versionWlan;

    @FXML
    private Label m_versionCpld0;

    @FXML
    private Label m_versionCpld1;

    private WirelessFTDemoModel m_model;

    @Override
    public void initialize(URL location, ResourceBundle resources) // Called once when AboutScreenController.fxml is loaded.
    {
    }

    // Runs after this screen is opened.
    public void initializeAboutScreen(WirelessFTDemoModel model, boolean connected) throws IOException, WirelessFTSensor.CommSeveredException 
    {
        m_model = model;
        
        m_versionDemo.setText(StartupScreenController.VERSION);
        
        if (connected)
        {
            String[] componentVersions = m_model.getComponentVersions();
            m_versionFirmware.setText(componentVersions[0]);
            m_versionWlan    .setText(componentVersions[1]);
            m_versionCpld0   .setText(componentVersions[2]);
            m_versionCpld1   .setText(componentVersions[3]);
        }
        else
        {
            m_versionFirmware.setDisable(true);
            m_versionWlan    .setDisable(true);
            m_versionCpld0   .setDisable(true);
            m_versionCpld1   .setDisable(true);
        }
    }
}
