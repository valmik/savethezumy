/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.util.*;
import RTADiscoveryProtocol.DiscoveryClient;
import RTADiscoveryProtocol.RTADevice;
import java.io.IOException;
import java.net.URL;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ListView;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseEvent;
import javafx.stage.Stage;
import java.util.ResourceBundle;
import java.util.logging.Level;
import java.util.logging.Logger;
import javafx.animation.Animation;
import javafx.animation.KeyFrame;
import javafx.animation.Timeline;
import javafx.event.EventHandler;
import javafx.fxml.Initializable;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.util.Duration;

/**
 * FXML Controller class
 *
 * @author Chris Collins
 */
public class DiscoveryResultsController implements Initializable 
{
    @FXML private Button           m_btnCancel;
    @FXML private ListView<String> m_listIP;
    @FXML         TextField        m_txtFieldSensorHostName;
    
    private static final Logger m_logger     = Logger.getLogger("discoveryresults");
    RTADevice[]                 m_devices;
    ArrayList<String>           m_wftStrings = new ArrayList<>();
    ArrayList<RTADevice>        m_wft        = new ArrayList<>();

    /** 
     * Select the Wireless F/T device and put its 
     * corresponding IP address in the text field
     * on the main screen.
     * 
     * @param e Not used.
     */
    @FXML
    protected void okButtonClicked(ActionEvent e) 
    {
        String device = m_listIP.getSelectionModel().getSelectedItem();
        
        if (device != null) 
        {
            for (RTADevice m_wft1 : m_wft) 
            {
                if (device.equals(m_wft1.toString())) 
                {
                    // Move IP to the TextField on the main screen.
                    m_txtFieldSensorHostName.setText(m_wft1.m_ipa.getHostAddress());
                    break;
                }
            }
        }
        
        cancelButtonClicked(null);
    }
    
    /**
     * Double-clicks are equivocal to hitting the "OK" button.
     * @param m The mouse button clicked.
     */
    @FXML
    protected void doubleClicked(MouseEvent m) 
    {
        if (m.getClickCount() > 1)
        {
            okButtonClicked(null);
        }
    }
    
    /**
     * Closes the window.
     * @param e Not used.
     */
    @FXML
    protected void cancelButtonClicked(ActionEvent e) 
    {
        Stage d = (Stage) m_btnCancel.getScene().getWindow();
        d.close();
    }
    
    /**
     * Opens the window and populates it with WNets
     * discovered successfully with the @RTADiscoveryProtocol.
     * @param url Not used.
     * @param rb  Not used.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) // Called once when DiscoveryResults.fxml is loaded.
    {
        m_listIP.getSelectionModel().setSelectionMode(SelectionMode.SINGLE);
    }
    
    private class DiscoveryThread implements Runnable
    {
        private Stage m_window = null;
        
        public DiscoveryThread(Stage window)
        {
            m_window = window;
        }
        
        @Override
        public void run()
        {
            while (m_window.isShowing()) // While the Discovery Results window is open,
            {
                try 
                {
                    m_devices = DiscoveryClient.discoverRTADevicesLocalBroadcast(); // Try to Discover devices on the network via broadcast.
                } 
                catch (IOException ex) 
                {
                    m_logger.log(Level.WARNING, "Could not query devices: " + ex.getMessage());
                }
            }
        }
    }
    
    public void DiscoverButtonPressedCommon(AnchorPane page)
    {
        Scene scene;
        scene = new Scene(page);

        final Stage stage;
        stage = new Stage();
        stage.setScene(scene);
        stage.setTitle("Discovery Results");
        stage.setResizable(false);
        stage.show();

        new Thread(new DiscoveryThread(stage)).start(); // Start the continuous Discovery Ping task.

        final Timeline discoveryAnimation;
        discoveryAnimation = new Timeline();
        discoveryAnimation.getKeyFrames().clear();
        discoveryAnimation.getKeyFrames().add(new KeyFrame(Duration.millis(100), new EventHandler<ActionEvent>() 
        {
            @Override
            public void handle(ActionEvent actionEvent) 
            {
                if (stage.isShowing())                                 // If the Discovery Results window is open,
                {
                    RTADevice[] devices = m_devices;                   // get the results from the last Discovery message.
                    
                    if (devices != null)                               // If we found any devices,
                    {
                        for (RTADevice device : devices)               // For each device found,
                        {
                            String devString = device.toString();      // Get device string

                            if (    devString.contains("Wireless F/T") // If this is a Wireless Net F/T device,
                                && !m_wftStrings.contains(devString))  // and we have not seen this device before,
                            {
                                m_wft.add(device);                     // add it to the list.
                                m_wftStrings.add(devString);
                                m_listIP.getItems().add(devString);
                            }
                        }
                    }
                } 
                else                           // If the Discovery Results window is closed,
                {
                    discoveryAnimation.stop(); // we are done.
                }
            }
        }));
        discoveryAnimation.setCycleCount(Animation.INDEFINITE);
        discoveryAnimation.play();
    }
    
    /**
     * Grants access to the text field on the main
     * screen so results selected can be sent to it.
     * 
     * @param t IP/Hostname text field from the main screen
     */
    public void retrieveHostTextBox(TextField t) 
    {
        m_txtFieldSensorHostName = t;
    }
}
   
