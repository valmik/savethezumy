/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo.userinterface;

import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;
import wirelessftjavademo.WirelessFTJavaDemo;

/**
 * JavaFX has no default dialogs/pop-ups,
 * so I made one from scratch.
 * 
 * @author Chris Collins
 */
public class PopupDialogController implements Initializable 
{
    @FXML
    Label m_lblMessage;
    
    @FXML
    Button m_btnOK;
    
    /**
     * Initializes the controller class.
     * @param url
     * @param rb
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) 
    {
    }
    
    public void setText(String message) 
    {
        m_lblMessage.setText(message);
    }
    
    /**
     * Simply closes the form.
     */
    @FXML
    protected void m_btnOKPressed() 
    {
        /**
         * Nothing special about the "OK" button;
         * we could get the active scene/window
         * through any control.
         */
        ((Stage)m_btnOK.getScene().getWindow()).close();
    }
    
   /**
     * Pop up a dialog box, typically for handling exceptions.
     * @param title
     * @param text
     * @throws java.io.IOException
     */
    public static void PopUpDialogBox(String title, String text) throws IOException
    {
        URL                   url        = WirelessFTJavaDemo.class.getResource("/wirelessftjavademo/userinterface/PopupDialog.fxml");
        FXMLLoader            loader     = new FXMLLoader(url);
        AnchorPane            page       = (AnchorPane)            loader.load();
        PopupDialogController controller = (PopupDialogController) loader.getController();                           
        Scene                 scene      = new Scene(page);
        Stage                 stage      = new Stage();

        stage.setTitle(title);
        stage.setScene(scene);
        stage.setResizable(false);
        controller.setText(text);
        stage.showAndWait();
    }
}