/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo;

import java.io.IOException;
import java.net.URL;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;

/**
 * Runs the application in one of two different
 * view modes based on screen resolution.
 * 
 * @author Sam Skuce, Chris Collins
 */
public class WirelessFTJavaDemo extends Application 
{
    
    /**
     * Starts the application with a primary
     * window in which to view the main stage.
     * 
     * @param stage 
     */
    @Override
    public void start(Stage stage) 
    {
        try 
        {
            URL        url;
            FXMLLoader loader;
            AnchorPane page;
            Scene      scene;
            
            stage.setResizable(false);
            
            url    = WirelessFTJavaDemo.class.getResource("/wirelessftjavademo/userinterface/StartupScreen.fxml");
            loader = new FXMLLoader(url);
            page   = (AnchorPane) loader.load();
            scene  = new Scene(page);
            
            stage.setTitle("Wireless F/T Profile Wizard");
            stage.getIcons().add(new Image("/wirelessftjavademo/userinterface/atiLogo.png"));
            stage.setScene(scene);
            stage.show();
        } 
        catch(IOException ioe) 
        {
            // Something went unimaginably wrong.
        }
    }
    
    /**
     * The main() method is ignored in correctly deployed JavaFX application.
     * main() serves only as fallback in case the application can not be
     * launched through deployment artifacts, e.g., in IDEs with limited FX
     * support. NetBeans ignores main().
     *
     * @param args the command line arguments
     */
    public static void main(String[] args) 
    {
        launch(args);
    }
}