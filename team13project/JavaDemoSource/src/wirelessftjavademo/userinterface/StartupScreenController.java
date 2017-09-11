package wirelessftjavademo.userinterface;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.util.ResourceBundle;
import java.util.prefs.Preferences;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import javafx.scene.image.Image;
import javafx.scene.layout.AnchorPane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import wirelessftjavademo.WirelessFTDemoProfile;
import wirelessftjavademo.WirelessFTJavaDemo;

/**
 * Startup screen/profile
 * editor for the main demo.
 *
 * @author colch
 */
public class StartupScreenController implements Initializable 
{
    @FXML
    TextField m_txtProfile;
    
    @FXML
    Button m_btnCreate;
    
    @FXML
    Button m_btnEdit;
    
    @FXML
    Button m_btnDots;
    
    @FXML
    CheckBox m_chkSave;
    
    @FXML
    Button m_btnStart;
    
    @FXML
    Button m_btnClose;
    
    /**
     * The version of this demo.
     */
    public static String VERSION = "1.0.2";
    
    /**
     * The screen resolution necessary to view the demo
     * in its default mode (up to 6 graphs). If the primary
     * screen has a smaller height or width than these values,
     * the compressed view is used where a single graph must
     * be chosen to view at a time.
     * 
     * NOTE: File writing and other functions are still
     * available even when only one graph is shown.
     */
    public static final double PREF_WIDTH  = 1280.0;
    public static final double PREF_HEIGHT =  760.0; // was 984
    
    /**
     * Stage minimums (based on view mode).
     */
    private final int MIN_WIDTH        = 1220;
    private final int MIN_HEIGHT       =  800;
    
    /**
     * The currently-active WNet profile,
     * initially, the default profile.
     */
    private WirelessFTDemoProfile m_profile = new WirelessFTDemoProfile();
    
    /**
     * This flag is set by the WNet profile
     * wizard when the last-changed file should
     * be set as the new default file.
     */
    private boolean m_makeDefault = false;
    
    // ---------- Preference keys ---------- \\
    
    /**
     * Keeps track of user preferences.
     */
    protected static Preferences m_prefsRoot;
    
    /**
     * The preferences user root node pathname.
     */
    public static final String PREF_USER_ROOT_NODE_PATHNAME = "com.FTDemo.preference.Settings";
    
    /**
     * The folder in which profiles are presumably stored.
     */
    public static final String PREF_PROFILE_DIRECTORY = "profile directory";
    
    /**
     * The default profile.
     */
    public static final String PREF_DEFAULT_PROFILE = "default profile";
    
    // ---------- Methods ---------- \\
    
    /**
     * Initializes the controller class (Not used).
     * 
     * @param url Not used.
     * @param rb  Not used.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) 
    {
        m_prefsRoot = Preferences.userRoot(); // Establish a preferences file.
        
        Preferences prefs = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        m_txtProfile.setText(prefs.get(PREF_DEFAULT_PROFILE, null));
    }
    
    public void setDefaultFlag() 
    {
        m_makeDefault = true;
    }
    
   /**
     * Create Button Pressed
     * 
     * Allows the user to create and edit a new
     * WNet profile, then save and (optionally)
     * select it as the new starting profile.
     */
    @FXML
    protected void m_btnCreatePressed() 
    {
        // Set preferred profile directory.
        Preferences prefs    = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_PROFILE_DIRECTORY, null);
        FileChooser chooser  = GetProfilePath(pathname, "Save new profile");
        File        profile  = chooser.showSaveDialog(((Node)m_btnCreate).getScene().getWindow()); // Select profile
        
        if (profile != null)                     // If a new profile was chosen,
        {
            try 
            {
                if(!profile.getAbsolutePath().toLowerCase().endsWith(".xml")) 
                {
                    profile = new File(profile.getAbsolutePath() + ".xml");
                }

                if (profile.getParentFile().exists()) 
                {
                    prefs.put(PREF_PROFILE_DIRECTORY, profile.getParentFile().getAbsolutePath());
                }

                URL                                     url        = getClass().getResource("WirelessFTConfigurationScreen.fxml");
                FXMLLoader                              loader     = new FXMLLoader(url);
                AnchorPane                              page       = (AnchorPane) loader.load();
                WirelessFTConfigurationScreenController controller =              loader.getController();
                Scene                                   scene      = new Scene(page);
                Stage                                   stage      = new Stage();
                
                controller.setProfile(profile, false, this); // Do not read the existing profile. It does not exist until Finish is pressed.
                stage.setTitle("Wireless F/T Profile Wizard");
                stage.getIcons().add(new Image("/wirelessftjavademo/userinterface/atiLogo.png"));
                stage.setResizable(false);
                stage.setScene(scene);
                stage.showAndWait(); // Finish may be pressed while the wizard is open, and create the profile.

                if (m_makeDefault && profile.exists())               // If this is the default profile, and the profile exists,
                {
                    m_profile = new WirelessFTDemoProfile(profile);  // get profile,
                    m_txtProfile.setText(profile.getAbsolutePath()); // and save its path.
                }
            } 
            catch (Exception e) 
            {
                PopUpDialogBoxEx(e, "Invalid XML File", "Could not build a valid profile from " + profile.getName() + ": " + e.getMessage());
            }
        }
        
        m_makeDefault = false;
    }
    
    /**
     * Edit Button Pressed
     * 
     * Opens a window so the user can
     * select an existing WNet profile
     * and make changes to it.
     */
    @FXML
    protected void m_btnEditPressed() 
    {
        // Set preferred profile directory.
        Preferences prefs    = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_PROFILE_DIRECTORY, null);
        FileChooser chooser  = GetProfilePath(pathname, "Edit existing profile");
        File        profile  = chooser.showOpenDialog(((Node)m_btnEdit  ).getScene().getWindow()); // Select profile
        
        if (profile != null && profile.exists()) // If an existing profile was chosen,
        {
            try 
            {
                if(!profile.getAbsolutePath().toLowerCase().endsWith(".xml")) 
                {
                    profile = new File(profile.getAbsolutePath() + ".xml");
                }
                
                if (profile.getParentFile().exists()) 
                {
                    prefs.put(PREF_PROFILE_DIRECTORY, profile.getParentFile().getAbsolutePath());
                }
                
                URL                                     url        = getClass().getResource("WirelessFTConfigurationScreen.fxml");
                FXMLLoader                              loader     = new FXMLLoader(url);
                AnchorPane                              page       = (AnchorPane) loader.load();
                WirelessFTConfigurationScreenController controller =              loader.getController();
                Scene                                   scene      = new Scene(page);
                Stage                                   stage      = new Stage();
                
                controller.setProfile(profile, true, this); // Read the existing profile.
                stage.setTitle("Wireless F/T Profile Wizard");
                stage.getIcons().add(new Image("/wirelessftjavademo/userinterface/atiLogo.png"));
                stage.setResizable(false);
                stage.setScene(scene);
                stage.showAndWait(); // Finish may be pressed while the wizard is open, and modify the profile.
                
                if (m_makeDefault)                                   // If this is the default profile,
                {
                    m_profile = new WirelessFTDemoProfile(profile);  // get profile,
                    m_txtProfile.setText(profile.getAbsolutePath()); // and save its path.
                }
            } 
            catch (Exception e) 
            {
                PopUpDialogBoxEx(e, "Invalid XML File", "Could not build a valid profile from " + profile.getName() + ": " + e.getMessage());
            }
        }
        
        m_makeDefault = false;
    }
    
    /**
     * Choose Profile File Button Pressed (...)
     * 
     * Allows the user to select an XML file
     * corresponding to an existing WNet profile.
     * If the profile cannot be parsed into a
     * WirelessFTDemoProfile object, an error
     * message is shown.
     */
    @FXML
    protected void m_btnChooseProfileFile() 
    {
        // Set preferred profile directory.
        Preferences prefs    = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        String      pathname = prefs.get(PREF_PROFILE_DIRECTORY, null);
        FileChooser chooser  = GetProfilePath(pathname, "Choose profile");
        File        profile  = chooser.showOpenDialog(((Node)m_btnDots).getScene().getWindow()); // Select profile
        
        if (profile != null && profile.exists()) // If a profile was chosen, and that profile exists,
        {
            try 
            {
                m_profile = new WirelessFTDemoProfile(profile); // Get profile
                m_txtProfile.setText(profile.getAbsolutePath());
                
                if (profile.getParentFile().exists()) 
                {
                    prefs.put(PREF_PROFILE_DIRECTORY, profile.getParentFile().getAbsolutePath());
                }
            } 
            catch (Exception e) 
            {
                PopUpDialogBoxEx(e, "Invalid XML File", "Could not build a valid profile from " + profile.getName() + ": " + e.getMessage());
            }
        }
    }
    
    /**
     * Start Button Pressed
     * 
     * Loads in the current profile (or the default
     * profile, if one hasn't been selected) and
     * opens the main demo screen.
     */
    @FXML
    protected void m_btnStartPressed() 
    {
        try
        {
            if (m_txtProfile.getText().isEmpty()) 
            {
                m_profile = new WirelessFTDemoProfile();
            } 
            else 
            {
                m_profile = new WirelessFTDemoProfile(new File(m_txtProfile.getText()));
            }
        }
        catch (Exception e) 
        {
            String filename = m_txtProfile.getText();
            PopUpDialogBoxEx(e, "Profile load failed", "File \"" + filename + "\" does not exist.");
            return;
        }
                
        URL        url;
        FXMLLoader loader;
        AnchorPane page;
        Scene      scene;
        Stage      stage = new Stage();

        int        width  = MIN_WIDTH;
        int        height = MIN_HEIGHT;
            
        try
        {
            WirelessFTDemoMainScreenController controller;
            url        = WirelessFTJavaDemo.class.getResource("/wirelessftjavademo/userinterface/WirelessFTDemoMainScreen.fxml");
            loader     = new FXMLLoader(url);
            page       = (AnchorPane) loader.load();
            controller =              loader.getController();
            controller.setProfile(m_profile, m_chkSave.isSelected());
        } 
        catch (Exception e) 
        {
            e.printStackTrace();
            PopUpDialogBoxEx(e, "Demo Failed", "Could not initialize demo: " + e.getMessage());
            return;
        }

        scene = new Scene(page);
        stage.getIcons().add(new Image("/wirelessftjavademo/userinterface/atiLogo.png"));
        stage.setScene(scene);
        stage.setTitle("ATI Industrial Automation Wireless F/T Demo " + VERSION);

        scene.widthProperty().addListener(new ChangeListener<Number>() // Listen for Main Screen width changes
        {
            @Override 
            public void changed(ObservableValue<? extends Number> observableValue, Number oldSceneWidth, Number newSceneWidth) 
            {
                String path  = WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;
                String width = WirelessFTDemoMainScreenController.PREF_LAST_WINDOW_WIDTH;
                Preferences.userRoot().node(path).putDouble(width, (double) newSceneWidth);
            }
        });

        scene.heightProperty().addListener(new ChangeListener<Number>() // Listen for Main Screen height changes
        {
            @Override 
            public void changed(ObservableValue<? extends Number> observableValue, Number oldSceneHeight, Number newSceneHeight) 
            {
                String path   = WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;
                String height = WirelessFTDemoMainScreenController.PREF_LAST_WINDOW_HEIGHT;
                Preferences.userRoot().node(path).putDouble(height, (double) newSceneHeight);
            }
        });

        stage.xProperty().addListener(new ChangeListener<Number>() // Listen for Main Screen x movements
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number oldX, Number newX) 
            {
                String path  = WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;
                String lastX = WirelessFTDemoMainScreenController.PREF_LAST_X_POSITION;
                Preferences.userRoot().node(path).putDouble(lastX, (double) newX);
            }
        });

        stage.yProperty().addListener(new ChangeListener<Number>() // Listen for Main Screen y movements
        {
            @Override
            public void changed(ObservableValue<? extends Number> oldY, Number t, Number newY) 
            {
                String path  = WirelessFTDemoMainScreenController.PREF_USER_ROOT_NODE_PATHNAME;
                String lastY = WirelessFTDemoMainScreenController.PREF_LAST_Y_POSITION;
                Preferences.userRoot().node(path).putDouble(lastY, (double) newY);
            }
        });

        Preferences prefs = m_prefsRoot.node(PREF_USER_ROOT_NODE_PATHNAME);
        prefs.put(PREF_DEFAULT_PROFILE, m_txtProfile.getText());

        stage.setWidth (prefs.getDouble(WirelessFTDemoMainScreenController.PREF_LAST_WINDOW_WIDTH, 1000.0));
        stage.setHeight(prefs.getDouble(WirelessFTDemoMainScreenController.PREF_LAST_WINDOW_HEIGHT, 800.0));
        stage.setX     (prefs.getDouble(WirelessFTDemoMainScreenController.PREF_LAST_X_POSITION,        0));
        stage.setY     (prefs.getDouble(WirelessFTDemoMainScreenController.PREF_LAST_Y_POSITION,        0));
        stage.setMinWidth (width);
        stage.setMinHeight(height);
        stage.show(); // Open the Main Screen

        // Close the Startup Screen. Not kidding, this is the "easiest" way to close a JavaFX window.
        ((Stage)m_txtProfile.getScene().getWindow()).close();

        final WirelessFTDemoMainScreenController controller = (WirelessFTDemoMainScreenController) loader.getController();
        stage.setOnCloseRequest(new EventHandler<WindowEvent>() // Listen for Main Screen close
        {
            @Override
            public void handle(WindowEvent t) 
            {
                controller.OnCloseRequest();
                Platform.exit();
            }
        });
    }
    
   /**
     * Pop up a dialog box with exception handling, typically for handling exceptions.
     */
    private void PopUpDialogBoxEx(Exception e, String title, String text)
    {
        try 
        {
            PopupDialogController.PopUpDialogBox(title, text);
        } 
        catch (IOException ioe) 
        {
            e.printStackTrace();
            ioe.printStackTrace();
        }
    }

    /**
     * Get Profile Path.
     * 
     * Get the path to select a profile file from.
     */
    private FileChooser GetProfilePath(String pathname, String message)
    {
        FileChooser chooser = new FileChooser();
        
        chooser.setTitle(message);
        chooser.getExtensionFilters().add(new FileChooser.ExtensionFilter("XML Files (*.xml)", "*.XML"));
        
        if (pathname != null)                      // If there is a preferred directory (from last time),
        {
            try                                    // try to set it.
            {
                chooser.setInitialDirectory(new File(pathname));
            } 
            catch (Exception e)                    // If the preferred directory was not found in the file system,
            {
                chooser.setInitialDirectory(null); // set a default directory.
            }
        } 
        else                                       // If there is no preferred directory (first use),
        {
            chooser.setInitialDirectory(null);     // set a default directory.
        }

        return chooser;
    }
    
    /**
     * Close Button Pressed
     * 
     * Just closes the form.
     */
    @FXML
    protected void m_btnClosePressed() 
    {
        Platform.exit();
    }
}
