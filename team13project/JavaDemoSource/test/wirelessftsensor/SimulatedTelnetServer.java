/* 
 * Copyright 2013
 * ATI Industrial Automation banana
 */
package wirelessftsensor;

import java.io.IOException;
import java.net.ServerSocket;

/**
 * Simulates the telnet server on the Wireless F/T to aid in testing.
 * @author Sam Skuce
 */
public class SimulatedTelnetServer {
    
    public ServerSocket TelnetServerSocket = new ServerSocket(23);
    
    /**
     * Opens server socket.
     * @throws IOException If there is an error opening the server socket.
     */
    public SimulatedTelnetServer() throws IOException
    {
        
    }
    
}
