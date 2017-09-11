/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package RTADiscoveryProtocol;

import java.net.*;

/**
 *
 * @author fleda
 * Ported from RTADiscoveryProtocolClient.cs
 */
public class RTADevice 
{
    public InetAddress m_ipa;            // The IP address of the device.
    public String      m_macstring;      // String representation of mac address
    public InetAddress m_ipaGateway;     // The device's default gateway.
    public InetAddress m_ipaNetmask;     // The network mask of the device.
    public String      m_strApplication; // The application description string of the device, i.e. what it is, its name.
    public String      m_strLocation;    // The location of the device. This is a string provided by the Wnet.

    /// <summary>
    /// Gets a description of this device.
    /// </summary>
    /// <returns>A user-friendly description of the device.</returns>
    @Override
    public String toString()
    {
        //Convert IP Address to good looking string. toString() works but puts a "/" in front of the ip address
        byte   tempByteArray[] = m_ipa.getAddress();
        String tempString      = new String();

        for (int i = 0; i < 4; i++) 
        {
            int x = tempByteArray[i];
            
            if (x < 0) x += 256; // Compensate for the lack of an unsigned char type.

            tempString += String.format("%d", x);
            
            if (i < 3)
            {
                tempString += ".";
            }
        }
        
        return String.format("IP=%1$-17sMAC=%2$-21sINFO=%3$-52s LOC=%4$s", tempString, m_macstring, m_strApplication, m_strLocation);
    }

    /// <summary>
    /// Compares two RTADevices for equality.
    /// </summary>
    /// <param name="obj">The RTADevice to compare against.</param>
    /// <returns>True if obj has the same property values as this.</returns>
    public boolean Equals(Object obj)
    {
        RTADevice  comparee = (RTADevice)obj;
        
        return (   comparee.m_ipa.equals(m_ipa) 
                && comparee.m_ipaGateway.equals(m_ipaGateway) 
                && comparee.m_ipaNetmask.equals(m_ipaNetmask) 
                && comparee.m_macstring.equals(m_macstring) 
                && comparee.m_strApplication.equals(m_strApplication)
                && comparee.m_strLocation.equals(m_strLocation));
    }
}