/*
 * Copyright 2013
 * ATI Industrial Automation
 */
package wirelessftjavademo;

import java.util.ArrayList;

/**
 * The Wi-Fi and IP settings for a Wireless F/T.
 * @author Sam Skuce
 */
public class IPSettings {
    /**
     * The possible antenna settings.
     */
    public enum AntennaSetting
    {
        External,
        Internal
    };
    
    /**
     * The antenna setting.
     */
    public AntennaSetting Antenna;
    
    /**
     * The possible band settings.
     */
    public enum BandSetting{
        Spectrum2_4Ghz,
        Spectrum5Ghz
    };
    
    /**
     * The band setting.
     */
    public BandSetting Band = BandSetting.Spectrum2_4Ghz;
    
    /**
     * Whether or not the DHCP is turned on.
     */
    public boolean DHCP = true;
    
    /**
     * The IP address.
     */
    public String IPAddress = "192.168.1.20";
    
    /**
     * The default gateway.
     */
    public String DefaultGateway = "192.168.1.1";
    
    /**
     * The subnet mask.
     */
    public String SubnetMask = "255.255.255.0";
    
    /** The Wi-Fi SSID. */
    public String SSID = "Test";
    
    /**
     * Initializes IP settings based on IP command response from the Wireless F/T.
     * @param ipCommandResponse 
     */
    public IPSettings(String ipCommandResponse)
    {
        String   antenna = getFieldValue(ipCommandResponse, "ANTENNA", "\r\n").trim();
        String[] ant     = antenna.split(" ");
        
        switch (ant[0])
        {
            case "External":
                Antenna = AntennaSetting.External;
                break;
            case "Internal":
                Antenna = AntennaSetting.Internal;
                break;
            default:
                throw new IllegalArgumentException("Unexpected value for antenna setting.");
        }
        
        switch (getFieldValue(ipCommandResponse, "BAND", "\r\n").trim())
        {
            case "2.4 GHz":
                Band = BandSetting.Spectrum2_4Ghz;
                break;
            case "5 GHz":
                Band = BandSetting.Spectrum5Ghz;
                break;
            default:
                throw new IllegalArgumentException("Unexpected value for band setting.");
        }
        
        String dhcp;
        dhcp           = getFieldValue(ipCommandResponse, "NET DHCP", "\r\n").trim();
        DHCP           = (dhcp.toUpperCase().contains("ON"));
        IPAddress      = getFieldValue(ipCommandResponse, "DEVIP",    "\r\n").trim();
        DefaultGateway = getFieldValue(ipCommandResponse, "GATEIP",   "\r\n").trim();
        SSID           = getFieldValue(ipCommandResponse, "SSID",     "\r\n").trim();
        SubnetMask     = getFieldValue(ipCommandResponse, "NETMASK",  "\r\n").trim();
        
        String[]   splitIP      = IPAddress     .split(" ");
        String[]   splitGateway = DefaultGateway.split(" ");
        String[]   splitSSID    = SSID          .split(" ");
        String[]   splitMask    = SubnetMask    .split(" ");
        String[][] split        = {splitIP, splitGateway, splitSSID, splitMask};
        
        ArrayList<String> ipData      = new ArrayList<>();
        ArrayList<String> gatewayData = new ArrayList<>();
        ArrayList<String> SSIDData    = new ArrayList<>();
        ArrayList<String> maskData    = new ArrayList<>();
        ArrayList[]       data        = {ipData, gatewayData, SSIDData, maskData};
        
        for (int i = 0; i < split.length; i++) 
        {
            for (String item : split[i]) 
            {
                if (!item.isEmpty()) 
                {
                    data[i].add(item);
                }
            }
        }
        IPAddress      = ipData     .get(0);
        DefaultGateway = gatewayData.get(1);
        SSID           = SSIDData   .get(1);
        SubnetMask     = maskData   .get(1);
    }
    
    private static String getFieldValue(String commandResponse, String beginMarker, String endMarker )
    {
        int startIndex = commandResponse.indexOf(beginMarker) + beginMarker.length();
        int endIndex   = commandResponse.indexOf(endMarker, startIndex);
        
        if ( startIndex == -1 || endIndex == -1 )
        {
            throw new IllegalArgumentException("Field begin or end not found in command response.");
        }
        
        return commandResponse.substring(startIndex, endIndex);
    }
   
    /** 
     * Default constructor.
     */
    public IPSettings()
    {
    }
}