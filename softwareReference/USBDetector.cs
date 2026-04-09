using System.IO.Ports;
using System;
using UnityEngine;

namespace USB
{

    public class USBDetector
    {
        public string portName = "";
        public string hardwareId = "";
        
        /// <summary>
        /// Find Teensy COM port. Returns first available COM port (single device scenario).
        /// If serialNum is provided, can be used for future filtering.
        /// </summary>
        public (string port, string hwid) FindTeensyComPort(string serialNum = null)
        {
            try
            {
                string[] ports = SerialPort.GetPortNames();
                
                if (ports.Length == 0)
                {
                    Debug.Log("No COM ports found.");
                    return (null, null);
                }
                
                // Return first available COM port (since we're typically connecting to a single Teensy)
                return (ports[0], null);
            }
            catch (Exception ex)
            {
                Debug.LogError($"Error querying COM ports: {ex.Message}");
            }
            
            return (null, null);
        }
        
        /// <summary>
        /// Legacy method: Find first available COM port or first Teensy device.
        /// </summary>
        public void ReadCOMs()
        {
            // First, try to find a Teensy device
            (string teensyPort, string teensyHwid) = FindTeensyComPort();
            
            if (!string.IsNullOrEmpty(teensyPort))
            {
                portName = teensyPort;
                hardwareId = teensyHwid;
                Debug.Log($"Teensy found on: {portName}");
                if (!string.IsNullOrEmpty(hardwareId))
                {
                    Debug.Log($"Hardware ID: {hardwareId}");
                }
                return;
            }
            
            // Fallback: use standard COM port detection
            string[] ports = SerialPort.GetPortNames();
            Debug.Log("Found " + ports.Length + " available COM ports:");
            if (ports.Length == 1)
            {
                portName = ports[0];
                Debug.Log("Using COM port: " + portName);
            }
            else if (ports.Length > 0)
            {
                Debug.LogWarning("Multiple COM ports found. Please ensure only the intended device is connected.");
            }
            else
            {
                Debug.LogWarning("No COM ports found.");
            }
        }
    }


}