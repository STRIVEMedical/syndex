using UnityEngine;
using System.IO.Ports;
using System;
public class SerialScanner : MonoBehaviour
{
    public SerialScanner()
    {
        ReadCOMs();
    }
    public string portName = "";
    void ReadCOMs()
    {
        // Get a list of all available port names
        string[] ports = SerialPort.GetPortNames();

        Debug.Log("Found " + ports.Length + " available COM ports:");

        foreach (string port in ports)
        {
            Debug.Log("Port: " + port);
            portName = port; // Set the portName to the last found port (you can modify this logic as needed)
        }
    }
}