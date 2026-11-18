// Unity side of the Pi <-> Unity Bluetooth link (protocol: see base/include/unity_link.h).
// Requires: Project Settings > Player > Api Compatibility Level = ".NET Framework".
// Attach to a GameObject and set portName to the Windows COM port of the paired Pi.
using System;
using System.Globalization;
using System.IO.Ports;
using UnityEngine;

public class PiBluetoothLinkSimple : MonoBehaviour
{
    public string portName = "COM5";
    public Vector3 LatestPosition;   // updated from the Pi's "POS x y z" lines

    SerialPort port;

    void Start()
    {
        port = new SerialPort(portName, 115200);
        port.NewLine = "\n";
        port.ReadTimeout = 10;
        port.Open();
    }

    void Update()
    {
        if (port == null || !port.IsOpen) return;
        try
        {
            while (port.BytesToRead > 0)
            {
                string line = port.ReadLine().Trim();
                string[] p = line.Split(' ');

                if (p[0] == "POS" && p.Length == 4)
                {
                    LatestPosition = new Vector3(
                        float.Parse(p[1], CultureInfo.InvariantCulture),
                        float.Parse(p[2], CultureInfo.InvariantCulture),
                        float.Parse(p[3], CultureInfo.InvariantCulture));
                }
                else if (p[0] == "ERR")
                {
                    Debug.LogWarning("Pi error: " + line);
                }
            }
        }
        catch (TimeoutException) { }
    }

    // Call this to send a command to the Pi, e.g. SendCommand(1).
    public void SendCommand(int id)
    {
        if (port != null && port.IsOpen) port.WriteLine("CMD " + id);
    }

    void OnDestroy()
    {
        if (port != null && port.IsOpen) port.Close();
    }
}
