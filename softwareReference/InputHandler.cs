using UnityEngine;
using UnityEngine.InputSystem;
using PacketProcessor;

/// <summary>
/// InputHandler maps keyboard input to USB and PacketProcessor actions.
/// It demonstrates how to subscribe to packet events (PONG) and measure timeouts for responses.
/// </summary>
public class InputHandler : MonoBehaviour
{
    // Cached references
    private USB.USBManager _usbManager;

    // Track whether we've already subscribed to the Pong response to avoid duplicate subscriptions.
    private bool _hasSubscribedToPong = false;

    // State machine for waiting for a Pong after sending Ping.
    private bool _isWaitingForPong = false;

    // Ping timeout logic: simple timer measured in Update loop.
    private float _pingTimeoutTimer = 0f;
    private const float PingTimeoutDuration = 1f; // 1 second timeout for pong response

    // Volatile flag updated from the packet handler callback to indicate a pong was received.
    // Marked volatile because it may be written on a thread other than the Unity main thread.
    private volatile bool _pongReceivedFlag = false; // Flag to indicate pong received

    private void Start()
    {
        // FindFirstObjectByType<T> is a Unity helper to locate the USBManager instance in the scene.
        // Cache it to avoid repeated scene searches.
        _usbManager = FindFirstObjectByType<USB.USBManager>();

        if (_usbManager == null)
            Debug.LogError("InputHandler: Could not find USBManager in scene.");

        if (PacketProcessor.PacketProcessor.Instance == null)
            Debug.LogWarning("InputHandler: PacketProcessor instance not found yet. It may not be initialized.");
    }

    private void Update()
    {
        // If we are waiting for a Pong, check the response flag set by the packet handler,
        // or update the timeout timer and handle expiration.
        if (_isWaitingForPong)
        {
            if (_pongReceivedFlag)
            {
                Debug.Log("<color=green>InputHandler: Pong received within timeout.</color>");
                _isWaitingForPong = false;
                _pongReceivedFlag = false; // Reset flag for next ping
            }
            else
            {
                _pingTimeoutTimer += Time.deltaTime;
                if (_pingTimeoutTimer >= PingTimeoutDuration)
                {
                    Debug.LogWarning("<color=red>InputHandler: Pong response timed out.</color>");
                    _isWaitingForPong = false;
                    _pingTimeoutTimer = 0f; // Reset timer
                }
            }
        }


        // Space � Connect to Teensy
        if (Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            if (_usbManager != null)
            {
                Debug.Log("InputHandler: Space pressed � connecting to Teensy.");
                _usbManager.ConnectToTeensy();
            }
            else
            {
                Debug.LogWarning("InputHandler: USBManager not available.");
            }
        }

        // D � Disconnect from Teensy
        if (Keyboard.current.dKey.wasPressedThisFrame)
        {
            if (_usbManager != null)
            {
                Debug.Log("InputHandler: D pressed � disconnecting from Teensy.");
                _usbManager.Disconnect();
            }
            else
            {
                Debug.LogWarning("InputHandler: USBManager not available.");
            }
        }

        // P � Send Ping
        if (Keyboard.current.pKey.wasPressedThisFrame)
        {
            Debug.Log($"InputHandler: P pressed (USB connected: {USB.USBManager.IsConnected}, processor ready: {PacketProcessor.PacketProcessor.Instance != null}).");

            if (PacketProcessor.PacketProcessor.Instance != null)
            {
                // Subscribe to pong response only once. Subscribe registers a callback delegate with the PacketProcessor's dictionary.
                // When PacketProcessor dispatches a RESP_PONG packet, this OnPongReceived method will be invoked.
                if (!_hasSubscribedToPong)
                {
                    PacketProcessor.PacketProcessor.Instance.Subscribe(PacketType.RESP_PONG, OnPongReceived);
                    _hasSubscribedToPong = true;
                }

                // Start the ping request if we are not already waiting for a pong.
                if (!_isWaitingForPong)
                {
                    Debug.Log("InputHandler: P pressed � sending ping.");
                    _isWaitingForPong = true;
                    _pongReceivedFlag = false;
                    _pingTimeoutTimer = 0f; // Reset timer for new ping
                    bool pingQueued = PacketProcessor.PacketProcessor.Instance.SendPing();
                    if (!pingQueued)
                    {
                        Debug.LogWarning("InputHandler: Ping enqueue failed; aborting wait-for-pong state.");
                        _isWaitingForPong = false;
                    }
                }
                else
                {
                    Debug.LogWarning("InputHandler: Already waiting for a pong response. Please wait.");
                    return;
                }
            }
            else
            {
                Debug.LogWarning("InputHandler: PacketProcessor instance not available. Is USB connected?");
            }
        }
    }

    /// <summary>
    /// Called when PacketProcessor dispatches a RESP_PONG packet.
    /// This sets a flag that the Update loop observes to mark success.
    /// Note: Packet callbacks may execute on non-main threads depending on how the dispatch code is written;
    /// using a volatile flag is a simple thread-safe signal to the Update loop.
    /// </summary>
    private void OnPongReceived(Packet packet)
    {
        if (_isWaitingForPong)
        {
            _pongReceivedFlag = true; // Set flag to indicate pong was received
        }
    }
}