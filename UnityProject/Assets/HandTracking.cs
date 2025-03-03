using UnityEngine;
using Oculus.Interaction.Input;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System;

public enum HandSide { Left, Right }

public class HandJointAnglesLogger : MonoBehaviour
{
    [SerializeField]
    private bool _logToHUD = true;

    [SerializeField]
    private float _logFrequency = 0.01f; // Log every 0.01 seconds

    // New: specify which hand this logger is for.
    [SerializeField] private HandSide handSide;

    private IHand _hand;
    private float _timer = 0f;

    
    public string remoteIP = "255.255.255.255"; // Broadcast address
    public int remotePort = 9000; // Port to send data to
    private UdpClient udpClient;

    private IPEndPoint remoteEndPoint;

    private void Start()
    {
        _hand = GetComponent<IHand>();
        if (_hand == null)
        {
            LogHUD("HandJointAnglesLogger requires a component that implements IHand on the same GameObject");
            enabled = false;
            return;
        }

        // Subscribe to hand updates
        _hand.WhenHandUpdated += OnHandUpdated;

        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);
    }

    private void OnDestroy()
    {
        if (_hand != null)
        {
            _hand.WhenHandUpdated -= OnHandUpdated;
        }
    }

    private void OnHandUpdated()
    {
        // This will be called whenever the hand data is updated
        if (_logToHUD)
        {
            _timer += Time.deltaTime;
            if (_timer >= _logFrequency)
            {
                _timer = 0f;
                LogJointAngles();
            }
        }
    }

    private void LogJointAngles()
    {
        if (!_hand.IsTrackedDataValid)
        {
            LogHUD("Hand tracking not valid");
            return;
        }

        if (_hand.GetJointPosesLocal(out ReadOnlyHandJointPoses jointPoses))
        {
            string logMessage = "";
            // Process thumb data
            logMessage += GetThumbAngles(jointPoses);
            // Process each non-thumb finger
            logMessage += GetFingerAngles(jointPoses, HandFinger.Index);
            logMessage += GetFingerAngles(jointPoses, HandFinger.Middle);
            logMessage += GetFingerAngles(jointPoses, HandFinger.Ring);
            logMessage += GetFingerAngles(jointPoses, HandFinger.Pinky);
            LogHUD(handSide.ToString() + " hand Angles:\n" + logMessage);
            // Send the joint angles over UDP
            SendJointAngles(handSide.ToString() + " hand Angles:\n" +logMessage); 
        }
        else
        {
            LogHUD("Failed to get joint poses");
        }
    }

    /// <summary>
    /// Returns a multi-line string containing all thumb joint angles.
    /// Each line is formatted as "JointName: Angle".
    /// </summary>
    private string GetThumbAngles(ReadOnlyHandJointPoses jointPoses)
    {
        int[] indices = GetFingerJointIndices(HandFinger.Thumb);
        if (indices.Length < 4)
        {
            return "Insufficient joint data for Thumb\n";
        }

        // Transition: CMC → MCP
        Quaternion cmcToMcp = Quaternion.Inverse(jointPoses[indices[0]].rotation) * jointPoses[indices[1]].rotation;
        Vector3 cmcToMcpEuler = cmcToMcp.eulerAngles;
        float cmcMcpFlexion = NormalizeAngle(cmcToMcpEuler.x);
        float cmcMcpAdduction = NormalizeAngle(cmcToMcpEuler.y);

        // Transition: MCP → IP
        Quaternion mcpToIp = Quaternion.Inverse(jointPoses[indices[1]].rotation) * jointPoses[indices[2]].rotation;
        Vector3 mcpToIpEuler = mcpToIp.eulerAngles;
        float mcpIpFlexion = NormalizeAngle(mcpToIpEuler.x);
        float mcpIpAdduction = NormalizeAngle(mcpToIpEuler.y);

        // Transition: IP → TIP
        Quaternion ipToTip = Quaternion.Inverse(jointPoses[indices[2]].rotation) * jointPoses[indices[3]].rotation;
        Vector3 ipToTipEuler = ipToTip.eulerAngles;
        float ipTipFlexion = NormalizeAngle(ipToTipEuler.x);
        float ipTipAdduction = NormalizeAngle(ipToTipEuler.y);

        string s = "";
        s += $"Thumb CMC Flexion: {cmcMcpFlexion:F1}°\n";
        s += $"Thumb CMC Adduction: {cmcMcpAdduction:F1}°\n";
        s += $"Thumb MCP Flexion: {mcpIpFlexion:F1}°\n";
        s += $"Thumb MCP Adduction: {mcpIpAdduction:F1}°\n";
        // s += $"Thumb IP Flexion: {ipTipFlexion:F1}°\n";
        // s += $"Thumb IP Adduction: {ipTipAdduction:F1}°\n";
        return s;
    }

    /// <summary>
    /// Returns a multi-line string containing joint angles for a non-thumb finger.
    /// For non-thumb fingers, we compute:
    /// - MCP→PIP rotation (using Euler angles for flexion and adduction)
    /// - PIP→DIP rotation (using axis-angle for flexion)
    /// Each line is formatted as "JointName: Angle".
    /// </summary>
    private string GetFingerAngles(ReadOnlyHandJointPoses jointPoses, HandFinger finger)
    {
        int[] indices = GetFingerJointIndices(finger);
        if (indices.Length < 5)
        {
            return $"Insufficient joint data for {finger} finger\n";
        }

        // Compute MCP → PIP rotation
        Quaternion mcpToPip = Quaternion.Inverse(jointPoses[indices[1]].rotation) * jointPoses[indices[2]].rotation;
        Vector3 mcpEuler = mcpToPip.eulerAngles;
        float mcpFlexion = NormalizeAngle(mcpEuler.x);
        float mcpAdduction = NormalizeAngle(mcpEuler.y);

        // Compute PIP → DIP rotation using axis–angle (flexion only)
        Quaternion pipToDip = Quaternion.Inverse(jointPoses[indices[2]].rotation) * jointPoses[indices[3]].rotation;
        pipToDip.ToAngleAxis(out float pipAngle, out Vector3 pipAxis);
        float pipFlexion = NormalizeAngle(pipAngle);

        string s = "";
        s += $"{finger} MCP Flexion: {mcpFlexion:F1}°\n";
        s += $"{finger} MCP Adduction: {mcpAdduction:F1}°\n";
        s += $"{finger} PIP Flexion: {pipFlexion:F1}°\n";
        return s;
    }

    /// <summary>
    /// Normalizes an angle (in degrees) to the range [-180, 180].
    /// </summary>
    private float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f)
        {
            angle -= 360f;
        }
        return angle;
    }

    /// <summary>
    /// Returns joint indices for the given finger based on the standard Oculus hand skeleton.
    /// For the thumb, we assume a 4-joint chain (e.g. indices 1,2,3,4).
    /// For non-thumb fingers, we assume a 5-joint chain.
    /// Adjust these if your actual data uses different indexing.
    /// </summary>
    private int[] GetFingerJointIndices(HandFinger finger)
    {
        switch (finger)
        {
            case HandFinger.Thumb:
                return new int[] { 1, 2, 3, 4 };
            case HandFinger.Index:
                return new int[] { 5, 6, 7, 8, 9 };
            case HandFinger.Middle:
                return new int[] { 10, 11, 12, 13, 14 };
            case HandFinger.Ring:
                return new int[] { 15, 16, 17, 18, 19 };
            case HandFinger.Pinky:
                return new int[] { 20, 21, 22, 23, 24 };
            default:
                return new int[0];
        }
    }

    // New: Method to send joint angles over UDP
    void SendJointAngles(string message)
    {
        try
        {
            byte[] data = Encoding.UTF8.GetBytes(message);
            udpClient.Send(data, data.Length, remoteEndPoint);
        }
        catch (Exception ex)
        {
            LogHUD("UDP Send Error: " + ex.Message);
        }
    }

    // Helper method to log messages to the HUD on the channel for the current hand side.
    private void LogHUD(string message)
    {
        // Logs using the hand side as the source (e.g., "Left" or "Right")
        LogManager.Instance.Log(handSide.ToString(), message);
    }
}
