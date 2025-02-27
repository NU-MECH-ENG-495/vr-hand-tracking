using UnityEngine;
using System.Collections;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;

public class HandTracking : MonoBehaviour
{
    // Oculus Integration SDK objects for tracking hands.
    private OVRHand hand;
    private OVRSkeleton handSkeleton;
    private OVRSkeleton.SkeletonType handType;

    // ID's to identify the tip bone for each finger.
    private int[] tips =
    {
        (int)OVRSkeleton.BoneId.Hand_PinkyTip,
        (int)OVRSkeleton.BoneId.Hand_RingTip,
        (int)OVRSkeleton.BoneId.Hand_MiddleTip,
        (int)OVRSkeleton.BoneId.Hand_IndexTip,
        (int)OVRSkeleton.BoneId.Hand_ThumbTip
    };

    // Arrays to store calibration data for each finger's
    // tip threshold, minimum and maximum flexion, and the number of calibration cycles.
    private float[] tipThresholds = new float[5];
    private float[] minFlexions = new float[5];
    private float[] maxFlexions = new float[5];
    private int[] cycles = new int[5];

    // Boolean flags to track the state of the calibration process.
    private bool calibrationActive = false;
    private bool allFingersCalibrated = false;

    // Store the sum of all fingers interpreted as bits:
    // finger extended -> 1 ; finger flexed -> 0.
    // Thumb is the most significant bit and pinky the least.
    private bool[] isFingerExtended = new bool[5];
    private int _handSum;
    public int HandSum
    {
        get { return _handSum; }
    }

    // UDP networking fields.
    public string remoteIP = "192.168.0.102"; // Replace with your PC's IP address.
    public int remotePort = 9000; // Replace with your desired port.
    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;

    void Start()
    {
        // Get these components from the same GameObject this script is attached to.
        hand = GetComponent<OVRHand>();
        handSkeleton = GetComponent<OVRSkeleton>();
        // Read whether this is a right hand or left hand.
        handType = handSkeleton.GetSkeletonType();

        // Set up UDP client.
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(IPAddress.Parse(remoteIP), remotePort);
    }

    void Update()
    {
        if (!hand.IsTracked)
        {
            LogHUD("Not Tracking ...");
            return;
        }

        if (hand.IsTracked && !calibrationActive)
        {
            calibrationActive = true;
            StartCoroutine(CalibrateFingers());
        }

        for (int i = 0; i < tips.Length; i++)
        {
            // During calibration, adjust the minFlexions and maxFlexions.
            float extension = GetFingerExtension(i);
            if (calibrationActive)
            {
                // Adjust calibration values.
                GetFingerExtension(i);
            }
            else
            {
                // Determine if finger is extended based on the threshold.
                isFingerExtended[i] = extension >= tipThresholds[i];
            }
        }

        if (!calibrationActive && allFingersCalibrated)
        {
            // Print joint angles (flexion for all joints, plus MCP abduction/adduction for fingers)
            PrintJointAngles();
        }
    }

    IEnumerator CalibrateFingers()
    {
        // Set up base flexion values per finger and initialize counter.
        for (int i = 0; i < 5; i++)
        {
            // min starts at max value, max starts at min and they evolve towards
            // each other to find the mid-point for each tip to wrist distance.
            minFlexions[i] = float.MaxValue;
            maxFlexions[i] = float.MinValue;
            cycles[i] = 0;
        }

        float[] prevExtension = new float[5];
        while (!allFingersCalibrated)
        {
            // Each finger will get a chance to contradict that all fingers are calibrated.
            allFingersCalibrated = true;
            LogHUD("Calibrating...");
            for (int i = 0; i < 5; i++)
            {
                float extension = GetFingerExtension(i);
                minFlexions[i] = Mathf.Min(minFlexions[i], extension);
                maxFlexions[i] = Mathf.Max(maxFlexions[i], extension);
                tipThresholds[i] = (minFlexions[i] + maxFlexions[i]) / 2f;

                // Count calibration cycles based on crossing the threshold.
                if (prevExtension[i] < tipThresholds[i] && extension >= tipThresholds[i])
                {
                    cycles[i]++;
                }
                else if (prevExtension[i] >= tipThresholds[i] && extension < tipThresholds[i])
                {
                    cycles[i]++;
                }
                prevExtension[i] = extension;

                if (cycles[i] < 6)
                {
                    allFingersCalibrated = false;
                }
            }
            yield return null;
        }
        calibrationActive = false;
    }

    float GetFingerExtension(int _fingerId)
    {
        // Get tip bone for current finger.
        OVRBone bone = handSkeleton.Bones[(int)tips[_fingerId]];
        // Calculate the position of the tip bone relative to the handSkeleton object.
        Vector3 distance = handSkeleton.transform.InverseTransformPoint(bone.Transform.position);
        Vector3 distAbs = Abs(distance);
        // The thumb flexes on a different plane than the rest of the fingers.
        return bone.Id == OVRSkeleton.BoneId.Hand_ThumbTip ? distAbs.z : distAbs.x;
    }

    Vector3 Abs(Vector3 v)
    {
        return new Vector3(Mathf.Abs(v.x), Mathf.Abs(v.y), Mathf.Abs(v.z));
    }

    void PrintJointAngles()
    {
        string anglesOutput = "";
        // Loop over every bone in the hand skeleton.
        foreach (var bone in handSkeleton.Bones)
        {
            // Exclude certain bones from the output.
            if (bone.Id == OVRSkeleton.BoneId.Hand_Index3 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Middle3 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Ring3 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Pinky0 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Pinky3 ||
                bone.Id == OVRSkeleton.BoneId.Hand_IndexTip ||
                bone.Id == OVRSkeleton.BoneId.Hand_MiddleTip ||
                bone.Id == OVRSkeleton.BoneId.Hand_RingTip ||
                bone.Id == OVRSkeleton.BoneId.Hand_PinkyTip ||
                bone.Id == OVRSkeleton.BoneId.Hand_ThumbTip ||
                bone.Id == OVRSkeleton.BoneId.Hand_WristRoot ||
                bone.Id == OVRSkeleton.BoneId.Hand_ForearmStub)
            {
                continue;
            }

            float flexionAngle = 0f;
            float abductionAngle = 0f;

            // Compute signed flexion angle relative to parent.
            if (bone.Transform.parent != null)
            {
                // Here we assume that flexion/extension occurs around the parent's right axis.
                // Using parent's forward vectors as the comparison.
                flexionAngle = Vector3.SignedAngle(
                    bone.Transform.parent.up,
                    bone.Transform.up,
                    bone.Transform.parent.forward);
            }

            // For MCP joints (index, middle, ring, pinky), compute signed abduction/adduction.
            // We'll compute this as the signed angle between the parent's up vector and the bone's up vector,
            // using the parent's forward vector as the rotation axis.
            bool isMCP = false;
            if (bone.Id == OVRSkeleton.BoneId.Hand_Index1 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Middle1 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Ring1 ||
                bone.Id == OVRSkeleton.BoneId.Hand_Pinky1)
            {
                isMCP = true;
                abductionAngle = Vector3.SignedAngle(
                    bone.Transform.parent.forward,
                    bone.Transform.forward,
                    bone.Transform.parent.right);
            }

            // Append to output.
            if (isMCP)
            {
                anglesOutput += bone.Transform.name +
                                ": Flexion: " + flexionAngle.ToString("F2") + "°, " +
                                "Abduction: " + abductionAngle.ToString("F2") + "°\n";
            }
            else
            {
                anglesOutput += bone.Transform.name + ": Flexion: " + flexionAngle.ToString("F2") + "°\n";
            }
        }
        // Log and send the joint angles.
        LogHUD("Joint Angles:\n" + anglesOutput);
        SendJointAngles(anglesOutput);
    }

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

    void LogHUD(string message)
    {
        LogManager.Instance.Log(this.GetType() + ":" + handType + ": " + message);
    }
}
