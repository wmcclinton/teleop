/*
 * QuestHandSender.cs
 * ------------------
 * Attach to a GameObject in your Unity Quest 3 scene.
 * Requires the Meta XR SDK (com.meta.xr.sdk.core).
 *
 * Setup
 *   1. Add an OVRCameraRig to the scene.
 *   2. On the RightHandAnchor's child "OVRHandPrefab":
 *        - Ensure OVRHand and OVRSkeleton components are present.
 *   3. Drag those components into the fields below in the Inspector.
 *   4. Set PC_IP to your computer's LAN IP address.
 *   5. Build for Android (Quest 3) with Hand Tracking enabled in
 *      Project Settings → Oculus → Hand Tracking Support = Required.
 */

using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

[RequireComponent(typeof(OVRHand))]
public class QuestHandSender : MonoBehaviour
{
    [Header("Network — set PC_IP to your computer's LAN IP")]
    [SerializeField] private string pcIP   = "192.168.1.108";
    [SerializeField] private int    pcPort = 5005;

    [Header("Hand components (auto-detected if left empty)")]
    [SerializeField] private OVRHand      hand;
    [SerializeField] private OVRSkeleton  skeleton;

    // How many packets to send per second (Quest runs at 72 Hz; 40 Hz is plenty)
    [Header("Send rate")]
    [SerializeField] private float sendHz = 40f;

    // ── private state ─────────────────────────────────────────────────────────
    private UdpClient  _udp;
    private IPEndPoint _endpoint;
    private float      _nextSendTime;

    // ── lifecycle ─────────────────────────────────────────────────────────────
    private void Awake()
    {
        if (hand     == null) hand     = GetComponent<OVRHand>();
        if (skeleton == null) skeleton = GetComponent<OVRSkeleton>();
    }

    private void Start()
    {
        try
        {
            _udp      = new UdpClient();
            _endpoint = new IPEndPoint(IPAddress.Parse(pcIP), pcPort);
            Debug.Log($"[QuestHandSender] Sending to {pcIP}:{pcPort}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[QuestHandSender] UDP init failed: {e.Message}");
            enabled = false;
        }
    }

    private void Update()
    {
        if (Time.time < _nextSendTime) return;
        _nextSendTime = Time.time + 1f / sendHz;

        if (!hand.IsTracked) return;
        if (skeleton.Bones == null || skeleton.Bones.Count == 0) return;

        // ── Wrist bone ───────────────────────────────────────────────────────
        var wristBone = skeleton.Bones[(int)OVRSkeleton.BoneId.Hand_WristRoot];
        Transform wristTF = wristBone.Transform;

        Vector3    wp = wristTF.position;
        Quaternion wr = wristTF.rotation;

        // ── Pinch strength ───────────────────────────────────────────────────
        float pinch = hand.GetFingerPinchStrength(OVRHand.HandFinger.Index);

        // ── Build JSON manually (avoids an extra dependency) ─────────────────
        // wrist_rot is (w, x, y, z) to match teleop.py
        string json = "{"
            + $"\"wrist_pos\":[{wp.x:F4},{wp.y:F4},{wp.z:F4}],"
            + $"\"wrist_rot\":[{wr.w:F4},{wr.x:F4},{wr.y:F4},{wr.z:F4}],"
            + $"\"pinch\":{pinch:F3}"
            + "}";

        byte[] payload = Encoding.UTF8.GetBytes(json);

        try
        {
            _udp.Send(payload, payload.Length, _endpoint);
        }
        catch (SocketException)
        {
            // Network blip — skip this frame silently
        }
    }

    private void OnDestroy()
    {
        _udp?.Close();
    }
}
