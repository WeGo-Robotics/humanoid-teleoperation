// Builds the whole scene at runtime.
//
// The committed scene file contains exactly one empty GameObject with this
// component on it. Everything else -- camera rig, passthrough, session, HUD --
// is constructed in Awake().
//
// This is deliberate. A Unity .unity file is generated YAML with GUID
// references into a package that gets upgraded; it cannot be reviewed in a
// diff, and it breaks in ways that only reproduce on the machine that opened
// it. The scene graph for this app is fifteen objects, so it is cheaper to
// write it as code that a reviewer can read and the build script can generate
// from nothing.
//
// The practical consequence: the Editor build script needs no knowledge of the
// Meta SDK at all. Every OVR reference in the project is in this file and
// TeleopSession.

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopBootstrap : MonoBehaviour
    {
        [Header("Host (set at build time by QuestBuild)")]
        public string HostAddress = "192.168.123.2";
        public int Port = 8443;
        public bool UseTls = false;

        [Header("Passthrough")]
        [Tooltip("Alignment happens in passthrough so the operator can see the " +
                 "real robot they are matching. Turning this off makes the " +
                 "align gate a guess.")]
        public bool Passthrough = true;

        /// <summary>Deferred out of Awake deliberately.
        ///
        /// XR plug-in management initialises on its own schedule, and in Awake
        /// it has not finished: XRGeneralSettings.Instance.Manager.activeLoader
        /// is still null, which is precisely what OVRManager.InitOVRManager()
        /// dereferences. Building the rig there produced three failures that
        /// looked unrelated and were not:
        ///
        ///   * a NullReferenceException out of AddComponent&lt;OVRManager&gt;()
        ///   * passthrough never enabling, so the operator saw a black room
        ///   * OVRCameraRig's anchors not existing yet, so FindHead() returned
        ///     null and the HUD anchored to this object at the world origin --
        ///     a panel down by the floor instead of in front of the face
        ///
        /// Waiting for the loader before touching any of it fixes all three.
        /// See docs section 15.2.</summary>
        private System.Collections.IEnumerator Start()
        {
            var waited = 0f;
            while (!XrReady() && waited < XrWaitTimeout)
            {
                waited += Time.unscaledDeltaTime;
                yield return null;
            }
            if (!XrReady())
            {
                // Build anyway rather than leaving a dead app: tracking reads
                // through UnityEngine.XR still work in some of these states,
                // and a HUD that says DISCONNECTED is more debuggable than a
                // black screen with nothing in it.
                Debug.LogWarning($"[Teleop] XR loader still not active after {waited:F1}s; " +
                                 "building the rig anyway. Expect passthrough off and " +
                                 "a possibly mis-anchored HUD.");
            }

            var rig = BuildCameraRig();
            var head = FindHead(rig);
            if (head == null)
                Debug.LogWarning("[Teleop] no centre-eye camera; HUD will anchor to the " +
                                 "scene origin rather than the operator's head.");

            if (Passthrough) EnablePassthrough(head);

            // Built on an inactive child, configured, and only then switched
            // on. AddComponent runs OnEnable synchronously, so a component
            // added to a live object has already acted on its defaults before
            // the next line can configure it. For TeleopSession that means the
            // websocket was opened against the field default rather than the
            // address baked in by QuestBuild -- invisible while the two agreed,
            // and a connection to the wrong machine as soon as they did not.
            var host = new GameObject("TeleopRuntime");
            host.SetActive(false);
            host.transform.SetParent(transform, false);

            var session = host.AddComponent<TeleopSession>();
            session.HostAddress = HostAddress;
            session.Port = Port;
            session.UseTls = UseTls;

            var hud = host.AddComponent<TeleopHud>();
            hud.Session = session;
            hud.Anchor = head != null ? head.transform : transform;

            // The spatial half of the guide. Separate from the HUD on purpose:
            // the HUD is a billboard that can never leave view, while these are
            // objects in the room that are supposed to.
            var guide = host.AddComponent<TeleopAlignGuide>();
            guide.Session = session;
            guide.HeadAnchor = head != null ? head.transform : transform;

            host.SetActive(true);

            Debug.Log($"[Teleop] rig built after {waited:F2}s; head={(head != null ? head.name : "null")} " +
                      $"passthrough={Passthrough}");
        }

        private const float XrWaitTimeout = 10f;

        private static bool XrReady()
        {
            var settings = UnityEngine.XR.Management.XRGeneralSettings.Instance;
            return settings != null && settings.Manager != null
                   && settings.Manager.activeLoader != null;
        }

        /// <summary>OVRCameraRig builds its own anchor hierarchy in Awake via
        /// EnsureGameObjectIntegrity(), so adding the component to a bare
        /// GameObject is equivalent to instantiating the shipped prefab -- and
        /// does not break when the package moves the prefab's path.</summary>
        private static OVRCameraRig BuildCameraRig()
        {
            var go = new GameObject("OVRCameraRig");
            var rig = go.AddComponent<OVRCameraRig>();
            var manager = go.AddComponent<OVRManager>();

            // Presence must keep being reported while the headset is off the
            // head; see docs section 10.3. This is the app-level half of that --
            // the manifest's focus-aware flag and the MDM sleep settings are the
            // other two.
            Application.runInBackground = true;

            // Floor level, so the head height the host receives is height above
            // the floor. The OpenXR->robot chain subtracts the head from the
            // wrists, so a stage-relative origin would put the operator's hands
            // in a different place for every guardian setup.
            manager.trackingOriginType = OVRManager.TrackingOrigin.FloorLevel;
            return rig;
        }

        private static Camera FindHead(OVRCameraRig rig)
        {
            if (rig != null && rig.centerEyeAnchor != null)
            {
                var cam = rig.centerEyeAnchor.GetComponent<Camera>();
                if (cam != null) return cam;
            }
            return Camera.main;
        }

        /// <summary>Underlay passthrough: the camera clears to transparent and
        /// the passthrough layer renders behind everything Unity draws, so the
        /// HUD floats over the real room.</summary>
        private static void EnablePassthrough(Camera head)
        {
            // Clearing to transparent is only correct if something is actually
            // composited behind it. With passthrough off, transparent-black is
            // just black, and the operator gets an unreadable void with a HUD
            // floating in it -- which is what this looked like on device before
            // the rig was deferred past XR init. So the camera setup is
            // conditional on passthrough really being on, and the fallback is a
            // dim opaque background that the HUD is at least legible against.
            var mgr = OVRManager.instance;
            var on = false;
            if (mgr != null)
            {
                mgr.isInsightPassthroughEnabled = true;
                var layerGo = new GameObject("PassthroughLayer");
                var layer = layerGo.AddComponent<OVRPassthroughLayer>();
                layer.overlayType = OVROverlay.OverlayType.Underlay;
                on = true;
            }
            else
            {
                Debug.LogWarning("[Teleop] no OVRManager, so passthrough is off. " +
                                 "Aligning against a blank background instead of " +
                                 "the real robot.");
            }

            if (head == null) return;
            head.clearFlags = CameraClearFlags.SolidColor;
            head.backgroundColor = on ? new Color(0f, 0f, 0f, 0f)
                                      : new Color(0.05f, 0.06f, 0.08f, 1f);
        }
    }
}
