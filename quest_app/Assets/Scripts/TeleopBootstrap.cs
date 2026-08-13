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

        private void Awake()
        {
            var rig = BuildCameraRig();
            var head = FindHead(rig);

            if (Passthrough) EnablePassthrough(head);

            var session = gameObject.AddComponent<TeleopSession>();
            session.HostAddress = HostAddress;
            session.Port = Port;
            session.UseTls = UseTls;
            session.HeadTransform = head != null ? head.transform : null;

            var hud = gameObject.AddComponent<TeleopHud>();
            hud.Session = session;
            hud.Anchor = head != null ? head.transform : transform;
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
            OVRManager.instance.isInsightPassthroughEnabled = true;

            var layerGo = new GameObject("PassthroughLayer");
            var layer = layerGo.AddComponent<OVRPassthroughLayer>();
            layer.overlayType = OVROverlay.OverlayType.Underlay;

            if (head == null) return;
            head.clearFlags = CameraClearFlags.SolidColor;
            head.backgroundColor = new Color(0f, 0f, 0f, 0f);
        }
    }
}
