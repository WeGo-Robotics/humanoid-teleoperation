// Controller markers.
//
// A ring on each controller with a short stub showing which way it points, so
// the operator can see where the app thinks their hands are. When alignment
// will not close, the first question is always whether tracking is live, and
// without this nothing on screen could answer it.
//
// Why these are drawn from InputTracking rather than from Meta's
// OVRControllerPrefab / OVRControllerHelper: those resolve the controller
// through OVRInput, which resolves through OVRPlugin.GetControllerState6, and
// that call was measured returning empty ConnectedControllers and Buttons on
// this headset while the node-tracking pipe kept working (docs 14). Rendering
// the shipped controller model would have drawn nothing, or drawn it at the
// origin. The node pipe is the one that works here.
//
// This file used to also draw a wireframe body in the room. It has been
// removed: a stick figure is not something to ship, and the pose being matched
// is now shown by the real G1 on the console's stage (TeleopStage).

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopPostureGuide : MonoBehaviour
    {
        public TeleopSession Session;

        [Header("Controller markers")]
        public float MarkerRadius = 0.045f;

        // Brighter and fully opaque, was (0.75, 0.92, 1.00, 0.90): against a
        // flat fallback background the pale, translucent version stood out
        // fine, but the background is now a live camera feed and a pale
        // marker can land on a same-toned patch of the real room and nearly
        // vanish. See Outline below for the other half of the fix.
        private static readonly Color Marker = new Color(0.35f, 0.85f, 1.00f, 1f);

        /// <summary>Dark outline behind each marker, same reasoning as
        /// TeleopAlignGuide's ring outline.</summary>
        private static readonly Color Outline = new Color(0.02f, 0.02f, 0.03f, 0.9f);
        private const float OutlineDepthOffset = 0.004f;

        private Material _material;
        private LineRenderer _leftMarker, _rightMarker;
        private LineRenderer _leftRay, _rightRay;
        private LineRenderer _leftMarkerOutline, _rightMarkerOutline;
        private LineRenderer _leftRayOutline, _rightRayOutline;

        private const int MarkerSegments = 20;

        private void Start()
        {
            var shader = Shader.Find("Sprites/Default") ?? Shader.Find("UI/Default");
            if (shader == null)
            {
                Debug.LogError("[Teleop] no unlit shader; the posture guide cannot draw.");
                enabled = false;
                return;
            }
            _material = new Material(shader);

            _leftMarkerOutline = Line("CtrlOutlineL", MarkerSegments + 1, 0.011f, true);
            _rightMarkerOutline = Line("CtrlOutlineR", MarkerSegments + 1, 0.011f, true);
            _leftRayOutline = Line("CtrlRayOutlineL", 2, 0.009f, false);
            _rightRayOutline = Line("CtrlRayOutlineR", 2, 0.009f, false);
            _leftMarker = Line("CtrlL", MarkerSegments + 1, 0.005f, true);
            _rightMarker = Line("CtrlR", MarkerSegments + 1, 0.005f, true);
            _leftRay = Line("CtrlRayL", 2, 0.004f, false);
            _rightRay = Line("CtrlRayR", 2, 0.004f, false);
        }

        private LineRenderer Line(string name, int points, float width, bool loop)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.material = _material;
            lr.useWorldSpace = true;
            lr.loop = loop;
            lr.positionCount = points;
            lr.startWidth = lr.endWidth = width;
            lr.numCapVertices = 2;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.enabled = false;
            return lr;
        }

        private void LateUpdate()
        {
            if (Session == null || _material == null) return;

            // Up whenever the session is live: knowing whether the controllers
            // are tracked matters in every state, not only during alignment.
            DrawControllers(Session.LinkConnected);
        }

        // ------------------------------------------------------------------
        private void DrawControllers(bool on)
        {
            _leftMarker.enabled = on; _leftMarkerOutline.enabled = on;
            _rightMarker.enabled = on; _rightMarkerOutline.enabled = on;
            _leftRay.enabled = on; _leftRayOutline.enabled = on;
            _rightRay.enabled = on; _rightRayOutline.enabled = on;
            if (!on) return;

            DrawHand(_leftMarker, _leftMarkerOutline, _leftRay, _leftRayOutline,
                     Session.LeftWristPosition, Session.LeftWristRotation);
            DrawHand(_rightMarker, _rightMarkerOutline, _rightRay, _rightRayOutline,
                     Session.RightWristPosition, Session.RightWristRotation);
        }

        private void DrawHand(LineRenderer ring, LineRenderer ringOutline,
                              LineRenderer ray, LineRenderer rayOutline,
                              Vector3 pos, Quaternion rot)
        {
            // A ring in the controller's own plane plus a short forward stub.
            // The stub is what makes the roll visible; a bare dot cannot show
            // that a controller is upside down, and an upside-down controller
            // is a real way to fail the rotation half of the gate. Each has a
            // dark outline set back along -fwd (away from the operator, who
            // is generally looking down the ray toward the controller) for
            // contrast against the live camera background -- see Outline.
            var fwd = rot * Vector3.forward;
            var right = rot * Vector3.right;
            var up = rot * Vector3.up;
            var behind = pos - fwd * OutlineDepthOffset;

            ring.positionCount = MarkerSegments + 1;
            ringOutline.positionCount = MarkerSegments + 1;
            for (var i = 0; i <= MarkerSegments; i++)
            {
                var th = (float)i / MarkerSegments * Mathf.PI * 2f;
                var offset = (right * Mathf.Cos(th) + up * Mathf.Sin(th)) * MarkerRadius;
                ring.SetPosition(i, pos + offset);
                ringOutline.SetPosition(i, behind + offset);
            }
            Tint(ring, Marker);
            Tint(ringOutline, Outline);

            ray.positionCount = 2;
            ray.SetPosition(0, pos);
            ray.SetPosition(1, pos + fwd * (MarkerRadius * 2.4f));
            Tint(ray, Marker);

            rayOutline.positionCount = 2;
            rayOutline.SetPosition(0, behind);
            rayOutline.SetPosition(1, behind + fwd * (MarkerRadius * 2.4f));
            Tint(rayOutline, Outline);
        }

        private static void Tint(LineRenderer lr, Color c)
        {
            lr.startColor = lr.endColor = c;
        }

        private void OnDestroy()
        {
            if (_material != null) Destroy(_material);
        }
    }
}
