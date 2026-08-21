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

        private static readonly Color Marker = new Color(0.75f, 0.92f, 1.00f, 0.90f);

        private Material _material;
        private LineRenderer _leftMarker, _rightMarker;
        private LineRenderer _leftRay, _rightRay;

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
            _leftMarker.enabled = on;
            _rightMarker.enabled = on;
            _leftRay.enabled = on;
            _rightRay.enabled = on;
            if (!on) return;

            DrawHand(_leftMarker, _leftRay, Session.LeftWristPosition,
                     Session.LeftWristRotation);
            DrawHand(_rightMarker, _rightRay, Session.RightWristPosition,
                     Session.RightWristRotation);
        }

        private void DrawHand(LineRenderer ring, LineRenderer ray,
                              Vector3 pos, Quaternion rot)
        {
            // A ring in the controller's own plane plus a short forward stub.
            // The stub is what makes the roll visible; a bare dot cannot show
            // that a controller is upside down, and an upside-down controller
            // is a real way to fail the rotation half of the gate.
            var fwd = rot * Vector3.forward;
            var right = rot * Vector3.right;
            var up = rot * Vector3.up;

            ring.positionCount = MarkerSegments + 1;
            for (var i = 0; i <= MarkerSegments; i++)
            {
                var th = (float)i / MarkerSegments * Mathf.PI * 2f;
                ring.SetPosition(i, pos + (right * Mathf.Cos(th) + up * Mathf.Sin(th))
                                          * MarkerRadius);
            }
            Tint(ring, Marker);

            ray.positionCount = 2;
            ray.SetPosition(0, pos);
            ray.SetPosition(1, pos + fwd * (MarkerRadius * 2.4f));
            Tint(ray, Marker);
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
