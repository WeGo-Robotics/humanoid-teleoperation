// The posture figure and the controller markers.
//
// Two things the console cannot do, because both have to exist in the room
// rather than on a panel:
//
//   * a wireframe figure standing in front of the operator, posed at the pose
//     the host is asking for. The design mock showed this as a skeleton in a
//     viewport; in a headset it belongs in the room at the operator's own
//     scale, facing the same way they are, so that copying it is a matter of
//     imitation rather than of mentally un-mirroring a diagram.
//   * a marker on each controller, so the operator can see where the app
//     thinks their hands are. When alignment will not close, the first
//     question is always whether tracking is live, and until now nothing on
//     screen could answer it.
//
// Why the markers are drawn from InputTracking rather than from Meta's
// OVRControllerPrefab / OVRControllerHelper: those resolve the controller
// through OVRInput, which resolves through OVRPlugin.GetControllerState6, and
// that call was measured returning empty ConnectedControllers and Buttons on
// this headset while the node-tracking pipe kept working (docs 14). Rendering
// the shipped controller model would therefore have drawn nothing, or drawn it
// at the origin. The node pipe is the one that works here.
//
// The figure is scaled to the OPERATOR, not to the G1. The gate subtracts the
// head from the wrists and compares against a fixed offset, so what is being
// matched is a posture and not a stature; a figure drawn at the robot's 1.32 m
// would tell the operator to crouch.

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopPostureGuide : MonoBehaviour
    {
        public TeleopSession Session;

        [Header("Figure")]
        [Tooltip("Metres in front of the operator. Set so the figure stands " +
                 "behind the console rather than inside it: from here the head, " +
                 "shoulders and both arms clear the console's top edge, and " +
                 "only the legs are hidden behind it.")]
        public float Distance = 2.6f;
        public float BoneWidth = 0.012f;

        [Header("Controller markers")]
        public float MarkerRadius = 0.045f;

        private static readonly Color BoneColour = new Color(0.24f, 0.88f, 0.49f, 0.85f);
        private static readonly Color Joint = new Color(0.55f, 1.00f, 0.74f, 0.95f);
        private static readonly Color Marker = new Color(0.75f, 0.92f, 1.00f, 0.90f);

        private Material _material;
        private LineRenderer[] _bones;
        private LineRenderer _leftMarker, _rightMarker;
        private LineRenderer _leftRay, _rightRay;

        // Skeleton, as offsets from the head in metres at a 1.75 m reference
        // height, scaled at runtime by the operator's measured head height.
        // Arms are not in here -- they come from the align targets, which is
        // the whole point of the figure.
        private const float RefHeight = 1.75f;
        private static readonly Vector3 Neck = new Vector3(0f, -0.17f, 0f);
        private static readonly Vector3 ShoulderC = new Vector3(0f, -0.23f, 0f);
        private static readonly Vector3 ShoulderL = new Vector3(-0.19f, -0.23f, 0f);
        private static readonly Vector3 ShoulderR = new Vector3(0.19f, -0.23f, 0f);
        private static readonly Vector3 HipC = new Vector3(0f, -0.68f, 0f);
        private static readonly Vector3 HipL = new Vector3(-0.10f, -0.70f, 0f);
        private static readonly Vector3 HipR = new Vector3(0.10f, -0.70f, 0f);
        private static readonly Vector3 KneeL = new Vector3(-0.11f, -1.11f, 0.02f);
        private static readonly Vector3 KneeR = new Vector3(0.11f, -1.11f, 0.02f);
        private static readonly Vector3 AnkleL = new Vector3(-0.11f, -1.51f, 0f);
        private static readonly Vector3 AnkleR = new Vector3(0.11f, -1.51f, 0f);

        private const int BoneCount = 16;
        private const int HeadRingSegments = 20;

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

            _bones = new LineRenderer[BoneCount];
            for (var i = 0; i < BoneCount - 1; i++)
                _bones[i] = Line($"Bone{i}", 2, BoneWidth, false);

            // Head is a ring rather than a bone, so the figure reads as facing
            // away rather than as an ambiguous stick.
            _bones[BoneCount - 1] = Line("Head", HeadRingSegments + 1, BoneWidth, true);

            _leftMarker = Line("CtrlL", HeadRingSegments + 1, 0.005f, true);
            _rightMarker = Line("CtrlR", HeadRingSegments + 1, 0.005f, true);
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

            // Markers stay up whenever the session is live: knowing whether the
            // controllers are tracked matters in every state, not only during
            // alignment.
            DrawControllers(Session.LinkConnected);

            var aligning = Session.SessionState == "ALIGN";
            if (!aligning)
            {
                foreach (var b in _bones) b.enabled = false;
                return;
            }
            DrawFigure();
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

            ring.positionCount = HeadRingSegments + 1;
            for (var i = 0; i <= HeadRingSegments; i++)
            {
                var th = (float)i / HeadRingSegments * Mathf.PI * 2f;
                ring.SetPosition(i, pos + (right * Mathf.Cos(th) + up * Mathf.Sin(th))
                                          * MarkerRadius);
            }
            Tint(ring, Marker);

            ray.positionCount = 2;
            ray.SetPosition(0, pos);
            ray.SetPosition(1, pos + fwd * (MarkerRadius * 2.4f));
            Tint(ray, Marker);
        }

        // ------------------------------------------------------------------
        private void DrawFigure()
        {
            var headPos = Session.HeadPosition;

            // Yaw only: the figure stands upright however the operator tilts
            // their head, and turns to stay in front of them.
            var yaw = Quaternion.Euler(0f, Session.HeadRotation.eulerAngles.y, 0f);
            var fwd = yaw * Vector3.forward;

            // Scaled by the operator's own head height above the floor. Falls
            // back to the reference if tracking has not produced a height yet,
            // rather than collapsing the figure to a point.
            var h = headPos.y > 0.8f ? headPos.y : RefHeight;
            var s = h / RefHeight;

            // Origin at the figure's head, standing on the floor in front,
            // facing the same way the operator faces -- so the operator is
            // looking at its back and can copy it directly. Facing them would
            // mirror every limb.
            var origin = new Vector3(headPos.x, h, headPos.z) + fwd * Distance;

            Vector3 P(Vector3 local) => origin + yaw * (local * s);

            var shoulderL = P(ShoulderL);
            var shoulderR = P(ShoulderR);

            // Arms are posed from the align targets when the host has sent
            // them. Without targets the figure would be inventing a pose, so
            // the arms are simply left out -- see DrawArm.
            var haveTargets = Session.HasAlignTargets;
            var wristL = Session.LeftAlignTarget;
            var wristR = Session.RightAlignTarget;

            // Target wrists are head-relative to the OPERATOR; re-anchor them
            // onto the figure so the figure shows the same relationship.
            // The figure shares the operator's yaw, so the world-space offset
            // from head to wrist carries over directly; only the scale changes.
            if (haveTargets)
            {
                wristL = origin + (wristL - headPos) * s;
                wristR = origin + (wristR - headPos) * s;
            }

            var i = 0;
            Bone(i++, P(Neck), P(ShoulderC), BoneColour);
            Bone(i++, shoulderL, shoulderR, BoneColour);
            Bone(i++, P(ShoulderC), P(HipC), BoneColour);
            Bone(i++, P(HipL), P(HipR), BoneColour);
            Bone(i++, P(HipC), P(HipL), BoneColour);
            Bone(i++, P(HipC), P(HipR), BoneColour);
            Bone(i++, P(HipL), P(KneeL), BoneColour);
            Bone(i++, P(HipR), P(KneeR), BoneColour);
            Bone(i++, P(KneeL), P(AnkleL), BoneColour);
            Bone(i++, P(KneeR), P(AnkleR), BoneColour);

            i = DrawArm(i, shoulderL, wristL, -1f, haveTargets);
            i = DrawArm(i, shoulderR, wristR, 1f, haveTargets);

            while (i < BoneCount - 1) _bones[i++].enabled = false;

            DrawHeadRing(_bones[BoneCount - 1], origin, yaw, 0.10f * s);
        }

        /// <summary>Shoulder to elbow to wrist, with the elbow placed by a
        /// fixed outward-and-down bend rather than solved. Two segments and a
        /// plausible elbow read as an arm; a straight line from shoulder to
        /// wrist reads as a stick and tells the operator to lock their elbow.
        /// Nothing downstream uses the elbow, so an approximation is honest
        /// here in a way it would not be for the wrist.</summary>
        private int DrawArm(int i, Vector3 shoulder, Vector3 wrist, float side,
                            bool haveTargets)
        {
            if (!haveTargets)
            {
                _bones[i++].enabled = false;
                _bones[i++].enabled = false;
                return i;
            }

            var mid = (shoulder + wrist) * 0.5f;
            var span = wrist - shoulder;
            var outward = Vector3.Cross(span.normalized, Vector3.up).normalized * side;
            var elbow = mid + outward * 0.06f + Vector3.down * 0.05f;

            Bone(i++, shoulder, elbow, BoneColour);
            Bone(i++, elbow, wrist, BoneColour);
            return i;
        }

        private void Bone(int i, Vector3 a, Vector3 b, Color c)
        {
            var lr = _bones[i];
            lr.enabled = true;
            lr.loop = false;
            lr.positionCount = 2;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
            Tint(lr, c);
        }

        private void DrawHeadRing(LineRenderer lr, Vector3 centre, Quaternion yaw, float r)
        {
            lr.enabled = true;
            lr.loop = true;
            lr.positionCount = HeadRingSegments + 1;
            var right = yaw * Vector3.right;
            var fwd = yaw * Vector3.forward;
            for (var i = 0; i <= HeadRingSegments; i++)
            {
                var th = (float)i / HeadRingSegments * Mathf.PI * 2f;
                lr.SetPosition(i, centre + (right * Mathf.Cos(th) + fwd * Mathf.Sin(th)) * r);
            }
            Tint(lr, Joint);
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
