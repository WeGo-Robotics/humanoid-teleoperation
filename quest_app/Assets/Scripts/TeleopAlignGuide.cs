// The spatial half of the alignment guide.
//
// TeleopHud is a billboard: it follows the operator's yaw so it can never leave
// view, which is right for text and wrong for a target. A target has to be a
// place in the room -- something you reach toward, that stays where it is when
// you turn away, and that tells you which way to turn to find it again. That is
// the entire reason this runs in a headset rather than on a monitor, and it is
// what docs 14.7 found missing: the operator was being asked to match a pose
// they had no way to see.
//
// What this draws, all from data the host already computes:
//
//   * a ring at each wrist target, coloured by how close that hand is
//   * a dashed line from the operator's hand to its ring while out of tolerance
//   * an arrow at the edge of view when a ring is behind or beside the operator
//
// The rings are anchored head-relative, not world-fixed -- see the comment on
// TeleopSession.ApplyAlignTargets for why that is the correct anchoring and not
// a shortcut.
//
// Rendering is LineRenderer rather than UI: these are objects in the room with
// real depth, and a world-space Canvas would sort against passthrough badly.

using System;
using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopAlignGuide : MonoBehaviour
    {
        public TeleopSession Session;
        public Transform HeadAnchor;

        [Header("Ring")]
        [Tooltip("Fallback radius in metres, used only until the host's first " +
                 "align report arrives. The real radius comes down the wire " +
                 "per side, because it is the gate's angular tolerance drawn " +
                 "at the marker's distance and only the host knows both.")]
        public float RingRadius = 0.10f;
        public int RingSegments = 48;
        public float RingWidth = 0.006f;

        [Header("Colour")]
        [Tooltip("Distance at which the ring reads fully red. Matches " +
                 "AlignConfig.guidance_range_m so the colour and the host's " +
                 "percentage agree.")]
        public float GuidanceRange = 0.75f;

        private static readonly Color Good = new Color(0.24f, 0.88f, 0.49f);
        private static readonly Color Warn = new Color(1.00f, 0.79f, 0.25f);
        private static readonly Color Bad = new Color(1.00f, 0.33f, 0.31f);

        private LineRenderer _leftRing, _rightRing, _leftLead, _rightLead;
        private LineRenderer _leftArrow, _rightArrow;
        private Material _material;

        private void Start()
        {
            // Sprites/Default is in Unity's always-included set; UI/Default is
            // pulled in by TeleopHud's Text/RawImage. Falling back through both
            // means a stripped shader shows up as a warning at startup instead
            // of invisible geometry that looks like a tracking failure.
            var shader = Shader.Find("Sprites/Default") ?? Shader.Find("UI/Default");
            if (shader == null)
            {
                Debug.LogError("[Teleop] no unlit shader available; the align " +
                               "guide cannot draw. Add Sprites/Default to " +
                               "Always Included Shaders.");
                enabled = false;
                return;
            }
            _material = new Material(shader);

            _leftRing = MakeLine("AlignRingL", RingSegments + 1, RingWidth, true);
            _rightRing = MakeLine("AlignRingR", RingSegments + 1, RingWidth, true);
            _leftLead = MakeLine("AlignLeadL", 2, RingWidth * 0.6f, false);
            _rightLead = MakeLine("AlignLeadR", 2, RingWidth * 0.6f, false);
            _leftArrow = MakeLine("AlignArrowL", 4, RingWidth * 0.8f, true);
            _rightArrow = MakeLine("AlignArrowR", 4, RingWidth * 0.8f, true);
        }

        private LineRenderer MakeLine(string name, int points, float width, bool loop)
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

            // Only during alignment. A ring left on screen while following
            // would read as "there is still something to line up with", which
            // is exactly the wrong thing to tell someone driving a robot.
            var aligning = Session.SessionState == "ALIGN" && Session.HasAlignTargets;
            if (!aligning)
            {
                SetAll(false);
                return;
            }

            DrawSide(Session.LeftAlignTarget, Session.LeftWristPosition, Session.LeftPosError,
                     Session.LeftInPosition, RadiusOr(Session.LeftRingRadius),
                     _leftRing, _leftLead, _leftArrow);
            DrawSide(Session.RightAlignTarget, Session.RightWristPosition, Session.RightPosError,
                     Session.RightInPosition, RadiusOr(Session.RightRingRadius),
                     _rightRing, _rightLead, _rightArrow);
        }

        /// <summary>Head pose from the session, for the same reason TeleopHud
        /// takes it from there: OVRCameraRig's anchors do not move on this
        /// headset, so anything derived from the camera transform -- ring
        /// facing, the in-view test, the edge chevrons -- was being computed
        /// against identity.</summary>
        private Vector3 HeadPos => Session != null ? Session.HeadPosition
                                 : (HeadAnchor != null ? HeadAnchor.position : Vector3.zero);

        private Quaternion HeadRot => Session != null ? Session.HeadRotation
                                    : (HeadAnchor != null ? HeadAnchor.rotation : Quaternion.identity);

        /// <summary>The host's radius when it has sent one, the inspector
        /// default until then.</summary>
        private float RadiusOr(float hostRadius)
        {
            return hostRadius > 1e-4f ? hostRadius : RingRadius;
        }

        private void DrawSide(Vector3 target, Vector3 wrist, float hostErr,
                              bool inPosition, float radius,
                              LineRenderer ring, LineRenderer lead, LineRenderer arrow)
        {
            // Distance is measured locally rather than taken from the host's
            // left_pos_err: the host's figure is one control cycle old and
            // arrives at message rate, and a ring whose colour lags the hand by
            // 100 ms feels broken to reach toward. The host's value still wins
            // for anything that gates -- this only drives colour.
            var err = Vector3.Distance(wrist, target);
            if (float.IsNaN(err)) err = float.IsNaN(hostErr) ? 0f : hostErr;

            // Green means the HOST says this hand counts, never a threshold of
            // ours. The gradient below green is local, because "getting
            // warmer" has to track the hand at frame rate and a colour that
            // lags by a message feels broken to reach toward -- but the one
            // colour that means "this is done" is the host's to give.
            var colour = inPosition ? Good : ColourFor(err, radius);

            var visible = IsInView(target);
            ring.enabled = visible;
            arrow.enabled = !visible;

            if (visible)
            {
                DrawRing(ring, target, colour, radius);
                var far = err > radius;
                lead.enabled = far;
                if (far)
                {
                    lead.positionCount = 2;
                    lead.SetPosition(0, wrist);
                    lead.SetPosition(1, target);
                    Tint(lead, colour, 0.55f);
                }
            }
            else
            {
                lead.enabled = false;
                DrawEdgeArrow(arrow, target);
            }
        }

        private Color ColourFor(float err, float radius)
        {
            if (err <= radius) return Good;
            var t = Mathf.Clamp01((err - radius) / Mathf.Max(GuidanceRange - radius, 1e-4f));
            return t < 0.5f ? Color.Lerp(Good, Warn, t * 2f)
                            : Color.Lerp(Warn, Bad, (t - 0.5f) * 2f);
        }

        /// <summary>Ring drawn facing the operator, so it reads as a hoop to put
        /// a hand through rather than an ellipse that happens to be edge-on.</summary>
        private void DrawRing(LineRenderer lr, Vector3 centre, Color colour, float radius)
        {
            var normal = (centre - HeadPos).normalized;
            if (normal.sqrMagnitude < 1e-6f) normal = Vector3.forward;
            var up = Mathf.Abs(Vector3.Dot(normal, Vector3.up)) > 0.95f ? Vector3.forward : Vector3.up;
            var a = Vector3.Normalize(Vector3.Cross(up, normal)) * radius;
            var b = Vector3.Normalize(Vector3.Cross(normal, a)) * radius;

            lr.positionCount = RingSegments + 1;
            for (var i = 0; i <= RingSegments; i++)
            {
                var th = (float)i / RingSegments * Mathf.PI * 2f;
                lr.SetPosition(i, centre + a * Mathf.Cos(th) + b * Mathf.Sin(th));
            }
            Tint(lr, colour, 1f);
        }

        /// <summary>A chevron pinned inside the view edge, pointing the shortest
        /// way to turn. Standard off-screen point-of-interest treatment: a
        /// world-anchored target is *supposed* to leave view, so it needs a way
        /// to say where it went.</summary>
        private void DrawEdgeArrow(LineRenderer lr, Vector3 target)
        {
            var headPos = HeadPos;
            var headRot = HeadRot;
            var fwd = headRot * Vector3.forward;
            var right = headRot * Vector3.right;
            var up = headRot * Vector3.up;
            var local = Quaternion.Inverse(headRot) * (target - headPos);

            // Behind the operator, the projected point mirrors; flipping keeps
            // the chevron pointing the short way round instead of the long way.
            var dir = new Vector2(local.x, local.y);
            if (local.z <= 0f) dir = new Vector2(-local.x, local.y);
            if (dir.sqrMagnitude < 1e-6f) dir = Vector2.right;
            dir.Normalize();

            // Pushed out to 32 degrees off-axis. The console spans about 30
            // degrees at the distance TeleopHud parks it, and at the previous
            // 19 degrees these chevrons were drawn straight across the middle
            // of it -- two red arrows over the readouts they were supposed to
            // be sending the operator to.
            const float dist = 1.0f, extent = 0.62f, size = 0.05f;
            var centre = headPos + fwd * dist
                       + right * dir.x * extent + up * dir.y * extent;
            var tip = right * dir.x + up * dir.y;
            var side = Vector3.Cross(tip, fwd).normalized;

            lr.positionCount = 4;
            lr.SetPosition(0, centre + tip * size);
            lr.SetPosition(1, centre - tip * size * 0.5f + side * size * 0.7f);
            lr.SetPosition(2, centre - tip * size * 0.5f - side * size * 0.7f);
            lr.SetPosition(3, centre + tip * size);
            Tint(lr, Bad, 1f);
        }

        /// <summary>Angle-based rather than WorldToViewportPoint, because the
        /// camera's transform does not track on this headset and the viewport
        /// test was therefore being run against a camera sitting at the origin
        /// facing +Z. Half-angles are a Quest 3's usable field of view with a
        /// margin, so a target near the edge switches to a chevron slightly
        /// before it actually leaves view.</summary>
        private const float HalfFovH = 44f, HalfFovV = 38f;

        private bool IsInView(Vector3 world)
        {
            var local = Quaternion.Inverse(HeadRot) * (world - HeadPos);
            if (local.z <= 0.01f) return false;
            return Mathf.Abs(Mathf.Atan2(local.x, local.z) * Mathf.Rad2Deg) < HalfFovH
                && Mathf.Abs(Mathf.Atan2(local.y, local.z) * Mathf.Rad2Deg) < HalfFovV;
        }

        private static void Tint(LineRenderer lr, Color c, float alpha)
        {
            c.a = alpha;
            lr.startColor = lr.endColor = c;
        }

        private void SetAll(bool on)
        {
            if (_leftRing == null) return;
            _leftRing.enabled = on; _rightRing.enabled = on;
            _leftLead.enabled = on; _rightLead.enabled = on;
            _leftArrow.enabled = on; _rightArrow.enabled = on;
        }

        private void OnDestroy()
        {
            if (_material != null) Destroy(_material);
        }
    }
}
