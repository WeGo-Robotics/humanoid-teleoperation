// The console's centre stage.
//
// In the design mock this is a canvas showing the pose being matched. It is
// not decoration and it is not a duplicate of the room: it is the panel the
// robot's camera stream will occupy, and until that stream exists it shows the
// G1 itself, with the wrist targets and the operator's live hands marked
// against it. Both readings answer the same question -- am I where the robot
// needs me? -- so the panel keeps its meaning when the content is swapped.
//
// Swapping is deliberately trivial. This renders a camera into a
// RenderTexture and hands it to TeleopHud as a plain Texture; when the stream
// lands, call TeleopHud.SetStageTexture with the video texture instead and
// nothing else on the console changes.
//
// The model is the real thing: Assets/Resources/G1/G1Robot, baked from
// assets/g1/g1_body29_hand14.urdf by WeGo > Import G1 Model. An earlier
// version drew a stick figure here, which was never shippable and was also
// unnecessary -- the actual robot's geometry is vendored in this repository
// under BSD-3-Clause.
//
// Everything lives on its own layer, parked far from the play area, and only
// the stage camera can see it. That is what keeps a second robot from
// appearing in the middle of the operator's room.

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopStage : MonoBehaviour
    {
        public TeleopSession Session;

        /// <summary>User layer nothing else in this project uses.</summary>
        public const int StageLayer = 30;

        /// <summary>Far enough from any plausible guardian that the model
        /// cannot wander into view even if tracking reports something
        /// absurd.</summary>
        private static readonly Vector3 StageOrigin = new Vector3(0f, 1000f, 0f);

        private const int RtWidth = 640, RtHeight = 584;   // the mock's 4:3.65

        public Texture Output => _rt;

        private RenderTexture _rt;
        private Camera _camera;
        private Material _material;
        private Transform _robot;

        private LineRenderer _leftRing, _rightRing, _leftHand, _rightHand;
        private const int Segments = 28;

        private static readonly Color Good = new Color(0.24f, 0.88f, 0.49f);
        private static readonly Color Warn = new Color(1.00f, 0.79f, 0.25f);
        private static readonly Color Bad = new Color(1.00f, 0.33f, 0.31f);
        private static readonly Color Hand = new Color(0.80f, 0.93f, 1.00f);

        /// <summary>The IK origin's offset from the head, from
        /// teleop/xr/transforms.py. The gate compares the operator's
        /// wrist-minus-head against the robot's wrist minus THIS, so drawing a
        /// target on the robot means adding it back and anchoring at the
        /// pelvis -- which is the URDF root and so the model's own origin.
        /// Anchoring at head_link instead put the rings down by the knees.
        /// Robot axes are x forward, y left, z up; ToUnityOffset maps that to
        /// (-y, z, x), giving (0, 0.45, 0.15) here.</summary>
        private static readonly Vector3 WaistOffset = new Vector3(0f, 0.45f, 0.15f);

        private void Start()
        {
            var shader = Shader.Find("Sprites/Default") ?? Shader.Find("UI/Default");
            if (shader == null)
            {
                Debug.LogError("[Teleop] no unlit shader; the stage cannot draw.");
                enabled = false;
                return;
            }
            _material = new Material(shader);

            _rt = new RenderTexture(RtWidth, RtHeight, 24, RenderTextureFormat.ARGB32)
            {
                name = "TeleopStage",
                antiAliasing = 2,
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
            };
            _rt.Create();

            BuildModel();
            BuildLighting();
            BuildCamera();

            _leftRing = Line("STargetL", 0.014f);
            _rightRing = Line("STargetR", 0.014f);
            _leftHand = Line("SHandL", 0.011f);
            _rightHand = Line("SHandR", 0.011f);
        }

        private void BuildModel()
        {
            var prefab = Resources.Load<GameObject>("G1/G1Robot");
            if (prefab == null)
            {
                Debug.LogError("[Teleop] no Resources/G1/G1Robot. Run " +
                               "WeGo > Import G1 Model; the stage will be empty.");
                return;
            }

            var go = Instantiate(prefab, StageOrigin, Quaternion.identity, transform);
            go.name = "G1";
            _robot = go.transform;
            SetLayer(_robot, StageLayer);

            // No pose is applied. The prefab carries the URDF's own rest
            // angles, which are the robot's natural standing posture and the
            // pose the reference render beside this panel is in. Live joint
            // angles from the host would be strictly better; hand-picked ones
            // are strictly worse, because they show the operator a posture
            // nothing is actually asking for.
        }

        private static void SetLayer(Transform t, int layer)
        {
            t.gameObject.layer = layer;
            for (var i = 0; i < t.childCount; i++) SetLayer(t.GetChild(i), layer);
        }

        /// <summary>The rest of this app is unlit, so the stage brings its own
        /// light and confines it to the stage layer. Without that the baked
        /// Standard-shader meshes render as flat silhouettes and the robot is
        /// unreadable.</summary>
        private void BuildLighting()
        {
            var keyGo = new GameObject("StageKey") { layer = StageLayer };
            keyGo.transform.SetParent(transform, false);
            var key = keyGo.AddComponent<Light>();
            key.type = LightType.Directional;
            key.color = new Color(0.85f, 1.00f, 0.92f);
            key.intensity = 1.15f;
            key.cullingMask = 1 << StageLayer;
            key.shadows = LightShadows.None;
            keyGo.transform.rotation = Quaternion.Euler(38f, 152f, 0f);

            var fillGo = new GameObject("StageFill") { layer = StageLayer };
            fillGo.transform.SetParent(transform, false);
            var fill = fillGo.AddComponent<Light>();
            fill.type = LightType.Directional;
            fill.color = new Color(0.24f, 0.55f, 0.40f);
            fill.intensity = 0.55f;
            fill.cullingMask = 1 << StageLayer;
            fill.shadows = LightShadows.None;
            fillGo.transform.rotation = Quaternion.Euler(12f, -40f, 0f);
        }

        private void BuildCamera()
        {
            var camGo = new GameObject("StageCamera");
            camGo.transform.SetParent(transform, false);
            _camera = camGo.AddComponent<Camera>();
            _camera.cullingMask = 1 << StageLayer;
            _camera.clearFlags = CameraClearFlags.SolidColor;
            _camera.backgroundColor = new Color(0.016f, 0.055f, 0.035f, 1f);
            _camera.targetTexture = _rt;
            _camera.fieldOfView = 34f;
            _camera.nearClipPlane = 0.05f;
            _camera.farClipPlane = 12f;
            // Never let this camera participate in stereo rendering; it is a
            // monoscopic offscreen render and XR would try to give it two eyes.
            _camera.stereoTargetEye = StereoTargetEyeMask.None;
            _camera.depth = -10;

            // Square on, not three-quarter. The robot is a reference to copy,
            // and any yaw offset reads as the model being tilted rather than as
            // the camera being placed at an angle. The conversion puts ROS +x
            // on Unity +z, so the robot faces +z and this is dead centre.
            var focus = StageOrigin + new Vector3(0f, 0.02f, 0f);
            camGo.transform.position = StageOrigin + new Vector3(0f, 0.10f, 2.15f);
            camGo.transform.LookAt(focus, Vector3.up);
        }

        private LineRenderer Line(string name, float width)
        {
            var go = new GameObject(name) { layer = StageLayer };
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.material = _material;
            lr.useWorldSpace = true;
            lr.loop = true;
            lr.positionCount = Segments + 1;
            lr.startWidth = lr.endWidth = width;
            lr.numCapVertices = 2;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.enabled = false;
            return lr;
        }

        // ------------------------------------------------------------------
        private void LateUpdate()
        {
            if (Session == null || _camera == null) return;

            var headPos = Session.HeadPosition;
            var anchor = StageOrigin + WaistOffset;

            // Overlays are head-relative offsets replayed from the robot's IK
            // origin, with the operator's yaw removed so the markers do not
            // swing around the model when the operator turns on the spot. This
            // is the same relationship the gate tests: wrist minus head.
            var unyaw = Quaternion.Inverse(
                Quaternion.Euler(0f, Session.HeadRotation.eulerAngles.y, 0f));
            Vector3 Map(Vector3 world) => anchor + unyaw * (world - headPos);

            var haveTargets = Session.HasAlignTargets;
            _leftRing.enabled = haveTargets;
            _rightRing.enabled = haveTargets;
            if (haveTargets)
            {
                // Radius and verdict both from the host, same as the rings in
                // the room: the panel and the room must not disagree about how
                // big the target is or whether a hand is in it.
                Ring(_leftRing, Map(Session.LeftAlignTarget),
                     RadiusOr(Session.LeftRingRadius),
                     Session.LeftInPosition ? Good : ColourFor(Session.LeftPosError));
                Ring(_rightRing, Map(Session.RightAlignTarget),
                     RadiusOr(Session.RightRingRadius),
                     Session.RightInPosition ? Good : ColourFor(Session.RightPosError));
            }

            Ring(_leftHand, Map(Session.LeftWristPosition), 0.045f, Hand);
            Ring(_rightHand, Map(Session.RightWristPosition), 0.045f, Hand);
        }

        /// <summary>The host's radius once it has sent one. The fallback is
        /// only ever seen in the frames before the first align report.</summary>
        private static float RadiusOr(float hostRadius)
        {
            return hostRadius > 1e-4f ? hostRadius : 0.10f;
        }

        private static Color ColourFor(float err)
        {
            if (float.IsNaN(err) || float.IsInfinity(err)) return Bad;
            return err <= 0.10f ? Good : (err <= 0.35f ? Warn : Bad);
        }

        /// <summary>Billboarded to the stage camera. A ring drawn in a fixed
        /// plane goes edge-on as soon as the camera is level with it and reads
        /// as a stray line rather than a circle.</summary>
        private void Ring(LineRenderer lr, Vector3 centre, float r, Color colour)
        {
            var normal = (centre - _camera.transform.position).normalized;
            if (normal.sqrMagnitude < 1e-6f) normal = Vector3.forward;
            var up = Mathf.Abs(Vector3.Dot(normal, Vector3.up)) > 0.95f
                   ? Vector3.forward : Vector3.up;
            var a = Vector3.Normalize(Vector3.Cross(up, normal));
            var b = Vector3.Normalize(Vector3.Cross(normal, a));

            lr.enabled = true;
            lr.positionCount = Segments + 1;
            for (var i = 0; i <= Segments; i++)
            {
                var th = (float)i / Segments * Mathf.PI * 2f;
                lr.SetPosition(i, centre + (a * Mathf.Cos(th) + b * Mathf.Sin(th)) * r);
            }
            lr.startColor = lr.endColor = colour;
        }

        private void OnDestroy()
        {
            if (_material != null) Destroy(_material);
            if (_rt != null) { _rt.Release(); Destroy(_rt); }
        }
    }
}
