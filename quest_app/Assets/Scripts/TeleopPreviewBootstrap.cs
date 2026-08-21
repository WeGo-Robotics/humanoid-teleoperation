// Scene builder for the desktop preview.
//
// The device path is TeleopBootstrap: wait for XR, build an OVRCameraRig, read
// the headset. None of that exists here. What is deliberately identical is
// everything after the rig -- the same TeleopSession, the same TeleopHud, the
// same TeleopAlignGuide, wired the same way. Only the source of the poses and
// the source of the align state differ, and both of those are behind
// interfaces the display code never touches.
//
// The room is a plain grid rather than a skybox. Passthrough on device shows a
// real room; a flat colour here would make the HUD's contrast look better than
// it is, and the depth cues in a grid are what make it obvious whether a ring
// is actually sitting where a hand can reach it.

#if UNITY_STANDALONE || UNITY_EDITOR

using UnityEngine;

namespace WeGo.Teleop
{
    public class TeleopPreviewBootstrap : MonoBehaviour
    {
        private void Start()
        {
            var camGo = new GameObject("PreviewHead");
            var cam = camGo.AddComponent<Camera>();
            cam.tag = "MainCamera";
            cam.nearClipPlane = 0.02f;
            cam.farClipPlane = 60f;
            // Chosen so the HORIZONTAL field of view on a 16:9 window matches a
            // Quest 3's ~100 degrees, because horizontal is what decides
            // whether the console fits. Setting the vertical FOV to the
            // headset's own figure instead gives 114 degrees across on this
            // aspect, which makes everything look smaller in the preview than
            // it does on device -- the one error a preview must not have.
            cam.fieldOfView = 68f;
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.05f, 0.06f, 0.08f, 1f);

            BuildRoom();

            // Built on an inactive child, configured, and only then switched
            // on. AddComponent runs OnEnable synchronously, so a component
            // added to a live object has already opened its socket and built
            // its UI by the time the next line assigns its fields -- which is
            // how the first preview ended up connecting to a real host.
            var host = new GameObject("TeleopRuntime");
            host.SetActive(false);
            host.transform.SetParent(transform, false);

            var session = host.AddComponent<TeleopSession>();
            session.Offline = true;

            var driver = host.AddComponent<TeleopPreviewDriver>();
            driver.Session = session;
            driver.Head = cam;

            var stage = host.AddComponent<TeleopStage>();
            stage.Session = session;

            var hud = host.AddComponent<TeleopHud>();
            hud.Session = session;
            hud.Stage = stage;
            hud.Anchor = cam.transform;

            cam.cullingMask &= ~(1 << TeleopStage.StageLayer);

            var guide = host.AddComponent<TeleopAlignGuide>();
            guide.Session = session;
            guide.HeadAnchor = cam.transform;

            var posture = host.AddComponent<TeleopPostureGuide>();
            posture.Session = session;

            host.SetActive(true);

            Debug.Log("[Teleop] preview scene built");
        }

        /// <summary>A 10x10 m floor grid on 0.5 m spacing, drawn with the same
        /// LineRenderer/Sprites-Default path the align guide uses. If the
        /// shader ever gets stripped, the floor disappears too -- which turns a
        /// silent rendering failure into an obvious one.</summary>
        private static void BuildRoom()
        {
            var shader = Shader.Find("Sprites/Default") ?? Shader.Find("UI/Default");
            if (shader == null) return;
            var mat = new Material(shader);

            const float half = 5f, step = 0.5f;
            var root = new GameObject("Floor").transform;
            for (var i = -half; i <= half + 1e-3f; i += step)
            {
                var major = Mathf.Abs(i % 1f) < 1e-3f;
                var c = major ? new Color(1f, 1f, 1f, 0.13f) : new Color(1f, 1f, 1f, 0.06f);
                Line(root, mat, c, new Vector3(i, 0f, -half), new Vector3(i, 0f, half));
                Line(root, mat, c, new Vector3(-half, 0f, i), new Vector3(half, 0f, i));
            }
        }

        private static void Line(Transform parent, Material mat, Color colour,
                                 Vector3 a, Vector3 b)
        {
            var go = new GameObject("GridLine");
            go.transform.SetParent(parent, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.material = mat;
            lr.useWorldSpace = true;
            lr.positionCount = 2;
            lr.startWidth = lr.endWidth = 0.004f;
            lr.startColor = lr.endColor = colour;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows = false;
            lr.SetPosition(0, a);
            lr.SetPosition(1, b);
        }
    }
}

#endif
