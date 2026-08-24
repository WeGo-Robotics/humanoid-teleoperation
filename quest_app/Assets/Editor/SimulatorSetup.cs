// Running the real device path on the PC, under Meta XR Simulator.
//
// The problem this solves is not "we have no Quest". It is that verifying on a
// Quest costs a build, an adb dance, a headset on the head, and a logcat window
// -- per attempt -- and the defects in docs section 16 were each a one-line
// disagreement that took a whole session to find. tools/fake_quest.py cannot
// help with any of them: it speaks the wire protocol directly and never runs a
// line of the app, so every defect that lives on the device is invisible to it.
// Section 16.1 is the clearest case -- the simulator was sending the right
// thing all along and the app was the odd one out.
//
// Meta XR Simulator is an OpenXR runtime that stands in for a headset, so
// pressing Play in the editor runs TeleopBootstrap, OVRManager, OVRCameraRig,
// the UnityEngine.XR input subsystem and OVRPassthroughLayer -- the actual code
// paths, not a mock of them. What that buys, against the section 16 list:
//
//   16.1  the confirm binding, because CommonUsages.trigger is read for real
//   16.2  the stage mirror, because the console renders for real
//   16.3  the align gate, against a host on the same machine
//   16.6  passthrough, against the simulator's synthetic rooms
//   16.7  the ring placement, which needs a head pose at a plausible height
//
// And the topology problem from section 15.3 disappears: the host is on this
// same PC, so it is 127.0.0.1 with no Wi-Fi bridge and no `adb reverse`.
// Debug.Log lands in the editor console rather than in a 256 KiB logcat ring
// that VrApi is busy overwriting.
//
// What it cannot tell you: framerate, real tracking noise, real doff/don
// timing, anything about the APK or the manifest, and whether the robot moves.
// It narrows what has to be checked on hardware; it does not replace it.
//
// Deliberately no compile-time reference to the simulator package. Activation
// is a menu item that package adds (Meta > Meta XR Simulator > Activate) and
// its API surface is not something this project should pin itself to; if the
// package is absent, everything here still compiles and the scene still opens.

using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.XR.Management;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.XR.Management;

namespace WeGo.Teleop.Editor
{
    public static class SimulatorSetup
    {
        private const string ScenePath = "Assets/Scenes/TeleopSimulator.unity";
        private const string OculusLoader = "Unity.XR.Oculus.OculusLoader";

        /// <summary>The host runs on this machine, so the address that used to
        /// need a Wi-Fi bridge onto the robot network is just loopback.</summary>
        private const string DefaultHost = "127.0.0.1";
        private const int DefaultPort = 8443;

        [MenuItem("WeGo/Prepare Simulator Scene")]
        public static void PrepareFromMenu() => Run(DefaultHost, DefaultPort);

        /// <summary>Batchmode entry point; see tools/simulator.ps1.</summary>
        public static void Prepare()
        {
            var host = DefaultHost;
            var port = DefaultPort;
            var args = Environment.GetCommandLineArgs();
            for (var i = 0; i < args.Length - 1; i++)
            {
                if (args[i] == "-host") host = args[i + 1];
                if (args[i] == "-port") port = int.Parse(args[i + 1]);
            }

            try
            {
                Run(host, port);
                if (Application.isBatchMode) EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Debug.LogError($"[SimulatorSetup] {e.GetType().Name}: {e.Message}");
                if (Application.isBatchMode) EditorApplication.Exit(1);
            }
        }

        private static void Run(string host, int port)
        {
            ConfigureGraphics();
            ConfigureXr();
            GenerateScene(host, port);

            Log($"ready. Start the host with `python tools/sim_host.py` (it needs " +
                "no pinocchio, no DDS and no robot), then Meta > Meta XR " +
                $"Simulator > Activate, then press Play. Scene points at {host}:{port}.");
        }

        /// <summary>D3D11 rather than whatever the platform defaults to.
        ///
        /// Meta's own note: passthrough in the simulator needs an extra
        /// configuration parameter set by hand when the app renders with
        /// Vulkan. Nothing in this app needs Vulkan on the desktop, and the
        /// whole point of the exercise is to look at passthrough, so the
        /// cheaper fix is to not ask for Vulkan here. The Android player's
        /// graphics API is untouched -- QuestBuild owns that.</summary>
        private static void ConfigureGraphics()
        {
            PlayerSettings.SetUseDefaultGraphicsAPIs(BuildTarget.StandaloneWindows64, false);
            PlayerSettings.SetGraphicsAPIs(BuildTarget.StandaloneWindows64,
                                           new[] { GraphicsDeviceType.Direct3D11 });
            Log("standalone graphics API pinned to D3D11");
        }

        /// <summary>The same loader QuestBuild assigns for Android, assigned
        /// for Standalone as well. Something has to initialise XR for the
        /// editor to talk to the simulator's runtime at all, and without this
        /// TeleopBootstrap sits in its ten-second wait for an active loader and
        /// then builds the rig anyway with no tracking behind it.</summary>
        private static void ConfigureXr()
        {
            var perTarget = GetOrCreateXrSettings();
            perTarget.CreateDefaultManagerSettingsForBuildTarget(BuildTargetGroup.Standalone);
            var settings = perTarget.SettingsForBuildTarget(BuildTargetGroup.Standalone);
            if (settings == null || settings.Manager == null)
                throw new Exception("XR Management produced no manager for Standalone");

            // Initialise on start, or nothing comes up until something asks.
            settings.InitManagerOnStart = true;

            if (settings.Manager.activeLoaders.Any(
                    l => l != null && l.GetType().FullName == OculusLoader))
            {
                Log("Oculus XR loader already active for Standalone");
                return;
            }

            if (!UnityEditor.XR.Management.Metadata.XRPackageMetadataStore.AssignLoader(
                    settings.Manager, OculusLoader, BuildTargetGroup.Standalone))
                throw new Exception($"could not assign {OculusLoader} for Standalone");

            EditorUtility.SetDirty(settings.Manager);
            EditorUtility.SetDirty(settings);
            AssetDatabase.SaveAssets();
            Log("assigned the Oculus XR loader for Standalone");
        }

        private static XRGeneralSettingsPerBuildTarget GetOrCreateXrSettings()
        {
            if (EditorBuildSettings.TryGetConfigObject(
                    XRGeneralSettings.k_SettingsKey,
                    out XRGeneralSettingsPerBuildTarget existing) && existing != null)
                return existing;

            Directory.CreateDirectory("Assets/XR");
            var created = ScriptableObject.CreateInstance<XRGeneralSettingsPerBuildTarget>();
            AssetDatabase.CreateAsset(created, "Assets/XR/XRGeneralSettings.asset");
            AssetDatabase.SaveAssets();
            EditorBuildSettings.AddConfigObject(XRGeneralSettings.k_SettingsKey,
                                                created, true);
            Log("created XR general settings");
            return created;
        }

        /// <summary>The device scene, not the preview one.
        ///
        /// TeleopPreviewBootstrap exists to run the console without XR, on a
        /// plain camera driven by a mouse. That is the wrong thing here: it
        /// mocks exactly the parts -- OVRManager, the input subsystem,
        /// passthrough -- that the simulator exists to exercise. This scene
        /// carries TeleopBootstrap, the same component the APK runs.</summary>
        private static void GenerateScene(string host, int port)
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,
                                                    NewSceneMode.Single);

            // Meta's guidance for passthrough: no skybox, or it draws behind
            // the virtual content where the real room should be. This app
            // clears to a solid colour so a skybox would not be drawn anyway,
            // but an empty scene inherits the default one and leaving it set
            // is the kind of thing that costs an hour when the room is grey.
            RenderSettings.skybox = null;

            var go = new GameObject("Teleop");
            var boot = go.AddComponent<TeleopBootstrap>();
            boot.HostAddress = host;
            boot.Port = port;
            boot.UseTls = false;
            boot.Passthrough = true;

            Directory.CreateDirectory("Assets/Scenes");
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
                throw new Exception($"could not save {ScenePath}");
            Log($"generated {ScenePath} for {host}:{port}");
        }

        private static void Log(string m) => Debug.Log($"[SimulatorSetup] {m}");
    }
}
