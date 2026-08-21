// Windows preview build.
//
// Same project, same UI scripts, different player: a windowed desktop exe that
// runs TeleopHud and TeleopAlignGuide against a mouse instead of a Quest. The
// point is turnaround. Every UI change previously cost a 90-second APK build,
// an adb install, and someone putting a headset on to find out the panel was
// in the wrong place -- and if it was wrong, all of that again.
//
// This does not replace device testing. Tracking, passthrough, transport,
// controller input and performance can only be judged on hardware. It replaces
// the part of device testing that was only ever asking "does it look right?"
//
//   Unity.exe -quit -batchmode -projectPath quest_app \
//             -executeMethod WeGo.Teleop.Editor.PreviewBuild.Build \
//             -logFile preview.log
//
// tools/build_preview.ps1 wraps this and launches the result.

using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace WeGo.Teleop.Editor
{
    public static class PreviewBuild
    {
        private const string ScenePath = "Assets/Scenes/TeleopPreview.unity";
        private const string DefaultExe = "Build/preview/G1TeleopPreview.exe";

        [MenuItem("WeGo/Build Windows Preview")]
        public static void BuildFromMenu() => Run(DefaultExe);

        public static void Build()
        {
            var output = DefaultExe;
            var args = Environment.GetCommandLineArgs();
            for (var i = 0; i < args.Length - 1; i++)
                if (args[i] == "-output") output = args[i + 1];
            Run(output);
        }

        private static void Run(string output)
        {
            try
            {
                output = Path.GetFullPath(output);
                Directory.CreateDirectory(Path.GetDirectoryName(output) ?? ".");
                Log($"output={output}");

                if (EditorUserBuildSettings.activeBuildTarget != BuildTarget.StandaloneWindows64
                    && !EditorUserBuildSettings.SwitchActiveBuildTarget(
                            BuildTargetGroup.Standalone, BuildTarget.StandaloneWindows64))
                    throw new Exception("could not switch to StandaloneWindows64");

                ConfigurePlayer();
                GenerateScene();

                var report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
                {
                    scenes = new[] { ScenePath },
                    locationPathName = output,
                    target = BuildTarget.StandaloneWindows64,
                    targetGroup = BuildTargetGroup.Standalone,
                    options = BuildOptions.Development,
                });

                if (report.summary.result != BuildResult.Succeeded)
                {
                    Fail($"build {report.summary.result} with " +
                         $"{report.summary.totalErrors} error(s)");
                    return;
                }

                Log($"OK  {output}  {report.summary.totalSize / (1024 * 1024)} MB  " +
                    $"in {report.summary.totalTime.TotalSeconds:F0}s");
                if (Application.isBatchMode) EditorApplication.Exit(0);
            }
            catch (Exception e)
            {
                Fail($"{e.GetType().Name}: {e.Message}\n{e.StackTrace}");
            }
        }

        private static void ConfigurePlayer()
        {
            var standalone = NamedBuildTarget.Standalone;

            PlayerSettings.companyName = "WeGo Robotics";
            PlayerSettings.productName = "G1 Teleop Preview";
            PlayerSettings.SetScriptingBackend(standalone, ScriptingImplementation.Mono2x);

            // Windowed, and never the machine's native resolution: this sits
            // beside an editor and a log tail, not on its own.
            PlayerSettings.defaultIsNativeResolution = false;
            PlayerSettings.defaultScreenWidth = 1600;
            PlayerSettings.defaultScreenHeight = 900;
            PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
            PlayerSettings.resizableWindow = true;
            PlayerSettings.runInBackground = true;

            // Match the device build, or colours in the preview are a lie.
            PlayerSettings.colorSpace = ColorSpace.Linear;

            // Standalone must NOT initialise XR -- with an Oculus loader
            // assigned and no headset attached, the player either stalls at
            // startup or comes up with a black window, which is precisely the
            // failure this build exists to rule out.
            DisableStandaloneXr();

            AssetDatabase.SaveAssets();
        }

        private static void DisableStandaloneXr()
        {
            if (!EditorBuildSettings.TryGetConfigObject(
                    UnityEngine.XR.Management.XRGeneralSettings.k_SettingsKey,
                    out UnityEditor.XR.Management.XRGeneralSettingsPerBuildTarget perTarget)
                || perTarget == null)
                return;

            var settings = perTarget.SettingsForBuildTarget(BuildTargetGroup.Standalone);
            if (settings == null || settings.Manager == null) return;
            settings.InitManagerOnStart = false;
            EditorUtility.SetDirty(settings);
            Log("disabled XR auto-init for Standalone");
        }

        private static void GenerateScene()
        {
            var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,
                                                    NewSceneMode.Single);
            var go = new GameObject("TeleopPreview");
            go.AddComponent<TeleopPreviewBootstrap>();

            Directory.CreateDirectory("Assets/Scenes");
            if (!EditorSceneManager.SaveScene(scene, ScenePath))
                throw new Exception($"could not save {ScenePath}");
            Log($"generated {ScenePath}");
        }

        private static void Log(string m) => Debug.Log($"[PreviewBuild] {m}");

        private static void Fail(string m)
        {
            Debug.LogError($"[PreviewBuild] {m}");
            if (Application.isBatchMode) EditorApplication.Exit(1);
        }
    }
}
