// Bakes a vendored URDF into a Unity prefab.
//
// The stage used to draw a stick figure. That was never shippable, and the
// right asset was already in the repository: assets/g1/g1_body29_hand14.urdf
// with 64 STL meshes, from unitree_ros/g1_description under BSD-3-Clause --
// commercial use permitted, attribution required. It is the actual robot the
// operator is aligning to, which no third-party humanoid could be.
//
// This runs in the Editor, not on device. The STLs are ASCII and total 52 MB;
// parsing that at startup in a headset would cost tens of seconds and the
// text would have to ship inside the APK. Baked to indexed binary meshes it
// is a few MB and loads instantly.
//
//   WeGo > Import G1 Model
//   WeGo > Import R1 Model
//
// Re-run it only when the URDF or the meshes change. The output is committed
// so a fresh clone can build without the import step.
//
// Nothing here is G1-specific -- it reads whatever URDF it is pointed at --
// so a second robot costs one Robot entry and a re-run, not a second importer.
// The app picks between the baked prefabs at runtime from what the host says
// it is driving; see TeleopStage.
//
// Coordinate conversion: URDF is ROS convention -- z up, x forward, right
// handed. Unity is y up, z forward, left handed. Positions map (x,y,z) ->
// (-y,z,x), matching TeleopSession.ToUnityOffset so the model and the wrist
// targets cannot disagree about which way is left. The basis change has
// determinant -1, so triangle winding is reversed on import; skip that and
// every surface renders inside out.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Xml;
using UnityEditor;
using UnityEngine;

namespace WeGo.Teleop.Editor
{
    public static class RobotModelImporter
    {
        /// <summary>One robot the app can display. `Name` is both the
        /// Resources folder and the name the host uses on the wire, so the
        /// runtime lookup is Resources.Load($"{robot}/{robot}Robot") with no
        /// table to keep in step.</summary>
        private sealed class Robot
        {
            public string Name;
            public string UrdfRelative;
        }

        private static readonly Robot G1 = new Robot
        {
            Name = "G1",
            UrdfRelative = "../assets/g1/g1_body29_hand14.urdf",
        };

        private static readonly Robot R1 = new Robot
        {
            Name = "R1",
            UrdfRelative = "../assets/r1/R1.urdf",
        };

        // Set per import so the mesh/material writers below need no extra
        // plumbing. Not re-entrant, which is fine: this is a menu item.
        private static string _outDir, _prefabPath, _meshDir, _tag;

        [MenuItem("WeGo/Import G1 Model")]
        public static void ImportG1() => Import(G1);

        [MenuItem("WeGo/Import R1 Model")]
        public static void ImportR1() => Import(R1);

        /// <summary>Batchmode entry point, for `-executeMethod`.</summary>
        public static void ImportAll()
        {
            Import(G1);
            Import(R1);
        }

        private static void Import(Robot robot)
        {
            _tag = $"{robot.Name}Import";
            _outDir = $"Assets/Resources/{robot.Name}";
            _prefabPath = $"{_outDir}/{robot.Name}Robot.prefab";
            _meshDir = $"{_outDir}/Meshes";
            // Materials are cached by colour against a path under _outDir, so
            // a stale cache would hand the second robot the first one's assets.
            Materials.Clear();
            try
            {
                var urdfPath = Path.GetFullPath(robot.UrdfRelative);
                if (!File.Exists(urdfPath))
                    throw new FileNotFoundException($"no URDF at {urdfPath}");

                var root = Path.GetDirectoryName(urdfPath);
                Directory.CreateDirectory(_meshDir);

                var doc = new XmlDocument();
                doc.Load(urdfPath);

                var links = ReadLinks(doc);
                var joints = ReadJoints(doc);
                Log($"{links.Count} links, {joints.Count} joints");

                var built = BuildHierarchy(links, joints, root);
                if (built == null) throw new Exception("no root link; is the URDF a tree?");

                Directory.CreateDirectory(_outDir);
                PrefabUtility.SaveAsPrefabAsset(built, _prefabPath);
                UnityEngine.Object.DestroyImmediate(built);

                AssetDatabase.SaveAssets();
                AssetDatabase.Refresh();
                Log($"wrote {_prefabPath}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[{_tag}] {e.GetType().Name}: {e.Message}\n{e.StackTrace}");
            }
        }

        // ------------------------------------------------------------------
        private sealed class LinkDef
        {
            public string Name;
            public string MeshFile;
            public Vector3 VisualXyz;
            public Vector3 VisualRpy;
            public Color Colour = new Color(0.72f, 0.74f, 0.76f);
        }

        private sealed class JointDef
        {
            public string Name, Parent, Child, Type;
            public Vector3 Xyz, Rpy, Axis;
        }

        private static List<LinkDef> ReadLinks(XmlDocument doc)
        {
            var list = new List<LinkDef>();
            foreach (XmlNode node in doc.SelectNodes("/robot/link"))
            {
                var link = new LinkDef { Name = Attr(node, "name") };
                var visual = node.SelectSingleNode("visual");
                if (visual != null)
                {
                    var mesh = visual.SelectSingleNode("geometry/mesh");
                    if (mesh != null) link.MeshFile = Attr(mesh, "filename");

                    var origin = visual.SelectSingleNode("origin");
                    if (origin != null)
                    {
                        link.VisualXyz = Vec(Attr(origin, "xyz"));
                        link.VisualRpy = Vec(Attr(origin, "rpy"));
                    }

                    var colour = visual.SelectSingleNode("material/color");
                    if (colour != null)
                    {
                        var c = Attr(colour, "rgba").Split(new[] { ' ' },
                                    StringSplitOptions.RemoveEmptyEntries);
                        if (c.Length >= 3)
                            link.Colour = new Color(F(c[0]), F(c[1]), F(c[2]),
                                                    c.Length > 3 ? F(c[3]) : 1f);
                    }
                }
                list.Add(link);
            }
            return list;
        }

        private static List<JointDef> ReadJoints(XmlDocument doc)
        {
            var list = new List<JointDef>();
            foreach (XmlNode node in doc.SelectNodes("/robot/joint"))
            {
                var parent = node.SelectSingleNode("parent");
                var child = node.SelectSingleNode("child");
                if (parent == null || child == null) continue;

                var j = new JointDef
                {
                    Name = Attr(node, "name"),
                    Type = Attr(node, "type"),
                    Parent = Attr(parent, "link"),
                    Child = Attr(child, "link"),
                    Axis = new Vector3(0f, 0f, 1f),
                };

                var origin = node.SelectSingleNode("origin");
                if (origin != null)
                {
                    j.Xyz = Vec(Attr(origin, "xyz"));
                    j.Rpy = Vec(Attr(origin, "rpy"));
                }
                var axis = node.SelectSingleNode("axis");
                if (axis != null) j.Axis = Vec(Attr(axis, "xyz"));

                list.Add(j);
            }
            return list;
        }

        // ------------------------------------------------------------------
        private static GameObject BuildHierarchy(List<LinkDef> links,
                                                 List<JointDef> joints,
                                                 string root)
        {
            var byName = new Dictionary<string, LinkDef>();
            foreach (var l in links) byName[l.Name] = l;

            var children = new HashSet<string>();
            foreach (var j in joints) children.Add(j.Child);

            string rootName = null;
            foreach (var l in links)
                if (!children.Contains(l.Name)) { rootName = l.Name; break; }
            if (rootName == null) return null;

            var objects = new Dictionary<string, GameObject>();
            foreach (var l in links)
            {
                var go = new GameObject(l.Name);
                objects[l.Name] = go;
                AttachVisual(go, l, root);
            }

            foreach (var j in joints)
            {
                if (!objects.TryGetValue(j.Parent, out var parent)) continue;
                if (!objects.TryGetValue(j.Child, out var child)) continue;

                child.transform.SetParent(parent.transform, false);
                child.transform.localPosition = RosToUnity(j.Xyz);
                child.transform.localRotation = RosToUnity(RpyToMatrix(j.Rpy));

                // Revolute joints get a marker so the runtime can find and
                // drive them by name without re-reading the URDF.
                if (j.Type == "revolute" || j.Type == "continuous")
                {
                    var axis = child.AddComponent<G1Joint>();
                    axis.JointName = j.Name;
                    axis.LocalAxis = RosToUnity(j.Axis).normalized;
                    axis.RestRotation = child.transform.localRotation;
                }
            }

            return objects[rootName];
        }

        private static void AttachVisual(GameObject go, LinkDef link, string root)
        {
            if (string.IsNullOrEmpty(link.MeshFile)) return;

            var path = Path.Combine(root, link.MeshFile.Replace("package://", ""));
            if (!File.Exists(path)) { Warn($"missing mesh {path}"); return; }

            var assetPath = $"{_meshDir}/{Path.GetFileNameWithoutExtension(path)}.asset";
            var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
            if (mesh == null)
            {
                mesh = StlReader.Read(path);
                if (mesh == null) { Warn($"could not read {path}"); return; }
                mesh.name = Path.GetFileNameWithoutExtension(path);
                AssetDatabase.CreateAsset(mesh, assetPath);
            }

            // A child holds the visual, because the URDF visual origin is an
            // offset from the link frame and the link frame is what joints
            // attach to. Collapsing them would move every child joint.
            var visual = new GameObject("visual");
            visual.transform.SetParent(go.transform, false);
            visual.transform.localPosition = RosToUnity(link.VisualXyz);
            visual.transform.localRotation = RosToUnity(RpyToMatrix(link.VisualRpy));

            visual.AddComponent<MeshFilter>().sharedMesh = mesh;
            var renderer = visual.AddComponent<MeshRenderer>();
            renderer.sharedMaterial = MaterialFor(link.Colour);
            renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            renderer.receiveShadows = false;
        }

        private static readonly Dictionary<string, Material> Materials =
            new Dictionary<string, Material>();

        private static Material MaterialFor(Color c)
        {
            var key = ColorUtility.ToHtmlStringRGBA(c);
            if (Materials.TryGetValue(key, out var cached)) return cached;

            var path = $"{_outDir}/Mat_{key}.mat";
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat == null)
            {
                mat = new Material(Shader.Find("Standard")) { color = c };
                mat.SetFloat("_Glossiness", 0.55f);
                mat.SetFloat("_Metallic", 0.35f);
                AssetDatabase.CreateAsset(mat, path);
            }
            Materials[key] = mat;
            return mat;
        }

        // ------------------------------------------------------------------
        // ROS -> Unity
        // ------------------------------------------------------------------
        public static Vector3 RosToUnity(Vector3 v) => new Vector3(-v.y, v.z, v.x);

        /// <summary>Rotation carried across the basis change as
        /// R_unity = M R_ros M^T, then read off as a quaternion. Doing it via
        /// the matrix avoids the sign errors that quaternion-component
        /// shuffling invites, and it is only run once per joint at import.</summary>
        private static Quaternion RosToUnity(Matrix4x4 rosRotation)
        {
            var m = Matrix4x4.identity;
            for (var col = 0; col < 3; col++)
            {
                var axis = new Vector3(rosRotation[0, col], rosRotation[1, col],
                                       rosRotation[2, col]);
                var mapped = RosToUnity(axis);
                m[0, col] = mapped.x; m[1, col] = mapped.y; m[2, col] = mapped.z;
            }
            // Columns are now the ROS axes in Unity space, in ROS order
            // (x,y,z). Reorder to Unity's own (x,y,z) = (-y_ros, z_ros, x_ros).
            var ux = -new Vector3(m[0, 1], m[1, 1], m[2, 1]);
            var uy = new Vector3(m[0, 2], m[1, 2], m[2, 2]);
            var uz = new Vector3(m[0, 0], m[1, 0], m[2, 0]);
            if (uz.sqrMagnitude < 1e-9f || uy.sqrMagnitude < 1e-9f) return Quaternion.identity;
            return Quaternion.LookRotation(uz, uy);
        }

        private static Matrix4x4 RpyToMatrix(Vector3 rpy)
        {
            // URDF fixed-axis roll-pitch-yaw: R = Rz(yaw) Ry(pitch) Rx(roll).
            float cr = Mathf.Cos(rpy.x), sr = Mathf.Sin(rpy.x);
            float cp = Mathf.Cos(rpy.y), sp = Mathf.Sin(rpy.y);
            float cy = Mathf.Cos(rpy.z), sy = Mathf.Sin(rpy.z);

            var m = Matrix4x4.identity;
            m[0, 0] = cy * cp; m[0, 1] = cy * sp * sr - sy * cr; m[0, 2] = cy * sp * cr + sy * sr;
            m[1, 0] = sy * cp; m[1, 1] = sy * sp * sr + cy * cr; m[1, 2] = sy * sp * cr - cy * sr;
            m[2, 0] = -sp;     m[2, 1] = cp * sr;                m[2, 2] = cp * cr;
            return m;
        }

        // ------------------------------------------------------------------
        private static string Attr(XmlNode node, string name)
        {
            return node?.Attributes?[name]?.Value ?? "";
        }

        private static Vector3 Vec(string s)
        {
            if (string.IsNullOrEmpty(s)) return Vector3.zero;
            var p = s.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
            return p.Length < 3 ? Vector3.zero : new Vector3(F(p[0]), F(p[1]), F(p[2]));
        }

        private static float F(string s)
        {
            return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture,
                                  out var v) ? v : 0f;
        }

        private static void Log(string m) => Debug.Log($"[{_tag}] {m}");
        private static void Warn(string m) => Debug.LogWarning($"[{_tag}] {m}");
    }
}
