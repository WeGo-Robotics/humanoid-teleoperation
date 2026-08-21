// ASCII and binary STL to Unity Mesh.
//
// Editor-only: this reads the vendored g1_description meshes at import time so
// that nothing on device ever parses geometry. The Unitree STLs are ASCII,
// which is roughly 250 bytes per triangle -- 52 MB across the model -- so the
// parser is written to stream rather than to split the file into an array of
// several million substrings.
//
// STL has no vertex sharing: every triangle carries three full vertices. The
// reader welds them on a quantised key, which takes the G1's torso from about
// 90,000 loose vertices to a third of that, and lets Unity compute smooth
// normals instead of the faceted ones STL implies.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

namespace WeGo.Teleop.Editor
{
    public static class StlReader
    {
        public static Mesh Read(string path)
        {
            return IsAscii(path) ? ReadAscii(path) : ReadBinary(path);
        }

        private static bool IsAscii(string path)
        {
            using (var s = File.OpenRead(path))
            {
                var head = new byte[6];
                if (s.Read(head, 0, 6) < 6) return false;
                // A binary STL may still begin with "solid" in its 80-byte
                // header, so confirm with the size the triangle count implies.
                if (Encoding.ASCII.GetString(head, 0, 5).ToLowerInvariant() != "solid")
                    return false;

                if (s.Length < 84) return true;
                s.Seek(80, SeekOrigin.Begin);
                var countBytes = new byte[4];
                if (s.Read(countBytes, 0, 4) < 4) return true;
                var count = BitConverter.ToUInt32(countBytes, 0);
                return 84L + count * 50L != s.Length;
            }
        }

        // ------------------------------------------------------------------
        private static Mesh ReadAscii(string path)
        {
            var builder = new Builder();
            var tri = new Vector3[3];
            var n = 0;

            using (var reader = new StreamReader(path, Encoding.ASCII, false, 1 << 20))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    var i = 0;
                    while (i < line.Length && char.IsWhiteSpace(line[i])) i++;
                    // Only "vertex" lines carry geometry; normals are recomputed.
                    if (i + 6 > line.Length) continue;
                    if (line[i] != 'v' || line[i + 1] != 'e' || line[i + 2] != 'r') continue;

                    if (!ParseTriple(line, i + 6, out var v)) continue;
                    tri[n++] = v;
                    if (n < 3) continue;
                    n = 0;
                    builder.AddTriangle(tri[0], tri[1], tri[2]);
                }
            }
            return builder.Build();
        }

        /// <summary>Hand-rolled float scan. string.Split on every vertex line
        /// allocates three strings per vertex and, over six hundred thousand
        /// vertices, spends more time in the garbage collector than in the
        /// parser.</summary>
        private static bool ParseTriple(string s, int start, out Vector3 v)
        {
            v = default;
            var i = start;
            if (!ScanFloat(s, ref i, out var x)) return false;
            if (!ScanFloat(s, ref i, out var y)) return false;
            if (!ScanFloat(s, ref i, out var z)) return false;
            v = new Vector3(x, y, z);
            return true;
        }

        private static bool ScanFloat(string s, ref int i, out float value)
        {
            value = 0f;
            while (i < s.Length && char.IsWhiteSpace(s[i])) i++;
            var begin = i;
            while (i < s.Length && !char.IsWhiteSpace(s[i])) i++;
            if (i <= begin) return false;
            return float.TryParse(s.Substring(begin, i - begin), NumberStyles.Float,
                                  CultureInfo.InvariantCulture, out value);
        }

        // ------------------------------------------------------------------
        private static Mesh ReadBinary(string path)
        {
            using (var reader = new BinaryReader(File.OpenRead(path)))
            {
                reader.ReadBytes(80);
                var count = reader.ReadUInt32();
                var builder = new Builder();
                for (var t = 0; t < count; t++)
                {
                    reader.ReadSingle(); reader.ReadSingle(); reader.ReadSingle();  // normal
                    var a = ReadVec(reader);
                    var b = ReadVec(reader);
                    var c = ReadVec(reader);
                    reader.ReadUInt16();                                            // attributes
                    builder.AddTriangle(a, b, c);
                }
                return builder.Build();
            }
        }

        private static Vector3 ReadVec(BinaryReader r)
        {
            return new Vector3(r.ReadSingle(), r.ReadSingle(), r.ReadSingle());
        }

        // ------------------------------------------------------------------
        private sealed class Builder
        {
            private readonly List<Vector3> _vertices = new List<Vector3>();
            private readonly List<int> _indices = new List<int>();
            private readonly Dictionary<long, int> _lookup = new Dictionary<long, int>();

            public void AddTriangle(Vector3 a, Vector3 b, Vector3 c)
            {
                // Reversed winding. The ROS->Unity basis change has determinant
                // -1, so preserving STL's order would leave every surface
                // facing inward.
                _indices.Add(Index(G1ModelImporter.RosToUnity(c)));
                _indices.Add(Index(G1ModelImporter.RosToUnity(b)));
                _indices.Add(Index(G1ModelImporter.RosToUnity(a)));
            }

            /// <summary>Welds on a 0.1 mm lattice. The G1's meshes are modelled
            /// in metres to roughly that precision, so this merges the shared
            /// corners STL duplicates without pulling distinct features
            /// together.</summary>
            private int Index(Vector3 v)
            {
                const float q = 10000f;
                var key = ((long)Mathf.RoundToInt(v.x * q) * 73856093L)
                        ^ ((long)Mathf.RoundToInt(v.y * q) * 19349663L)
                        ^ ((long)Mathf.RoundToInt(v.z * q) * 83492791L);

                if (_lookup.TryGetValue(key, out var existing)) return existing;
                var index = _vertices.Count;
                _vertices.Add(v);
                _lookup[key] = index;
                return index;
            }

            public Mesh Build()
            {
                if (_indices.Count == 0) return null;
                var mesh = new Mesh
                {
                    // The torso alone exceeds the 16-bit index limit.
                    indexFormat = _vertices.Count > 65000
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16,
                };
                mesh.SetVertices(_vertices);
                mesh.SetTriangles(_indices, 0);
                mesh.RecalculateNormals();
                mesh.RecalculateBounds();
                mesh.Optimize();
                return mesh;
            }
        }
    }
}
