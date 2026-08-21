// Procedurally generated shapes for the align console.
//
// Generated rather than imported for the same reason the scene is generated:
// a sprite asset is a .png plus a .meta full of import settings, none of which
// survive review, and a wrong compression or filter mode shows up as a soft
// edge on device that nobody can trace back to a checkbox. These are a few
// hundred bytes of arithmetic each and they are exact.
//
// Everything is built once, cached statically, and shared by every element.
// The console builds roughly forty widgets out of four textures.

using System.Collections.Generic;
using UnityEngine;

namespace WeGo.Teleop
{
    internal static class TeleopHudTextures
    {
        private static readonly Dictionary<string, Sprite> Cache =
            new Dictionary<string, Sprite>();

        /// <summary>A rounded rectangle, optionally hollow, as a nine-sliced
        /// sprite. Nine-slice is what lets one 64px texture back panels of any
        /// size without the corner radius stretching with them.</summary>
        public static Sprite RoundedRect(int radius, float border = 0f)
        {
            var key = $"rr{radius}:{border}";
            if (Cache.TryGetValue(key, out var cached)) return cached;

            var size = radius * 2 + 4;
            var tex = NewTexture(size, size);
            var px = new Color[size * size];

            for (var y = 0; y < size; y++)
            for (var x = 0; x < size; x++)
            {
                // Distance to the rounded-rect boundary, measured from the
                // nearest corner centre. Sampling at the pixel centre (+0.5)
                // and taking a one-pixel ramp is what antialiases the curve.
                var cx = Mathf.Clamp(x + 0.5f, radius, size - radius);
                var cy = Mathf.Clamp(y + 0.5f, radius, size - radius);
                var d = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f),
                                         new Vector2(cx, cy));
                var outer = Mathf.Clamp01(radius - d + 0.5f);
                var a = outer;
                if (border > 0f)
                {
                    // Hollow: subtract a second, smaller rounded rect.
                    var inner = Mathf.Clamp01(radius - border - d + 0.5f);
                    a = Mathf.Clamp01(outer - inner);
                }
                px[y * size + x] = new Color(1f, 1f, 1f, a);
            }

            tex.SetPixels(px);
            tex.Apply(false, false);

            var slice = radius + 1;
            var sprite = Sprite.Create(tex, new Rect(0, 0, size, size),
                                       new Vector2(0.5f, 0.5f), 100f, 0,
                                       SpriteMeshType.FullRect,
                                       new Vector4(slice, slice, slice, slice));
            Cache[key] = sprite;
            return sprite;
        }

        /// <summary>An annulus, drawn as a radial-filled Image to make the
        /// alignment gauge. Thickness is a fraction of the radius.</summary>
        public static Sprite Ring(int size, float thickness)
        {
            var key = $"ring{size}:{thickness}";
            if (Cache.TryGetValue(key, out var cached)) return cached;

            var tex = NewTexture(size, size);
            var px = new Color[size * size];
            var r = size * 0.5f;
            var inner = r - r * thickness;

            for (var y = 0; y < size; y++)
            for (var x = 0; x < size; x++)
            {
                var d = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f),
                                         new Vector2(r, r));
                var a = Mathf.Clamp01(r - 1f - d + 0.5f) *
                        Mathf.Clamp01(d - inner + 0.5f);
                px[y * size + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(a));
            }

            tex.SetPixels(px);
            tex.Apply(false, false);
            var sprite = Sprite.Create(tex, new Rect(0, 0, size, size),
                                       new Vector2(0.5f, 0.5f));
            Cache[key] = sprite;
            return sprite;
        }

        /// <summary>A filled disc. Used for the checklist markers and the
        /// e-stop reminder.</summary>
        public static Sprite Disc(int size)
        {
            var key = $"disc{size}";
            if (Cache.TryGetValue(key, out var cached)) return cached;

            var tex = NewTexture(size, size);
            var px = new Color[size * size];
            var r = size * 0.5f;

            for (var y = 0; y < size; y++)
            for (var x = 0; x < size; x++)
            {
                var d = Vector2.Distance(new Vector2(x + 0.5f, y + 0.5f),
                                         new Vector2(r, r));
                px[y * size + x] = new Color(1f, 1f, 1f, Mathf.Clamp01(r - 1f - d + 0.5f));
            }

            tex.SetPixels(px);
            tex.Apply(false, false);
            var sprite = Sprite.Create(tex, new Rect(0, 0, size, size),
                                       new Vector2(0.5f, 0.5f));
            Cache[key] = sprite;
            return sprite;
        }

        /// <summary>One scanline period, tiled by the console background.
        /// Subtle on purpose -- it is there to stop a large flat panel reading
        /// as a solid slab floating in the room, not to look like a prop.</summary>
        public static Sprite Scanline(float strength)
        {
            var key = $"scan{strength}";
            if (Cache.TryGetValue(key, out var cached)) return cached;

            var tex = NewTexture(1, 3);
            tex.wrapMode = TextureWrapMode.Repeat;
            tex.SetPixels(new[]
            {
                new Color(1f, 1f, 1f, strength),
                new Color(1f, 1f, 1f, 0f),
                new Color(1f, 1f, 1f, 0f),
            });
            tex.Apply(false, false);

            var sprite = Sprite.Create(tex, new Rect(0, 0, 1, 3),
                                       new Vector2(0.5f, 0.5f), 3f);
            Cache[key] = sprite;
            return sprite;
        }

        private static Texture2D NewTexture(int w, int h)
        {
            return new Texture2D(w, h, TextureFormat.RGBA32, false, true)
            {
                // Bilinear + clamp: the panels are viewed from a hand's-breadth
                // away in a headset, where point filtering on a curve is
                // immediately obvious as stair-stepping.
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
                anisoLevel = 1,
            };
        }
    }
}
