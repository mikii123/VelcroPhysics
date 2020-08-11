using FixedMath.Net;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision
{
    public static class TestPointHelper
    {
        public static bool TestPointCircle(ref Vec2 pos, Fix64 radius, ref Vec2 point, ref Transform transform)
        {
            Vec2 center = transform.p + MathUtils.Mul(transform.q, pos);
            Vec2 d = point - center;
            return Vec2.Dot(d, d) <= radius * radius;
        }

        public static bool TestPointPolygon(Vertices vertices, Vertices normals, ref Vec2 point, ref Transform transform)
        {
            Vec2 pLocal = MathUtils.MulT(transform.q, point - transform.p);

            for (int i = 0; i < vertices.Count; ++i)
            {
                Fix64 dot = Vec2.Dot(normals[i], pLocal - vertices[i]);
                if (dot > Fix64.Zero)
                {
                    return false;
                }
            }

            return true;
        }
    }
}
