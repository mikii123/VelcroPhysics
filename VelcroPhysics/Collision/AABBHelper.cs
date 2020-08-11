using FixedMath.Net;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision
{
    public static class AABBHelper
    {
        public static void ComputeEdgeAABB(ref Vec2 start, ref Vec2 end, ref Transform transform, out AABB aabb)
        {
            Vec2 v1 = MathUtils.Mul(ref transform, ref start);
            Vec2 v2 = MathUtils.Mul(ref transform, ref end);

            aabb.LowerBound = Vec2.Min(v1, v2);
            aabb.UpperBound = Vec2.Max(v1, v2);

            Vec2 r = new Vec2(Settings.PolygonRadius, Settings.PolygonRadius);
            aabb.LowerBound = aabb.LowerBound - r;
            aabb.UpperBound = aabb.UpperBound + r;
        }

        public static void ComputeCircleAABB(ref Vec2 pos, Fix64 radius, ref Transform transform, out AABB aabb)
        {
            Vec2 p = transform.p + MathUtils.Mul(transform.q, pos);
            aabb.LowerBound = new Vec2(p.X - radius, p.Y - radius);
            aabb.UpperBound = new Vec2(p.X + radius, p.Y + radius);
        }

        public static void ComputePolygonAABB(Vertices vertices, ref Transform transform, out AABB aabb)
        {
            Vec2 lower = MathUtils.Mul(ref transform, vertices[0]);
            Vec2 upper = lower;

            for (int i = 1; i < vertices.Count; ++i)
            {
                Vec2 v = MathUtils.Mul(ref transform, vertices[i]);
                lower = Vec2.Min(lower, v);
                upper = Vec2.Max(upper, v);
            }

            Vec2 r = new Vec2(Settings.PolygonRadius, Settings.PolygonRadius);
            aabb.LowerBound = lower - r;
            aabb.UpperBound = upper + r;
        }
    }
}
