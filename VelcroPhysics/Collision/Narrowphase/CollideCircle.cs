using FixedMath.Net;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class CollideCircle
    {
        /// <summary>
        /// Compute the collision manifold between two circles.
        /// </summary>
        public static void CollideCircles(ref Manifold manifold, CircleShape circleA, ref Transform xfA, CircleShape circleB, ref Transform xfB)
        {
            manifold.PointCount = 0;

            Vec2 pA = MathUtils.Mul(ref xfA, circleA.Position);
            Vec2 pB = MathUtils.Mul(ref xfB, circleB.Position);

            Vec2 d = pB - pA;
            Fix64 distSqr = Vec2.Dot(d, d);
            Fix64 rA = circleA.Radius, rB = circleB.Radius;
            Fix64 radius = rA + rB;
            if (distSqr > radius * radius)
            {
                return;
            }

            manifold.Type = ManifoldType.Circles;
            manifold.LocalPoint = circleA.Position;
            manifold.LocalNormal = Vec2.Zero;
            manifold.PointCount = 1;

            ManifoldPoint p0 = manifold.Points[0];
            p0.LocalPoint = circleB.Position;
            p0.Id.Key = 0;
            manifold.Points[0] = p0;
        }

        /// <summary>
        /// Compute the collision manifold between a polygon and a circle.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="polygonA">The polygon A.</param>
        /// <param name="xfA">The transform of A.</param>
        /// <param name="circleB">The circle B.</param>
        /// <param name="xfB">The transform of B.</param>
        public static void CollidePolygonAndCircle(ref Manifold manifold, PolygonShape polygonA, ref Transform xfA, CircleShape circleB, ref Transform xfB)
        {
            manifold.PointCount = 0;

            // Compute circle position in the frame of the polygon.
            Vec2 c = MathUtils.Mul(ref xfB, circleB.Position);
            Vec2 cLocal = MathUtils.MulT(ref xfA, c);

            // Find the min separating edge.
            int normalIndex = 0;
            Fix64 separation = Fix64.MinValue;
            Fix64 radius = polygonA.Radius + circleB.Radius;
            int vertexCount = polygonA.Vertices.Count;
            Vertices vertices = polygonA.Vertices;
            Vertices normals = polygonA.Normals;

            for (int i = 0; i < vertexCount; ++i)
            {
                Fix64 s = Vec2.Dot(normals[i], cLocal - vertices[i]);

                if (s > radius)
                {
                    // Early out.
                    return;
                }

                if (s > separation)
                {
                    separation = s;
                    normalIndex = i;
                }
            }

            // Vertices that subtend the incident face.
            int vertIndex1 = normalIndex;
            int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
            Vec2 v1 = vertices[vertIndex1];
            Vec2 v2 = vertices[vertIndex2];

            // If the center is inside the polygon ...
            if (separation < Fix64.Epsilon)
            {
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = normals[normalIndex];
                manifold.LocalPoint = Fix64.Half * (v1 + v2);
                manifold.Points.Value0.LocalPoint = circleB.Position;
                manifold.Points.Value0.Id.Key = 0;
                return;
            }

            // Compute barycentric coordinates
            Fix64 u1 = Vec2.Dot(cLocal - v1, v2 - v1);
            Fix64 u2 = Vec2.Dot(cLocal - v2, v1 - v2);

            if (u1 <= Fix64.Zero)
            {
                if (Vec2.DistanceSquared(cLocal, v1) > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = cLocal - v1;
                manifold.LocalNormal.Normalize();
                manifold.LocalPoint = v1;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                manifold.Points.Value0.Id.Key = 0;
            }
            else if (u2 <= Fix64.Zero)
            {
                if (Vec2.DistanceSquared(cLocal, v2) > radius * radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = cLocal - v2;
                manifold.LocalNormal.Normalize();
                manifold.LocalPoint = v2;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                manifold.Points.Value0.Id.Key = 0;
            }
            else
            {
                Vec2 faceCenter = Fix64.Half * (v1 + v2);
                Fix64 s = Vec2.Dot(cLocal - faceCenter, normals[vertIndex1]);
                if (s > radius)
                {
                    return;
                }

                manifold.PointCount = 1;
                manifold.Type = ManifoldType.FaceA;
                manifold.LocalNormal = normals[vertIndex1];
                manifold.LocalPoint = faceCenter;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                manifold.Points.Value0.Id.Key = 0;
            }
        }
    }
}