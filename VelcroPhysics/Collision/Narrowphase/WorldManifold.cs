using FixedMath.Net;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class WorldManifold
    {
        /// <summary>
        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the Shapes
        /// that generated the manifold.
        /// </summary>
        public static void Initialize(ref Manifold manifold, ref Transform xfA, Fix64 radiusA, ref Transform xfB, Fix64 radiusB, out Vec2 normal, out FixedArray2<Vec2> points, out FixedArray2<Fix64> separations)
        {
            normal = Vec2.Zero;
            points = new FixedArray2<Vec2>();
            separations = new FixedArray2<Fix64>();

            if (manifold.PointCount == 0)
            {
                return;
            }

            switch (manifold.Type)
            {
                case ManifoldType.Circles:
                    {
                        normal = new Vec2(Fix64.One, Fix64.Zero);
                        Vec2 pointA = MathUtils.Mul(ref xfA, manifold.LocalPoint);
                        Vec2 pointB = MathUtils.Mul(ref xfB, manifold.Points.Value0.LocalPoint);
                        if (Vec2.DistanceSquared(pointA, pointB) > Fix64.Epsilon * Fix64.Epsilon)
                        {
                            normal = pointB - pointA;
                            normal.Normalize();
                        }

                        Vec2 cA = pointA + radiusA * normal;
                        Vec2 cB = pointB - radiusB * normal;
                        points.Value0 = Fix64.Half * (cA + cB);
                        separations.Value0 = Vec2.Dot(cB - cA, normal);
                    }
                    break;

                case ManifoldType.FaceA:
                    {
                        normal = MathUtils.Mul(xfA.q, manifold.LocalNormal);
                        Vec2 planePoint = MathUtils.Mul(ref xfA, manifold.LocalPoint);

                        for (int i = 0; i < manifold.PointCount; ++i)
                        {
                            Vec2 clipPoint = MathUtils.Mul(ref xfB, manifold.Points[i].LocalPoint);
                            Vec2 cA = clipPoint + (radiusA - Vec2.Dot(clipPoint - planePoint, normal)) * normal;
                            Vec2 cB = clipPoint - radiusB * normal;
                            points[i] = Fix64.Half * (cA + cB);
                            separations[i] = Vec2.Dot(cB - cA, normal);
                        }
                    }
                    break;

                case ManifoldType.FaceB:
                    {
                        normal = MathUtils.Mul(xfB.q, manifold.LocalNormal);
                        Vec2 planePoint = MathUtils.Mul(ref xfB, manifold.LocalPoint);

                        for (int i = 0; i < manifold.PointCount; ++i)
                        {
                            Vec2 clipPoint = MathUtils.Mul(ref xfA, manifold.Points[i].LocalPoint);
                            Vec2 cB = clipPoint + (radiusB - Vec2.Dot(clipPoint - planePoint, normal)) * normal;
                            Vec2 cA = clipPoint - radiusA * normal;
                            points[i] = Fix64.Half * (cA + cB);
                            separations[i] = Vec2.Dot(cA - cB, normal);
                        }

                        // Ensure normal points from A to B.
                        normal = -normal;
                    }
                    break;
            }
        }
    }
}