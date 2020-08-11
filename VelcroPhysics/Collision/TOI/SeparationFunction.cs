using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.TOI
{
    public static class SeparationFunction
    {
        public static void Initialize(ref SimplexCache cache, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, Fix64 t1, out Vec2 axis, out Vec2 localPoint, out SeparationFunctionType type)
        {
            int count = cache.Count;
            Debug.Assert(0 < count && count < 3);

            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t1);
            sweepB.GetTransform(out xfB, t1);

            if (count == 1)
            {
                localPoint = Vec2.Zero;
                type = SeparationFunctionType.Points;
                Vec2 localPointA = proxyA.Vertices[cache.IndexA[0]];
                Vec2 localPointB = proxyB.Vertices[cache.IndexB[0]];
                Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);
                Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);
                axis = pointB - pointA;
                axis.Normalize();
            }
            else if (cache.IndexA[0] == cache.IndexA[1])
            {
                // Two points on B and one on A.
                type = SeparationFunctionType.FaceB;
                Vec2 localPointB1 = proxyB.Vertices[cache.IndexB[0]];
                Vec2 localPointB2 = proxyB.Vertices[cache.IndexB[1]];

                Vec2 a = localPointB2 - localPointB1;
                axis = new Vec2(a.Y, -a.X);
                axis.Normalize();
                Vec2 normal = MathUtils.Mul(ref xfB.q, axis);

                localPoint = Fix64.Half * (localPointB1 + localPointB2);
                Vec2 pointB = MathUtils.Mul(ref xfB, localPoint);

                Vec2 localPointA = proxyA.Vertices[cache.IndexA[0]];
                Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);

                Fix64 s = Vec2.Dot(pointA - pointB, normal);
                if (s < Fix64.Zero)
                {
                    axis = -axis;
                }
            }
            else
            {
                // Two points on A and one or two points on B.
                type = SeparationFunctionType.FaceA;
                Vec2 localPointA1 = proxyA.Vertices[cache.IndexA[0]];
                Vec2 localPointA2 = proxyA.Vertices[cache.IndexA[1]];

                Vec2 a = localPointA2 - localPointA1;
                axis = new Vec2(a.Y, -a.X);
                axis.Normalize();
                Vec2 normal = MathUtils.Mul(ref xfA.q, axis);

                localPoint = Fix64.Half * (localPointA1 + localPointA2);
                Vec2 pointA = MathUtils.Mul(ref xfA, localPoint);

                Vec2 localPointB = proxyB.Vertices[cache.IndexB[0]];
                Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);

                Fix64 s = Vec2.Dot(pointB - pointA, normal);
                if (s < Fix64.Zero)
                {
                    axis = -axis;
                }
            }

            //Velcro note: the returned value that used to be here has been removed, as it was not used.
        }

        public static Fix64 FindMinSeparation(out int indexA, out int indexB, Fix64 t, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, ref Vec2 axis, ref Vec2 localPoint, SeparationFunctionType type)
        {
            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t);
            sweepB.GetTransform(out xfB, t);

            switch (type)
            {
                case SeparationFunctionType.Points:
                    {
                        Vec2 axisA = MathUtils.MulT(ref xfA.q, axis);
                        Vec2 axisB = MathUtils.MulT(ref xfB.q, -axis);

                        indexA = proxyA.GetSupport(axisA);
                        indexB = proxyB.GetSupport(axisB);

                        Vec2 localPointA = proxyA.Vertices[indexA];
                        Vec2 localPointB = proxyB.Vertices[indexB];

                        Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        Fix64 separation = Vec2.Dot(pointB - pointA, axis);
                        return separation;
                    }

                case SeparationFunctionType.FaceA:
                    {
                        Vec2 normal = MathUtils.Mul(ref xfA.q, axis);
                        Vec2 pointA = MathUtils.Mul(ref xfA, localPoint);

                        Vec2 axisB = MathUtils.MulT(ref xfB.q, -normal);

                        indexA = -1;
                        indexB = proxyB.GetSupport(axisB);

                        Vec2 localPointB = proxyB.Vertices[indexB];
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        Fix64 separation = Vec2.Dot(pointB - pointA, normal);
                        return separation;
                    }

                case SeparationFunctionType.FaceB:
                    {
                        Vec2 normal = MathUtils.Mul(ref xfB.q, axis);
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPoint);

                        Vec2 axisA = MathUtils.MulT(ref xfA.q, -normal);

                        indexB = -1;
                        indexA = proxyA.GetSupport(axisA);

                        Vec2 localPointA = proxyA.Vertices[indexA];
                        Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);

                        Fix64 separation = Vec2.Dot(pointA - pointB, normal);
                        return separation;
                    }

                default:
                    Debug.Assert(false);
                    indexA = -1;
                    indexB = -1;
                    return Fix64.Zero;
            }
        }

        public static Fix64 Evaluate(int indexA, int indexB, Fix64 t, DistanceProxy proxyA, ref Sweep sweepA, DistanceProxy proxyB, ref Sweep sweepB, ref Vec2 axis, ref Vec2 localPoint, SeparationFunctionType type)
        {
            Transform xfA, xfB;
            sweepA.GetTransform(out xfA, t);
            sweepB.GetTransform(out xfB, t);

            switch (type)
            {
                case SeparationFunctionType.Points:
                    {
                        Vec2 localPointA = proxyA.Vertices[indexA];
                        Vec2 localPointB = proxyB.Vertices[indexB];

                        Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);
                        Fix64 separation = Vec2.Dot(pointB - pointA, axis);

                        return separation;
                    }
                case SeparationFunctionType.FaceA:
                    {
                        Vec2 normal = MathUtils.Mul(ref xfA.q, axis);
                        Vec2 pointA = MathUtils.Mul(ref xfA, localPoint);

                        Vec2 localPointB = proxyB.Vertices[indexB];
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPointB);

                        Fix64 separation = Vec2.Dot(pointB - pointA, normal);
                        return separation;
                    }
                case SeparationFunctionType.FaceB:
                    {
                        Vec2 normal = MathUtils.Mul(ref xfB.q, axis);
                        Vec2 pointB = MathUtils.Mul(ref xfB, localPoint);

                        Vec2 localPointA = proxyA.Vertices[indexA];
                        Vec2 pointA = MathUtils.Mul(ref xfA, localPointA);

                        Fix64 separation = Vec2.Dot(pointA - pointB, normal);
                        return separation;
                    }
                default:
                    Debug.Assert(false);
                    return Fix64.Zero;
            }
        }
    }
}