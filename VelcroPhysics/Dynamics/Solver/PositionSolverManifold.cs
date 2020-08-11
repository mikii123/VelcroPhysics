using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Dynamics.Solver
{
    public static class PositionSolverManifold
    {
        public static void Initialize(ContactPositionConstraint pc, Transform xfA, Transform xfB, int index, out Vec2 normal, out Vec2 point, out Fix64 separation)
        {
            Debug.Assert(pc.PointCount > 0);

            switch (pc.Type)
            {
                case ManifoldType.Circles:
                    {
                        Vec2 pointA = MathUtils.Mul(ref xfA, pc.LocalPoint);
                        Vec2 pointB = MathUtils.Mul(ref xfB, pc.LocalPoints[0]);
                        normal = pointB - pointA;

                        //Velcro: Fix to handle zero normalization
                        if (normal != Vec2.Zero)
                            normal.Normalize();

                        point = Fix64.Half * (pointA + pointB);
                        separation = Vec2.Dot(pointB - pointA, normal) - pc.RadiusA - pc.RadiusB;
                    }
                    break;

                case ManifoldType.FaceA:
                    {
                        normal = MathUtils.Mul(xfA.q, pc.LocalNormal);
                        Vec2 planePoint = MathUtils.Mul(ref xfA, pc.LocalPoint);

                        Vec2 clipPoint = MathUtils.Mul(ref xfB, pc.LocalPoints[index]);
                        separation = Vec2.Dot(clipPoint - planePoint, normal) - pc.RadiusA - pc.RadiusB;
                        point = clipPoint;
                    }
                    break;

                case ManifoldType.FaceB:
                    {
                        normal = MathUtils.Mul(xfB.q, pc.LocalNormal);
                        Vec2 planePoint = MathUtils.Mul(ref xfB, pc.LocalPoint);

                        Vec2 clipPoint = MathUtils.Mul(ref xfA, pc.LocalPoints[index]);
                        separation = Vec2.Dot(clipPoint - planePoint, normal) - pc.RadiusA - pc.RadiusB;
                        point = clipPoint;

                        // Ensure normal points from A to B
                        normal = -normal;
                    }
                    break;
                default:
                    normal = Vec2.Zero;
                    point = Vec2.Zero;
                    separation = Fix64.Zero;
                    break;
            }
        }
    }
}