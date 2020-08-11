using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class CollideEdge
    {
        /// <summary>
        /// Compute contact points for edge versus circle.
        /// This accounts for edge connectivity.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="edgeA">The edge A.</param>
        /// <param name="transformA">The transform A.</param>
        /// <param name="circleB">The circle B.</param>
        /// <param name="transformB">The transform B.</param>
        public static void CollideEdgeAndCircle(ref Manifold manifold, EdgeShape edgeA, ref Transform transformA, CircleShape circleB, ref Transform transformB)
        {
            manifold.PointCount = 0;

            // Compute circle in frame of edge
            Vec2 Q = MathUtils.MulT(ref transformA, MathUtils.Mul(ref transformB, ref circleB._position));

            Vec2 A = edgeA.Vertex1, B = edgeA.Vertex2;
            Vec2 e = B - A;

            // Barycentric coordinates
            Fix64 u = Vec2.Dot(e, B - Q);
            Fix64 v = Vec2.Dot(e, Q - A);

            Fix64 radius = edgeA.Radius + circleB.Radius;

            ContactFeature cf;
            cf.IndexB = 0;
            cf.TypeB = ContactFeatureType.Vertex;

            // Region A
            if (v <= Fix64.Zero)
            {
                Vec2 P1 = A;
                Vec2 d1 = Q - P1;
                Fix64 dd1 = Vec2.Dot(d1, d1);
                if (dd1 > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to A?
                if (edgeA.HasVertex0)
                {
                    Vec2 A1 = edgeA.Vertex0;
                    Vec2 B1 = A;
                    Vec2 e1 = B1 - A1;
                    Fix64 u1 = Vec2.Dot(e1, B1 - Q);

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > Fix64.Zero)
                    {
                        return;
                    }
                }

                cf.IndexA = 0;
                cf.TypeA = ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = Vec2.Zero;
                manifold.LocalPoint = P1;
                manifold.Points.Value0.Id.Key = 0;
                manifold.Points.Value0.Id.ContactFeature = cf;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                return;
            }

            // Region B
            if (u <= Fix64.Zero)
            {
                Vec2 P2 = B;
                Vec2 d2 = Q - P2;
                Fix64 dd2 = Vec2.Dot(d2, d2);
                if (dd2 > radius * radius)
                {
                    return;
                }

                // Is there an edge connected to B?
                if (edgeA.HasVertex3)
                {
                    Vec2 B2 = edgeA.Vertex3;
                    Vec2 A2 = B;
                    Vec2 e2 = B2 - A2;
                    Fix64 v2 = Vec2.Dot(e2, Q - A2);

                    // Is the circle in Region AB of the next edge?
                    if (v2 > Fix64.Zero)
                    {
                        return;
                    }
                }

                cf.IndexA = 1;
                cf.TypeA = (byte)ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = Vec2.Zero;
                manifold.LocalPoint = P2;
                manifold.Points.Value0.Id.Key = 0;
                manifold.Points.Value0.Id.ContactFeature = cf;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                return;
            }

            // Region AB
            Fix64 den = Vec2.Dot(e, e);
            Debug.Assert(den > Fix64.Zero);
            Vec2 P = (Fix64.One / den) * (u * A + v * B);
            Vec2 d = Q - P;
            Fix64 dd = Vec2.Dot(d, d);
            if (dd > radius * radius)
            {
                return;
            }

            Vec2 n = new Vec2(-e.Y, e.X);
            if (Vec2.Dot(n, Q - A) < Fix64.Zero)
            {
                n = new Vec2(-n.X, -n.Y);
            }
            n.Normalize();

            cf.IndexA = 0;
            cf.TypeA = ContactFeatureType.Face;
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = n;
            manifold.LocalPoint = A;
            manifold.Points.Value0.Id.Key = 0;
            manifold.Points.Value0.Id.ContactFeature = cf;
            manifold.Points.Value0.LocalPoint = circleB.Position;
        }

        /// <summary>
        /// Collides and edge and a polygon, taking into account edge adjacency.
        /// </summary>
        public static void CollideEdgeAndPolygon(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA, PolygonShape polygonB, ref Transform xfB)
        {
            EPCollider.Collide(ref manifold, edgeA, ref xfA, polygonB, ref xfB);
        }
    }
}
