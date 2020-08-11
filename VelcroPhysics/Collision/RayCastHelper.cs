using System;
using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.RayCast
{
    public static class RayCastHelper
    {
        public static bool RayCastEdge(ref Vec2 start, ref Vec2 end, ref RayCastInput input, ref Transform transform, out RayCastOutput output)
        {
            // p = p1 + t * d
            // v = v1 + s * e
            // p1 + t * d = v1 + s * e
            // s * e - t * d = p1 - v1

            output = new RayCastOutput();

            // Put the ray into the edge's frame of reference.
            Vec2 p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            Vec2 p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            Vec2 d = p2 - p1;

            Vec2 v1 = start;
            Vec2 v2 = end;
            Vec2 e = v2 - v1;
            Vec2 normal = new Vec2(e.Y, -e.X); //TODO: Could possibly cache the normal.
            normal.Normalize();

            // q = p1 + t * d
            // dot(normal, q - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0
            Fix64 numerator = Vec2.Dot(normal, v1 - p1);
            Fix64 denominator = Vec2.Dot(normal, d);

            if (denominator == Fix64.Zero)
            {
                return false;
            }

            Fix64 t = numerator / denominator;
            if (t < Fix64.Zero || input.MaxFraction < t)
            {
                return false;
            }

            Vec2 q = p1 + t * d;

            // q = v1 + s * r
            // s = dot(q - v1, r) / dot(r, r)
            Vec2 r = v2 - v1;
            Fix64 rr = Vec2.Dot(r, r);
            if (rr == Fix64.Zero)
            {
                return false;
            }

            Fix64 s = Vec2.Dot(q - v1, r) / rr;
            if (s < Fix64.Zero || Fix64.One < s)
            {
                return false;
            }

            output.Fraction = t;
            if (numerator > Fix64.Zero)
            {
                output.Normal = -MathUtils.MulT(transform.q, normal);
            }
            else
            {
                output.Normal = MathUtils.MulT(transform.q, normal);
            }
            return true;
        }

        public static bool RayCastCircle(ref Vec2 pos, Fix64 radius, ref RayCastInput input, ref Transform transform, out RayCastOutput output)
        {
            // Collision Detection in Interactive 3D Environments by Gino van den Bergen
            // From Section 3.1.2
            // x = s + a * r
            // norm(x) = radius

            output = new RayCastOutput();

            Vec2 position = transform.p + MathUtils.Mul(transform.q, pos);
            Vec2 s = input.Point1 - position;
            Fix64 b = Vec2.Dot(s, s) - radius * radius;

            // Solve quadratic equation.
            Vec2 r = input.Point2 - input.Point1;
            Fix64 c = Vec2.Dot(s, r);
            Fix64 rr = Vec2.Dot(r, r);
            Fix64 sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < Fix64.Zero || rr < Fix64.Epsilon)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            Fix64 a = -(c + Fix64.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (Fix64.Zero <= a && a <= input.MaxFraction * rr)
            {
                a /= rr;
                output.Fraction = a;
                output.Normal = s + a * r;
                output.Normal.Normalize();
                return true;
            }

            return false;
        }

        public static bool RayCastPolygon(Vertices vertices, Vertices normals, ref RayCastInput input, ref Transform transform,  out RayCastOutput output)
        {
            output = new RayCastOutput();

            // Put the ray into the polygon's frame of reference.
            Vec2 p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            Vec2 p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            Vec2 d = p2 - p1;

            Fix64 lower = Fix64.Zero, upper = input.MaxFraction;

            int index = -1;

            for (int i = 0; i < vertices.Count; ++i)
            {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                Fix64 numerator = Vec2.Dot(normals[i], vertices[i] - p1);
                Fix64 denominator = Vec2.Dot(normals[i], d);

                if (denominator == Fix64.Zero)
                {
                    if (numerator < Fix64.Zero)
                    {
                        return false;
                    }
                }
                else
                {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < Fix64.Zero && numerator < lower * denominator)
                    {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > Fix64.Zero && numerator < upper * denominator)
                    {
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                // The use of epsilon here causes the assert on lower to trip
                // in some cases. Apparently the use of epsilon was to make edge
                // shapes work, but now those are handled separately.
                //if (upper < lower - b2_epsilon)
                if (upper < lower)
                {
                    return false;
                }
            }

            Debug.Assert(Fix64.Zero <= lower && lower <= input.MaxFraction);

            if (index >= 0)
            {
                output.Fraction = lower;
                output.Normal = MathUtils.Mul(transform.q, normals[index]);
                return true;
            }

            return false;
        }
    }
}
