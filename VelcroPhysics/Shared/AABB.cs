using System;
using FixedMath.Net;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// An axis aligned bounding box.
    /// </summary>
    public struct AABB
    {
        /// <summary>
        /// The lower vertex
        /// </summary>
        public Vec2 LowerBound;

        /// <summary>
        /// The upper vertex
        /// </summary>
        public Vec2 UpperBound;

        public AABB(Vec2 min, Vec2 max)
            : this(ref min, ref max) { }

        public AABB(Vec2 center, Fix64 width, Fix64 height)
            : this(center - new Vec2(width / Fix64.Two, height / Fix64.Two), center + new Vec2(width / Fix64.Two, height / Fix64.Two))
        {
        }

        public AABB(ref Vec2 min, ref Vec2 max)
        {
            LowerBound = new Vec2(Fix64.Min(min.X, max.X), Fix64.Min(min.Y, max.Y));
            UpperBound = new Vec2(Fix64.Max(min.X, max.X), Fix64.Max(min.Y, max.Y));
        }

        public Fix64 Width => UpperBound.X - LowerBound.X;

        public Fix64 Height => UpperBound.Y - LowerBound.Y;

        /// <summary>
        /// Get the center of the AABB.
        /// </summary>
        public Vec2 Center => Fix64.Half * (LowerBound + UpperBound);

        /// <summary>
        /// Get the extents of the AABB (half-widths).
        /// </summary>
        public Vec2 Extents => Fix64.Half * (UpperBound - LowerBound);

        /// <summary>
        /// Get the perimeter length
        /// </summary>
        public Fix64 Perimeter
        {
            get
            {
                Fix64 wx = UpperBound.X - LowerBound.X;
                Fix64 wy = UpperBound.Y - LowerBound.Y;
                return Fix64.Two * (wx + wy);
            }
        }

        /// <summary>
        /// Gets the vertices of the AABB.
        /// </summary>
        /// <value>The corners of the AABB</value>
        public Vertices Vertices
        {
            get
            {
                Vertices vertices = new Vertices(4);
                vertices.Add(UpperBound);
                vertices.Add(new Vec2(UpperBound.X, LowerBound.Y));
                vertices.Add(LowerBound);
                vertices.Add(new Vec2(LowerBound.X, UpperBound.Y));
                return vertices;
            }
        }

        /// <summary>
        /// First quadrant
        /// </summary>
        public AABB Q1 => new AABB(Center, UpperBound);

        /// <summary>
        /// Second quadrant
        /// </summary>
        public AABB Q2 => new AABB(new Vec2(LowerBound.X, Center.Y), new Vec2(Center.X, UpperBound.Y));

        /// <summary>
        /// Third quadrant
        /// </summary>
        public AABB Q3 => new AABB(LowerBound, Center);

        /// <summary>
        /// Forth quadrant
        /// </summary>
        public AABB Q4 => new AABB(new Vec2(Center.X, LowerBound.Y), new Vec2(UpperBound.X, Center.Y));

        /// <summary>
        /// Combine an AABB into this one.
        /// </summary>
        /// <param name="aabb">The AABB.</param>
        public void Combine(ref AABB aabb)
        {
            LowerBound = Vec2.Min(LowerBound, aabb.LowerBound);
            UpperBound = Vec2.Max(UpperBound, aabb.UpperBound);
        }

        /// <summary>
        /// Combine two AABBs into this one.
        /// </summary>
        /// <param name="aabb1">The aabb1.</param>
        /// <param name="aabb2">The aabb2.</param>
        public void Combine(ref AABB aabb1, ref AABB aabb2)
        {
            LowerBound = Vec2.Min(aabb1.LowerBound, aabb2.LowerBound);
            UpperBound = Vec2.Max(aabb1.UpperBound, aabb2.UpperBound);
        }

        /// <summary>
        /// Does this AABB contain the provided AABB.
        /// </summary>
        /// <param name="aabb">The AABB.</param>
        /// <returns>
        /// <c>true</c> if it contains the specified AABB; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref AABB aabb)
        {
            bool result = LowerBound.X <= aabb.LowerBound.X;
            result = result && LowerBound.Y <= aabb.LowerBound.Y;
            result = result && aabb.UpperBound.X <= UpperBound.X;
            result = result && aabb.UpperBound.Y <= UpperBound.Y;
            return result;
        }

        /// <summary>
        /// Determines whether the AABB contains the specified point.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <returns>
        /// <c>true</c> if it contains the specified point; otherwise, <c>false</c>.
        /// </returns>
        public bool Contains(ref Vec2 point)
        {
            //using epsilon to try and guard against Fix64 rounding errors.
            return (point.X > (LowerBound.X + Fix64.Epsilon) && point.X < (UpperBound.X - Fix64.Epsilon) &&
                    (point.Y > (LowerBound.Y + Fix64.Epsilon) && point.Y < (UpperBound.Y - Fix64.Epsilon)));
        }

        /// <summary>
        /// Test if the two AABBs overlap.
        /// </summary>
        /// <param name="a">The first AABB.</param>
        /// <param name="b">The second AABB.</param>
        /// <returns>True if they are overlapping.</returns>
        public static bool TestOverlap(ref AABB a, ref AABB b)
        {
            Vec2 d1 = b.LowerBound - a.UpperBound;
            Vec2 d2 = a.LowerBound - b.UpperBound;

            return (d1.X <= Fix64.Zero) && (d1.Y <= Fix64.Zero) && (d2.X <= Fix64.Zero) && (d2.Y <= Fix64.Zero);
        }

        /// <summary>
        /// Raycast against this AABB using the specified points and maxfraction (found in input)
        /// </summary>
        /// <param name="output">The results of the raycast.</param>
        /// <param name="input">The parameters for the raycast.</param>
        /// <returns>True if the ray intersects the AABB</returns>
        public bool RayCast(out RayCastOutput output, ref RayCastInput input, bool doInteriorCheck = true)
        {
            // From Real-time Collision Detection, p179.

            output = new RayCastOutput();

            Fix64 tmin = Fix64.MinValue;
            Fix64 tmax = Fix64.MaxValue;

            Vec2 p = input.Point1;
            Vec2 d = input.Point2 - input.Point1;
            Vec2 absD = MathUtils.Abs(d);

            Vec2 normal = Vec2.Zero;

            for (int i = 0; i < 2; ++i)
            {
                Fix64 absD_i = i == 0 ? absD.X : absD.Y;
                Fix64 lowerBound_i = i == 0 ? LowerBound.X : LowerBound.Y;
                Fix64 upperBound_i = i == 0 ? UpperBound.X : UpperBound.Y;
                Fix64 p_i = i == 0 ? p.X : p.Y;

                if (absD_i < Fix64.Epsilon)
                {
                    // Parallel.
                    if (p_i < lowerBound_i || upperBound_i < p_i)
                    {
                        return false;
                    }
                }
                else
                {
                    Fix64 d_i = i == 0 ? d.X : d.Y;

                    Fix64 inv_d = Fix64.One / d_i;
                    Fix64 t1 = (lowerBound_i - p_i) * inv_d;
                    Fix64 t2 = (upperBound_i - p_i) * inv_d;

                    // Sign of the normal vector.
                    Fix64 s = -Fix64.One;

                    if (t1 > t2)
                    {
                        MathUtils.Swap(ref t1, ref t2);
                        s = Fix64.One;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        if (i == 0)
                        {
                            normal.X = s;
                        }
                        else
                        {
                            normal.Y = s;
                        }

                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = Fix64.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (doInteriorCheck && (tmin < Fix64.Zero || input.MaxFraction < tmin))
            {
                return false;
            }

            // Intersection.
            output.Fraction = tmin;
            output.Normal = normal;
            return true;
        }
    }
}