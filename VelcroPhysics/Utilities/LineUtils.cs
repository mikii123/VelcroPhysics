using FixedMath.Net;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Utilities
{
    /// <summary>
    /// Collection of helper methods for misc collisions.
    /// Does Fix64 tolerance and line collisions with lines and AABBs.
    /// </summary>
    public static class LineUtils
    {
        public static Fix64 DistanceBetweenPointAndLineSegment(ref Vec2 point, ref Vec2 start, ref Vec2 end)
        {
            if (start == end)
                return Vec2.Distance(point, start);

            Vec2 v = Vec2.Subtract(end, start);
            Vec2 w = Vec2.Subtract(point, start);

            Fix64 c1 = Vec2.Dot(w, v);
            if (c1 <= Fix64.Zero)
                return Vec2.Distance(point, start);

            Fix64 c2 = Vec2.Dot(v, v);
            if (c2 <= c1)
                return Vec2.Distance(point, end);

            Fix64 b = c1 / c2;
            Vec2 pointOnLine = Vec2.Add(start, Vec2.Multiply(v, b));
            return Vec2.Distance(point, pointOnLine);
        }

        // From Eric Jordan's convex decomposition library
        /// <summary>
        /// Check if the lines a0->a1 and b0->b1 cross.
        /// If they do, intersectionPoint will be filled
        /// with the point of crossing.
        /// Grazing lines should not return true.
        /// </summary>
        public static bool LineIntersect2(ref Vec2 a0, ref Vec2 a1, ref Vec2 b0, ref Vec2 b1, out Vec2 intersectionPoint)
        {
            intersectionPoint = Vec2.Zero;

            if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1)
                return false;

            Fix64 x1 = a0.X;
            Fix64 y1 = a0.Y;
            Fix64 x2 = a1.X;
            Fix64 y2 = a1.Y;
            Fix64 x3 = b0.X;
            Fix64 y3 = b0.Y;
            Fix64 x4 = b1.X;
            Fix64 y4 = b1.Y;

            //AABB early exit
            if (Fix64.Max(x1, x2) < Fix64.Min(x3, x4) || Fix64.Max(x3, x4) < Fix64.Min(x1, x2))
                return false;

            if (Fix64.Max(y1, y2) < Fix64.Min(y3, y4) || Fix64.Max(y3, y4) < Fix64.Min(y1, y2))
                return false;

            Fix64 ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3));
            Fix64 ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3));
            Fix64 denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
            if (Fix64.Abs(denom) < Fix64.Epsilon)
            {
                //Lines are too close to parallel to call
                return false;
            }
            ua /= denom;
            ub /= denom;

            if ((Fix64.Zero < ua) && (ua < Fix64.One) && (Fix64.Zero < ub) && (ub < Fix64.One))
            {
                intersectionPoint.X = (x1 + ua * (x2 - x1));
                intersectionPoint.Y = (y1 + ua * (y2 - y1));
                return true;
            }

            return false;
        }

        //From Mark Bayazit's convex decomposition algorithm
        public static Vec2 LineIntersect(Vec2 p1, Vec2 p2, Vec2 q1, Vec2 q2)
        {
            Vec2 i = Vec2.Zero;
            Fix64 a1 = p2.Y - p1.Y;
            Fix64 b1 = p1.X - p2.X;
            Fix64 c1 = a1 * p1.X + b1 * p1.Y;
            Fix64 a2 = q2.Y - q1.Y;
            Fix64 b2 = q1.X - q2.X;
            Fix64 c2 = a2 * q1.X + b2 * q1.Y;
            Fix64 det = a1 * b2 - a2 * b1;

            if (!MathUtils.FloatEquals(det, Fix64.Zero))
            {
                // lines are not parallel
                i.X = (b2 * c1 - b1 * c2) / det;
                i.Y = (a1 * c2 - a2 * c1) / det;
            }
            return i;
        }

        /// <summary>
        /// This method detects if two line segments (or lines) intersect,
        /// and, if so, the point of intersection. Use the <paramref name="firstIsSegment" /> and
        /// <paramref name="secondIsSegment" /> parameters to set whether the intersection point
        /// must be on the first and second line segments. Setting these
        /// both to true means you are doing a line-segment to line-segment
        /// intersection. Setting one of them to true means you are doing a
        /// line to line-segment intersection test, and so on.
        /// Note: If two line segments are coincident, then
        /// no intersection is detected (there are actually
        /// infinite intersection points).
        /// Author: Jeremy Bell
        /// </summary>
        /// <param name="point1">The first point of the first line segment.</param>
        /// <param name="point2">The second point of the first line segment.</param>
        /// <param name="point3">The first point of the second line segment.</param>
        /// <param name="point4">The second point of the second line segment.</param>
        /// <param name="point">
        /// This is set to the intersection
        /// point if an intersection is detected.
        /// </param>
        /// <param name="firstIsSegment">
        /// Set this to true to require that the
        /// intersection point be on the first line segment.
        /// </param>
        /// <param name="secondIsSegment">
        /// Set this to true to require that the
        /// intersection point be on the second line segment.
        /// </param>
        /// <returns>True if an intersection is detected, false otherwise.</returns>
        public static bool LineIntersect(ref Vec2 point1, ref Vec2 point2, ref Vec2 point3, ref Vec2 point4, bool firstIsSegment, bool secondIsSegment, out Vec2 point)
        {
            point = new Vec2();

            // these are reused later.
            // each lettered sub-calculation is used twice, except
            // for b and d, which are used 3 times
            Fix64 a = point4.Y - point3.Y;
            Fix64 b = point2.X - point1.X;
            Fix64 c = point4.X - point3.X;
            Fix64 d = point2.Y - point1.Y;

            // denominator to solution of linear system
            Fix64 denom = (a * b) - (c * d);

            // if denominator is 0, then lines are parallel
            if (!(denom >= -Fix64.Epsilon && denom <= Fix64.Epsilon))
            {
                Fix64 e = point1.Y - point3.Y;
                Fix64 f = point1.X - point3.X;
                Fix64 oneOverDenom = Fix64.One / denom;

                // numerator of first equation
                Fix64 ua = (c * e) - (a * f);
                ua *= oneOverDenom;

                // check if intersection point of the two lines is on line segment 1
                if (!firstIsSegment || ua >= Fix64.Zero && ua <= Fix64.One)
                {
                    // numerator of second equation
                    Fix64 ub = (b * e) - (d * f);
                    ub *= oneOverDenom;

                    // check if intersection point of the two lines is on line segment 2
                    // means the line segments intersect, since we know it is on
                    // segment 1 as well.
                    if (!secondIsSegment || ub >= Fix64.Zero && ub <= Fix64.One)
                    {
                        // check if they are coincident (no collision in this case)
                        if (ua != Fix64.Zero || ub != Fix64.Zero)
                        {
                            //There is an intersection
                            point.X = point1.X + ua * b;
                            point.Y = point1.Y + ua * d;
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// This method detects if two line segments (or lines) intersect,
        /// and, if so, the point of intersection. Use the <paramref name="firstIsSegment" /> and
        /// <paramref name="secondIsSegment" /> parameters to set whether the intersection point
        /// must be on the first and second line segments. Setting these
        /// both to true means you are doing a line-segment to line-segment
        /// intersection. Setting one of them to true means you are doing a
        /// line to line-segment intersection test, and so on.
        /// Note: If two line segments are coincident, then
        /// no intersection is detected (there are actually
        /// infinite intersection points).
        /// Author: Jeremy Bell
        /// </summary>
        /// <param name="point1">The first point of the first line segment.</param>
        /// <param name="point2">The second point of the first line segment.</param>
        /// <param name="point3">The first point of the second line segment.</param>
        /// <param name="point4">The second point of the second line segment.</param>
        /// <param name="intersectionPoint">
        /// This is set to the intersection
        /// point if an intersection is detected.
        /// </param>
        /// <param name="firstIsSegment">
        /// Set this to true to require that the
        /// intersection point be on the first line segment.
        /// </param>
        /// <param name="secondIsSegment">
        /// Set this to true to require that the
        /// intersection point be on the second line segment.
        /// </param>
        /// <returns>True if an intersection is detected, false otherwise.</returns>
        public static bool LineIntersect(Vec2 point1, Vec2 point2, Vec2 point3, Vec2 point4, bool firstIsSegment, bool secondIsSegment, out Vec2 intersectionPoint)
        {
            return LineIntersect(ref point1, ref point2, ref point3, ref point4, firstIsSegment, secondIsSegment, out intersectionPoint);
        }

        /// <summary>
        /// This method detects if two line segments intersect,
        /// and, if so, the point of intersection.
        /// Note: If two line segments are coincident, then
        /// no intersection is detected (there are actually
        /// infinite intersection points).
        /// </summary>
        /// <param name="point1">The first point of the first line segment.</param>
        /// <param name="point2">The second point of the first line segment.</param>
        /// <param name="point3">The first point of the second line segment.</param>
        /// <param name="point4">The second point of the second line segment.</param>
        /// <param name="intersectionPoint">
        /// This is set to the intersection
        /// point if an intersection is detected.
        /// </param>
        /// <returns>True if an intersection is detected, false otherwise.</returns>
        public static bool LineIntersect(ref Vec2 point1, ref Vec2 point2, ref Vec2 point3, ref Vec2 point4, out Vec2 intersectionPoint)
        {
            return LineIntersect(ref point1, ref point2, ref point3, ref point4, true, true, out intersectionPoint);
        }

        /// <summary>
        /// This method detects if two line segments intersect,
        /// and, if so, the point of intersection.
        /// Note: If two line segments are coincident, then
        /// no intersection is detected (there are actually
        /// infinite intersection points).
        /// </summary>
        /// <param name="point1">The first point of the first line segment.</param>
        /// <param name="point2">The second point of the first line segment.</param>
        /// <param name="point3">The first point of the second line segment.</param>
        /// <param name="point4">The second point of the second line segment.</param>
        /// <param name="intersectionPoint">
        /// This is set to the intersection
        /// point if an intersection is detected.
        /// </param>
        /// <returns>True if an intersection is detected, false otherwise.</returns>
        public static bool LineIntersect(Vec2 point1, Vec2 point2, Vec2 point3, Vec2 point4, out Vec2 intersectionPoint)
        {
            return LineIntersect(ref point1, ref point2, ref point3, ref point4, true, true, out intersectionPoint);
        }

        /// <summary>
        /// Get all intersections between a line segment and a list of vertices
        /// representing a polygon. The vertices reuse adjacent points, so for example
        /// edges one and two are between the first and second vertices and between the
        /// second and third vertices. The last edge is between vertex vertices.Count - 1
        /// and verts0. (ie, vertices from a Geometry or AABB)
        /// </summary>
        /// <param name="point1">The first point of the line segment to test</param>
        /// <param name="point2">The second point of the line segment to test.</param>
        /// <param name="vertices">The vertices, as described above</param>
        public static Vertices LineSegmentVerticesIntersect(ref Vec2 point1, ref Vec2 point2, Vertices vertices)
        {
            Vertices intersectionPoints = new Vertices();

            for (int i = 0; i < vertices.Count; i++)
            {
                Vec2 point;
                if (LineIntersect(vertices[i], vertices[vertices.NextIndex(i)], point1, point2, true, true, out point))
                {
                    intersectionPoints.Add(point);
                }
            }

            return intersectionPoints;
        }

        /// <summary>
        /// Get all intersections between a line segment and an AABB.
        /// </summary>
        /// <param name="point1">The first point of the line segment to test</param>
        /// <param name="point2">The second point of the line segment to test.</param>
        /// <param name="aabb">The AABB that is used for testing intersection.</param>
        public static Vertices LineSegmentAABBIntersect(ref Vec2 point1, ref Vec2 point2, AABB aabb)
        {
            return LineSegmentVerticesIntersect(ref point1, ref point2, aabb.Vertices);
        }
    }
}