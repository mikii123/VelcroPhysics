using System;
using System.Collections.Generic;
using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.PolygonManipulation
{
    /// <summary>
    /// Provides a set of tools to simplify polygons in various ways.
    /// </summary>
    public static class SimplifyTools
    {
        /// <summary>
        /// Removes all collinear points on the polygon.
        /// </summary>
        /// <param name="vertices">The polygon that needs simplification.</param>
        /// <param name="collinearityTolerance">The collinearity tolerance.</param>
        /// <returns>A simplified polygon.</returns>
        public static Vertices CollinearSimplify(Vertices vertices, Fix64 collinearityTolerance = new Fix64())
        {
            if (vertices.Count <= 3)
                return vertices;

            Vertices simplified = new Vertices(vertices.Count);

            for (int i = 0; i < vertices.Count; i++)
            {
                Vec2 prev = vertices.PreviousVertex(i);
                Vec2 current = vertices[i];
                Vec2 next = vertices.NextVertex(i);

                //If they collinear, continue
                if (MathUtils.IsCollinear(ref prev, ref current, ref next, collinearityTolerance))
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Ramer-Douglas-Peucker polygon simplification algorithm. This is the general recursive version that does not use the
        /// speed-up technique by using the Melkman convex hull.
        /// If you pass in 0, it will remove all collinear points.
        /// </summary>
        /// <returns>The simplified polygon</returns>
        public static Vertices DouglasPeuckerSimplify(Vertices vertices, Fix64 distanceTolerance)
        {
            if (vertices.Count <= 3)
                return vertices;

            bool[] usePoint = new bool[vertices.Count];

            for (int i = 0; i < vertices.Count; i++)
                usePoint[i] = true;

            SimplifySection(vertices, 0, vertices.Count - 1, usePoint, distanceTolerance);

            Vertices simplified = new Vertices(vertices.Count);

            for (int i = 0; i < vertices.Count; i++)
            {
                if (usePoint[i])
                    simplified.Add(vertices[i]);
            }

            return simplified;
        }

        private static void SimplifySection(Vertices vertices, int i, int j, bool[] usePoint, Fix64 distanceTolerance)
        {
            if ((i + 1) == j)
                return;

            Vec2 a = vertices[i];
            Vec2 b = vertices[j];

            Fix64 maxDistance = -Fix64.One;
            int maxIndex = i;
            for (int k = i + 1; k < j; k++)
            {
                Vec2 point = vertices[k];

                Fix64 distance = LineUtils.DistanceBetweenPointAndLineSegment(ref point, ref a, ref b);

                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxIndex = k;
                }
            }

            if (maxDistance <= distanceTolerance)
            {
                for (int k = i + 1; k < j; k++)
                {
                    usePoint[k] = false;
                }
            }
            else
            {
                SimplifySection(vertices, i, maxIndex, usePoint, distanceTolerance);
                SimplifySection(vertices, maxIndex, j, usePoint, distanceTolerance);
            }
        }

        /// <summary>
        /// Merges all parallel edges in the list of vertices
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="tolerance">The tolerance.</param>
        public static Vertices MergeParallelEdges(Vertices vertices, Fix64 tolerance)
        {
            //From Eric Jordan's convex decomposition library

            if (vertices.Count <= 3)
                return vertices; //Can't do anything useful here to a triangle

            bool[] mergeMe = new bool[vertices.Count];
            int newNVertices = vertices.Count;

            //Gather points to process
            for (int i = 0; i < vertices.Count; ++i)
            {
                int lower = i == 0 ? vertices.Count - 1 : i - 1;
                int middle = i;
                int upper = i == vertices.Count - 1 ? 0 : i + 1;

                Fix64 dx0 = vertices[middle].X - vertices[lower].X;
                Fix64 dy0 = vertices[middle].Y - vertices[lower].Y;
                Fix64 dx1 = vertices[upper].X - vertices[middle].X;
                Fix64 dy1 = vertices[upper].Y - vertices[middle].Y;
                Fix64 norm0 = Fix64.Sqrt(dx0 * dx0 + dy0 * dy0);
                Fix64 norm1 = Fix64.Sqrt(dx1 * dx1 + dy1 * dy1);

                if (!(norm0 > Fix64.Zero && norm1 > Fix64.Zero) && newNVertices > 3)
                {
                    //Merge identical points
                    mergeMe[i] = true;
                    --newNVertices;
                }

                dx0 /= norm0;
                dy0 /= norm0;
                dx1 /= norm1;
                dy1 /= norm1;
                Fix64 cross = dx0 * dy1 - dx1 * dy0;
                Fix64 dot = dx0 * dx1 + dy0 * dy1;

                if (Fix64.Abs(cross) < tolerance && dot > Fix64.Zero && newNVertices > 3)
                {
                    mergeMe[i] = true;
                    --newNVertices;
                }
                else
                    mergeMe[i] = false;
            }

            if (newNVertices == vertices.Count || newNVertices == 0)
                return vertices;

            int currIndex = 0;

            //Copy the vertices to a new list and clear the old
            Vertices newVertices = new Vertices(newNVertices);

            for (int i = 0; i < vertices.Count; ++i)
            {
                if (mergeMe[i] || currIndex == newNVertices)
                    continue;

                Debug.Assert(currIndex < newNVertices);

                newVertices.Add(vertices[i]);
                ++currIndex;
            }

            return newVertices;
        }

        /// <summary>
        /// Merges the identical points in the polygon.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        public static Vertices MergeIdenticalPoints(Vertices vertices)
        {
            HashSet<Vec2> unique = new HashSet<Vec2>();

            foreach (Vec2 vertex in vertices)
            {
                unique.Add(vertex);
            }

            return new Vertices(unique);
        }

        /// <summary>
        /// Reduces the polygon by distance.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="distance">The distance between points. Points closer than this will be removed.</param>
        public static Vertices ReduceByDistance(Vertices vertices, Fix64 distance)
        {
            if (vertices.Count <= 3)
                return vertices;

            Fix64 distance2 = distance * distance;

            Vertices simplified = new Vertices(vertices.Count);

            for (int i = 0; i < vertices.Count; i++)
            {
                Vec2 current = vertices[i];
                Vec2 next = vertices.NextVertex(i);

                //If they are closer than the distance, continue
                if ((next - current).MagnitudeSquared <= distance2)
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Reduces the polygon by removing the Nth vertex in the vertices list.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="nth">The Nth point to remove. Example: 5.</param>
        /// <returns></returns>
        public static Vertices ReduceByNth(Vertices vertices, int nth)
        {
            if (vertices.Count <= 3)
                return vertices;

            if (nth == 0)
                return vertices;

            Vertices simplified = new Vertices(vertices.Count);

            for (int i = 0; i < vertices.Count; i++)
            {
                if (i % nth == 0)
                    continue;

                simplified.Add(vertices[i]);
            }

            return simplified;
        }

        /// <summary>
        /// Simplify the polygon by removing all points that in pairs of 3 have an area less than the tolerance.
        /// Pass in 0 as tolerance, and it will only remove collinear points.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="areaTolerance"></param>
        /// <returns></returns>
        public static Vertices ReduceByArea(Vertices vertices, Fix64 areaTolerance)
        {
            //From physics2d.net

            if (vertices.Count <= 3)
                return vertices;

            if (areaTolerance < Fix64.Zero)
                throw new ArgumentOutOfRangeException(nameof(areaTolerance), "must be equal to or greater than zero.");

            Vertices simplified = new Vertices(vertices.Count);
            Vec2 v3;
            Vec2 v1 = vertices[vertices.Count - 2];
            Vec2 v2 = vertices[vertices.Count - 1];
            areaTolerance *= Fix64.Two;

            for (int i = 0; i < vertices.Count; ++i, v2 = v3)
            {
                v3 = i == vertices.Count - 1 ? simplified[0] : vertices[i];

                MathUtils.Cross(ref v1, ref v2, out Fix64 old1);
                MathUtils.Cross(ref v2, ref v3, out Fix64 old2);
                MathUtils.Cross(ref v1, ref v3, out Fix64 new1);

                if (Fix64.Abs(new1 - (old1 + old2)) > areaTolerance)
                {
                    simplified.Add(v2);
                    v1 = v2;
                }
            }

            return simplified;
        }
    }
}