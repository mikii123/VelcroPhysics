﻿using System.Collections.Generic;
using System.Diagnostics;
using FixedMath.Net;
using Microsoft.Xna.Framework;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.Triangulation.Bayazit
{
    //From phed rev 36: http://code.google.com/p/phed/source/browse/trunk/Polygon.cpp

    /// <summary>
    /// Convex decomposition algorithm created by Mark Bayazit (http://mnbayazit.com/)
    /// 
    /// Properties:
    /// - Tries to decompose using polygons instead of triangles.
    /// - Tends to produce optimal results with low processing time.
    /// - Running time is O(nr), n = number of vertices, r = reflex vertices.
    /// - Does not support holes.
    /// 
    /// For more information about this algorithm, see http://mnbayazit.com/406/bayazit
    /// </summary>
    internal static class BayazitDecomposer
    {
        /// <summary>
        /// Decompose the polygon into several smaller non-concave polygon.
        /// If the polygon is already convex, it will return the original polygon, unless it is over Settings.MaxPolygonVertices.
        /// </summary>
        public static List<Vertices> ConvexPartition(Vertices vertices)
        {
            Debug.Assert(vertices.Count > 3);
            Debug.Assert(vertices.IsCounterClockWise());

            return TriangulatePolygon(vertices);
        }

        private static List<Vertices> TriangulatePolygon(Vertices vertices)
        {
            List<Vertices> list = new List<Vertices>();
            Vec2 lowerInt = new Vec2();
            Vec2 upperInt = new Vec2(); // intersection points
            int lowerIndex = 0, upperIndex = 0;
            Vertices lowerPoly, upperPoly;

            for (int i = 0; i < vertices.Count; ++i)
            {
                if (Reflex(i, vertices))
                {
                    Fix64 upperDist;
                    Fix64 lowerDist = upperDist = Fix64.MaxValue;
                    for (int j = 0; j < vertices.Count; ++j)
                    {
                        // if line intersects with an edge
                        Fix64 d;
                        Vec2 p;
                        if (Left(At(i - 1, vertices), At(i, vertices), At(j, vertices)) && RightOn(At(i - 1, vertices), At(i, vertices), At(j - 1, vertices)))
                        {
                            // find the point of intersection
                            p = LineUtils.LineIntersect(At(i - 1, vertices), At(i, vertices), At(j, vertices), At(j - 1, vertices));

                            if (Right(At(i + 1, vertices), At(i, vertices), p))
                            {
                                // make sure it's inside the poly
                                d = SquareDist(At(i, vertices), p);
                                if (d < lowerDist)
                                {
                                    // keep only the closest intersection
                                    lowerDist = d;
                                    lowerInt = p;
                                    lowerIndex = j;
                                }
                            }
                        }

                        if (Left(At(i + 1, vertices), At(i, vertices), At(j + 1, vertices)) && RightOn(At(i + 1, vertices), At(i, vertices), At(j, vertices)))
                        {
                            p = LineUtils.LineIntersect(At(i + 1, vertices), At(i, vertices), At(j, vertices), At(j + 1, vertices));

                            if (Left(At(i - 1, vertices), At(i, vertices), p))
                            {
                                d = SquareDist(At(i, vertices), p);
                                if (d < upperDist)
                                {
                                    upperDist = d;
                                    upperIndex = j;
                                    upperInt = p;
                                }
                            }
                        }
                    }

                    // if there are no vertices to connect to, choose a point in the middle
                    if (lowerIndex == (upperIndex + 1) % vertices.Count)
                    {
                        Vec2 p = ((lowerInt + upperInt) / 2);

                        lowerPoly = Copy(i, upperIndex, vertices);
                        lowerPoly.Add(p);
                        upperPoly = Copy(lowerIndex, i, vertices);
                        upperPoly.Add(p);
                    }
                    else
                    {
                        Fix64 highestScore = Fix64.Zero, bestIndex = new Fix64(lowerIndex);
                        while (upperIndex < lowerIndex)
                            upperIndex += vertices.Count;

                        for (int j = lowerIndex; j <= upperIndex; ++j)
                        {
                            if (CanSee(i, j, vertices))
                            {
                                Fix64 score = 1 / (SquareDist(At(i, vertices), At(j, vertices)) + 1);
                                if (Reflex(j, vertices))
                                {
                                    if (RightOn(At(j - 1, vertices), At(j, vertices), At(i, vertices)) && LeftOn(At(j + 1, vertices), At(j, vertices), At(i, vertices)))
                                        score += 3;
                                    else
                                        score += 2;
                                }
                                else
                                {
                                    score += 1;
                                }
                                if (score > highestScore)
                                {
                                    bestIndex = new Fix64(j);
                                    highestScore = score;
                                }
                            }
                        }
                        lowerPoly = Copy(i, (int)bestIndex, vertices);
                        upperPoly = Copy((int)bestIndex, i, vertices);
                    }
                    list.AddRange(TriangulatePolygon(lowerPoly));
                    list.AddRange(TriangulatePolygon(upperPoly));
                    return list;
                }
            }

            // polygon is already convex
            if (vertices.Count > Settings.MaxPolygonVertices)
            {
                lowerPoly = Copy(0, vertices.Count / 2, vertices);
                upperPoly = Copy(vertices.Count / 2, 0, vertices);
                list.AddRange(TriangulatePolygon(lowerPoly));
                list.AddRange(TriangulatePolygon(upperPoly));
            }
            else
                list.Add(vertices);

            return list;
        }

        private static Vec2 At(int i, Vertices vertices)
        {
            int s = vertices.Count;
            return vertices[i < 0 ? s - 1 - ((-i - 1) % s) : i % s];
        }

        private static Vertices Copy(int i, int j, Vertices vertices)
        {
            while (j < i)
                j += vertices.Count;

            Vertices p = new Vertices(j);

            for (; i <= j; ++i)
            {
                p.Add(At(i, vertices));
            }
            return p;
        }

        private static bool CanSee(int i, int j, Vertices vertices)
        {
            if (Reflex(i, vertices))
            {
                if (LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)) && RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)))
                    return false;
            }
            else
            {
                if (RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)) || LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)))
                    return false;
            }
            if (Reflex(j, vertices))
            {
                if (LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)) && RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)))
                    return false;
            }
            else
            {
                if (RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)) || LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)))
                    return false;
            }
            for (int k = 0; k < vertices.Count; ++k)
            {
                if ((k + 1) % vertices.Count == i || k == i || (k + 1) % vertices.Count == j || k == j)
                    continue; // ignore incident edges

                Vec2 intersectionPoint;

                if (LineUtils.LineIntersect(At(i, vertices), At(j, vertices), At(k, vertices), At(k + 1, vertices), out intersectionPoint))
                    return false;
            }
            return true;
        }

        private static bool Reflex(int i, Vertices vertices)
        {
            return Right(i, vertices);
        }

        private static bool Right(int i, Vertices vertices)
        {
            return Right(At(i - 1, vertices), At(i, vertices), At(i + 1, vertices));
        }

        private static bool Left(Vec2 a, Vec2 b, Vec2 c)
        {
            return MathUtils.Area(ref a, ref b, ref c) > 0;
        }

        private static bool LeftOn(Vec2 a, Vec2 b, Vec2 c)
        {
            return MathUtils.Area(ref a, ref b, ref c) >= 0;
        }

        private static bool Right(Vec2 a, Vec2 b, Vec2 c)
        {
            return MathUtils.Area(ref a, ref b, ref c) < 0;
        }

        private static bool RightOn(Vec2 a, Vec2 b, Vec2 c)
        {
            return MathUtils.Area(ref a, ref b, ref c) <= 0;
        }

        private static Fix64 SquareDist(Vec2 a, Vec2 b)
        {
            Fix64 dx = b.X - a.X;
            Fix64 dy = b.Y - a.Y;
            return dx * dx + dy * dy;
        }
    }
}