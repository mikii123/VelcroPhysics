using System;
using System.Diagnostics;
using FixedMath.Net;
using VelcroPhysics.Collision.Distance;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
    internal struct Simplex
    {
        internal int Count;
        internal FixedArray3<SimplexVertex> V;

        internal void ReadCache(ref SimplexCache cache, ref DistanceProxy proxyA, ref Transform transformA, ref DistanceProxy proxyB, ref Transform transformB)
        {
            Debug.Assert(cache.Count <= 3);

            // Copy data from cache.
            Count = cache.Count;
            for (int i = 0; i < Count; ++i)
            {
                SimplexVertex v = V[i];
                v.IndexA = cache.IndexA[i];
                v.IndexB = cache.IndexB[i];
                Vec2 wALocal = proxyA.Vertices[v.IndexA];
                Vec2 wBLocal = proxyB.Vertices[v.IndexB];
                v.WA = MathUtils.Mul(ref transformA, wALocal);
                v.WB = MathUtils.Mul(ref transformB, wBLocal);
                v.W = v.WB - v.WA;
                v.A = Fix64.Zero;
                V[i] = v;
            }

            // Compute the new simplex metric, if it is substantially different than
            // old metric then flush the simplex.
            if (Count > 1)
            {
                Fix64 metric1 = cache.Metric;
                Fix64 metric2 = GetMetric();
                if (metric2 < Fix64.Half * metric1 || Fix64.Two * metric1 < metric2 || metric2 < Fix64.Epsilon)
                {
                    // Reset the simplex.
                    Count = 0;
                }
            }

            // If the cache is empty or invalid ...
            if (Count == 0)
            {
                SimplexVertex v = V[0];
                v.IndexA = 0;
                v.IndexB = 0;
                Vec2 wALocal = proxyA.Vertices[0];
                Vec2 wBLocal = proxyB.Vertices[0];
                v.WA = MathUtils.Mul(ref transformA, wALocal);
                v.WB = MathUtils.Mul(ref transformB, wBLocal);
                v.W = v.WB - v.WA;
                v.A = Fix64.One;
                V[0] = v;
                Count = 1;
            }
        }

        internal void WriteCache(ref SimplexCache cache)
        {
            cache.Metric = GetMetric();
            cache.Count = (ushort)Count;
            for (int i = 0; i < Count; ++i)
            {
                cache.IndexA[i] = (byte)V[i].IndexA;
                cache.IndexB[i] = (byte)V[i].IndexB;
            }
        }

        internal Vec2 GetSearchDirection()
        {
            switch (Count)
            {
                case 1:
                    return -V[0].W;

                case 2:
                    {
                        Vec2 e12 = V[1].W - V[0].W;
                        Fix64 sgn = MathUtils.Cross(e12, -V[0].W);
                        if (sgn > Fix64.Zero)
                        {
                            // Origin is left of e12.
                            return MathUtils.Cross(Fix64.One, e12);
                        }
                        else
                        {
                            // Origin is right of e12.
                            return MathUtils.Cross(e12, Fix64.One);
                        }
                    }

                default:
                    Debug.Assert(false);
                    return Vec2.Zero;
            }
        }

        internal Vec2 GetClosestPoint()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return Vec2.Zero;

                case 1:
                    return V[0].W;

                case 2:
                    return V[0].A * V[0].W + V[1].A * V[1].W;

                case 3:
                    return Vec2.Zero;

                default:
                    Debug.Assert(false);
                    return Vec2.Zero;
            }
        }

        internal void GetWitnessPoints(out Vec2 pA, out Vec2 pB)
        {
            switch (Count)
            {
                case 0:
                    pA = Vec2.Zero;
                    pB = Vec2.Zero;
                    Debug.Assert(false);
                    break;

                case 1:
                    pA = V[0].WA;
                    pB = V[0].WB;
                    break;

                case 2:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA;
                    pB = V[0].A * V[0].WB + V[1].A * V[1].WB;
                    break;

                case 3:
                    pA = V[0].A * V[0].WA + V[1].A * V[1].WA + V[2].A * V[2].WA;
                    pB = pA;
                    break;

                default:
                    throw new Exception();
            }
        }

        internal Fix64 GetMetric()
        {
            switch (Count)
            {
                case 0:
                    Debug.Assert(false);
                    return Fix64.Zero;
                case 1:
                    return Fix64.Zero;

                case 2:
                    return (V[0].W - V[1].W).Magnitude;

                case 3:
                    return MathUtils.Cross(V[1].W - V[0].W, V[2].W - V[0].W);

                default:
                    Debug.Assert(false);
                    return Fix64.Zero;
            }
        }

        // Solve a line segment using barycentric coordinates.
        //
        // p = a1 * w1 + a2 * w2
        // a1 + a2 = 1
        //
        // The vector from the origin to the closest point on the line is
        // perpendicular to the line.
        // e12 = w2 - w1
        // dot(p, e) = 0
        // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
        //
        // 2-by-2 linear system
        // [1      1     ][a1] = [1]
        // [w1.e12 w2.e12][a2] = [0]
        //
        // Define
        // d12_1 =  dot(w2, e12)
        // d12_2 = -dot(w1, e12)
        // d12 = d12_1 + d12_2
        //
        // Solution
        // a1 = d12_1 / d12
        // a2 = d12_2 / d12

        internal void Solve2()
        {
            Vec2 w1 = V[0].W;
            Vec2 w2 = V[1].W;
            Vec2 e12 = w2 - w1;

            // w1 region
            Fix64 d12_2 = -Vec2.Dot(w1, e12);
            if (d12_2 <= Fix64.Zero)
            {
                // a2 <= 0, so we clamp it to 0
                V.Value0.A = Fix64.One;
                Count = 1;
                return;
            }

            // w2 region
            Fix64 d12_1 = Vec2.Dot(w2, e12);
            if (d12_1 <= Fix64.Zero)
            {
                // a1 <= 0, so we clamp it to 0
                V.Value1.A = Fix64.One;
                Count = 1;
                V.Value0 = V.Value1;
                return;
            }

            // Must be in e12 region.
            Fix64 inv_d12 = Fix64.One / (d12_1 + d12_2);
            V.Value0.A = d12_1 * inv_d12;
            V.Value1.A = d12_2 * inv_d12;
            Count = 2;
        }

        // Possible regions:
        // - points[2]
        // - edge points[0]-points[2]
        // - edge points[1]-points[2]
        // - inside the triangle
        internal void Solve3()
        {
            Vec2 w1 = V[0].W;
            Vec2 w2 = V[1].W;
            Vec2 w3 = V[2].W;

            // Edge12
            // [1      1     ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            Vec2 e12 = w2 - w1;
            Fix64 w1e12 = Vec2.Dot(w1, e12);
            Fix64 w2e12 = Vec2.Dot(w2, e12);
            Fix64 d12_1 = w2e12;
            Fix64 d12_2 = -w1e12;

            // Edge13
            // [1      1     ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            Vec2 e13 = w3 - w1;
            Fix64 w1e13 = Vec2.Dot(w1, e13);
            Fix64 w3e13 = Vec2.Dot(w3, e13);
            Fix64 d13_1 = w3e13;
            Fix64 d13_2 = -w1e13;

            // Edge23
            // [1      1     ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            Vec2 e23 = w3 - w2;
            Fix64 w2e23 = Vec2.Dot(w2, e23);
            Fix64 w3e23 = Vec2.Dot(w3, e23);
            Fix64 d23_1 = w3e23;
            Fix64 d23_2 = -w2e23;

            // Triangle123
            Fix64 n123 = MathUtils.Cross(e12, e13);

            Fix64 d123_1 = n123 * MathUtils.Cross(w2, w3);
            Fix64 d123_2 = n123 * MathUtils.Cross(w3, w1);
            Fix64 d123_3 = n123 * MathUtils.Cross(w1, w2);

            // w1 region
            if (d12_2 <= Fix64.Zero && d13_2 <= Fix64.Zero)
            {
                V.Value0.A = Fix64.One;
                Count = 1;
                return;
            }

            // e12
            if (d12_1 > Fix64.Zero && d12_2 > Fix64.Zero && d123_3 <= Fix64.Zero)
            {
                Fix64 inv_d12 = Fix64.One / (d12_1 + d12_2);
                V.Value0.A = d12_1 * inv_d12;
                V.Value1.A = d12_2 * inv_d12;
                Count = 2;
                return;
            }

            // e13
            if (d13_1 > Fix64.Zero && d13_2 > Fix64.Zero && d123_2 <= Fix64.Zero)
            {
                Fix64 inv_d13 = Fix64.One / (d13_1 + d13_2);
                V.Value0.A = d13_1 * inv_d13;
                V.Value2.A = d13_2 * inv_d13;
                Count = 2;
                V.Value1 = V.Value2;
                return;
            }

            // w2 region
            if (d12_1 <= Fix64.Zero && d23_2 <= Fix64.Zero)
            {
                V.Value1.A = Fix64.One;
                Count = 1;
                V.Value0 = V.Value1;
                return;
            }

            // w3 region
            if (d13_1 <= Fix64.Zero && d23_1 <= Fix64.Zero)
            {
                V.Value2.A = Fix64.One;
                Count = 1;
                V.Value0 = V.Value2;
                return;
            }

            // e23
            if (d23_1 > Fix64.Zero && d23_2 > Fix64.Zero && d123_1 <= Fix64.Zero)
            {
                Fix64 inv_d23 = Fix64.One / (d23_1 + d23_2);
                V.Value1.A = d23_1 * inv_d23;
                V.Value2.A = d23_2 * inv_d23;
                Count = 2;
                V.Value0 = V.Value2;
                return;
            }

            // Must be in triangle123
            Fix64 inv_d123 = Fix64.One / (d123_1 + d123_2 + d123_3);
            V.Value0.A = d123_1 * inv_d123;
            V.Value1.A = d123_2 * inv_d123;
            V.Value2.A = d123_3 * inv_d123;
            Count = 3;
        }
    }
}