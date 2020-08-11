/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using System;
using System.Runtime.InteropServices;
using FixedMath.Net;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Utilities
{
    public static class MathUtils
    {
        public static Fix64 Cross(ref Vec2 a, ref Vec2 b)
        {
            return a.X * b.Y - a.Y * b.X;
        }

        public static Fix64 Cross(Vec2 a, Vec2 b)
        {
            return Cross(ref a, ref b);
        }

        /// Perform the cross product on two vectors.
        public static Vec3 Cross(Vec3 a, Vec3 b)
        {
            return new Vec3(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        }

        public static Vec2 Cross(Vec2 a, Fix64 s)
        {
            return new Vec2(s * a.Y, -s * a.X);
        }

        public static Vec2 Cross(Fix64 s, Vec2 a)
        {
            return new Vec2(-s * a.Y, s * a.X);
        }

        public static Vec2 Abs(Vec2 v)
        {
            return new Vec2(Fix64.Abs(v.X), Fix64.Abs(v.Y));
        }

        public static Vec2 Mul(ref Mat22 A, Vec2 v)
        {
            return Mul(ref A, ref v);
        }

        public static Vec2 Mul(ref Mat22 A, ref Vec2 v)
        {
            return new Vec2(A.ex.X * v.X + A.ey.X * v.Y, A.ex.Y * v.X + A.ey.Y * v.Y);
        }

        public static Vec2 Mul(ref Transform T, Vec2 v)
        {
            return Mul(ref T, ref v);
        }

        public static Vec2 Mul(ref Transform T, ref Vec2 v)
        {
            Fix64 x = (T.q.c * v.X - T.q.s * v.Y) + T.p.X;
            Fix64 y = (T.q.s * v.X + T.q.c * v.Y) + T.p.Y;

            return new Vec2(x, y);
        }

        public static Vec2 MulT(ref Mat22 A, Vec2 v)
        {
            return MulT(ref A, ref v);
        }

        public static Vec2 MulT(ref Mat22 A, ref Vec2 v)
        {
            return new Vec2(v.X * A.ex.X + v.Y * A.ex.Y, v.X * A.ey.X + v.Y * A.ey.Y);
        }

        public static Vec2 MulT(ref Transform T, Vec2 v)
        {
            return MulT(ref T, ref v);
        }

        public static Vec2 MulT(ref Transform T, ref Vec2 v)
        {
            Fix64 px = v.X - T.p.X;
            Fix64 py = v.Y - T.p.Y;
            Fix64 x = (T.q.c * px + T.q.s * py);
            Fix64 y = (-T.q.s * px + T.q.c * py);

            return new Vec2(x, y);
        }

        // A^T * B
        public static void MulT(ref Mat22 A, ref Mat22 B, out Mat22 C)
        {
            C = new Mat22();
            C.ex.X = A.ex.X * B.ex.X + A.ex.Y * B.ex.Y;
            C.ex.Y = A.ey.X * B.ex.X + A.ey.Y * B.ex.Y;
            C.ey.X = A.ex.X * B.ey.X + A.ex.Y * B.ey.Y;
            C.ey.Y = A.ey.X * B.ey.X + A.ey.Y * B.ey.Y;
        }

        /// Multiply a matrix times a vector.
        public static Vec3 Mul(Mat33 A, Vec3 v)
        {
            return v.X * A.ex + v.Y * A.ey + v.Z * A.ez;
        }

        // v2 = A.q.Rot(B.q.Rot(v1) + B.p) + A.p
        //    = (A.q * B.q).Rot(v1) + A.q.Rot(B.p) + A.p
        public static Transform Mul(Transform A, Transform B)
        {
            Transform C = new Transform();
            C.q = Mul(A.q, B.q);
            C.p = Mul(A.q, B.p) + A.p;
            return C;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        public static void MulT(ref Transform A, ref Transform B, out Transform C)
        {
            C = new Transform();
            C.q = MulT(A.q, B.q);
            C.p = MulT(A.q, B.p - A.p);
        }

        public static void Swap<T>(ref T a, ref T b)
        {
            T tmp = a;
            a = b;
            b = tmp;
        }

        /// Multiply a matrix times a vector.
        public static Vec2 Mul22(Mat33 A, Vec2 v)
        {
            return new Vec2(A.ex.X * v.X + A.ey.X * v.Y, A.ex.Y * v.X + A.ey.Y * v.Y);
        }

        /// Multiply two rotations: q * r
        public static Rot Mul(Rot q, Rot r)
        {
            // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
            // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
            // s = qs * rc + qc * rs
            // c = qc * rc - qs * rs
            Rot qr;
            qr.s = q.s * r.c + q.c * r.s;
            qr.c = q.c * r.c - q.s * r.s;
            return qr;
        }

        public static Vec2 MulT(Transform T, Vec2 v)
        {
            Fix64 px = v.X - T.p.X;
            Fix64 py = v.Y - T.p.Y;
            Fix64 x = (T.q.c * px + T.q.s * py);
            Fix64 y = (-T.q.s * px + T.q.c * py);

            return new Vec2(x, y);
        }

        /// Transpose multiply two rotations: qT * r
        public static Rot MulT(Rot q, Rot r)
        {
            // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
            // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
            // s = qc * rs - qs * rc
            // c = qc * rc + qs * rs
            Rot qr;
            qr.s = q.c * r.s - q.s * r.c;
            qr.c = q.c * r.c + q.s * r.s;
            return qr;
        }

        // v2 = A.q' * (B.q * v1 + B.p - A.p)
        //    = A.q' * B.q * v1 + A.q' * (B.p - A.p)
        public static Transform MulT(Transform A, Transform B)
        {
            Transform C = new Transform();
            C.q = MulT(A.q, B.q);
            C.p = MulT(A.q, B.p - A.p);
            return C;
        }

        /// Rotate a vector
        public static Vec2 Mul(Rot q, Vec2 v)
        {
            return new Vec2(q.c * v.X - q.s * v.Y, q.s * v.X + q.c * v.Y);
        }

        /// Inverse rotate a vector
        public static Vec2 MulT(Rot q, Vec2 v)
        {
            return new Vec2(q.c * v.X + q.s * v.Y, -q.s * v.X + q.c * v.Y);
        }

        /// Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
        public static Vec2 Skew(Vec2 input)
        {
            return new Vec2(-input.Y, input.X);
        }

        /// <summary>
        /// This is a approximate yet fast inverse square-root.
        /// </summary>
        /// <param name="x">The x.</param>
        /// <returns></returns>
        public static Fix64 InvSqrt(Fix64 x)
        {
            FloatConverter convert = new FloatConverter();
            convert.x = x;
            Fix64 xhalf = Fix64.Half * x;
            convert.i = 0x5f3759df - (convert.i >> 1);
            x = convert.x;
            x = x * ((Fix64.One + Fix64.Half) - xhalf * x * x);
            return x;
        }

        public static int Clamp(int a, int low, int high)
        {
            return Math.Max(low, Math.Min(a, high));
        }

        public static Fix64 Clamp(Fix64 a, Fix64 low, Fix64 high)
        {
            return Fix64.Max(low, Fix64.Min(a, high));
        }

        public static Vec2 Clamp(Vec2 a, Vec2 low, Vec2 high)
        {
            return Vec2.Max(low, Vec2.Min(a, high));
        }

        public static void Cross(ref Vec2 a, ref Vec2 b, out Fix64 c)
        {
            c = a.X * b.Y - a.Y * b.X;
        }

        /// <summary>
        /// Return the angle between two vectors on a plane
        /// The angle is from vector 1 to vector 2, positive anticlockwise
        /// The result is between -pi -> pi
        /// </summary>
        public static Fix64 VectorAngle(ref Vec2 p1, ref Vec2 p2)
        {
            Fix64 theta1 = Fix64.Atan2(p1.Y, p1.X);
            Fix64 theta2 = Fix64.Atan2(p2.Y, p2.X);
            Fix64 dtheta = theta2 - theta1;
            while (dtheta > Fix64.Pi)
                dtheta -= (Fix64.PiTimes2);
            while (dtheta < -Fix64.Pi)
                dtheta += (Fix64.PiTimes2);

            return (dtheta);
        }

        /// Perform the dot product on two vectors.
        public static Fix64 Dot(Vec3 a, Vec3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        public static Fix64 VectorAngle(Vec2 p1, Vec2 p2)
        {
            return VectorAngle(ref p1, ref p2);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>
        /// Positive number if point is left, negative if point is right,
        /// and 0 if points are collinear.
        /// </returns>
        public static Fix64 Area(Vec2 a, Vec2 b, Vec2 c)
        {
            return Area(ref a, ref b, ref c);
        }

        /// <summary>
        /// Returns a positive number if c is to the left of the line going from a to b.
        /// </summary>
        /// <returns>
        /// Positive number if point is left, negative if point is right,
        /// and 0 if points are collinear.
        /// </returns>
        public static Fix64 Area(ref Vec2 a, ref Vec2 b, ref Vec2 c)
        {
            return a.X * (b.Y - c.Y) + b.X * (c.Y - a.Y) + c.X * (a.Y - b.Y);
        }

        /// <summary>
        /// Determines if three vertices are collinear (ie. on a straight line)
        /// </summary>
        /// <param name="a">First vertex</param>
        /// <param name="b">Second vertex</param>
        /// <param name="c">Third vertex</param>
        /// <param name="tolerance">The tolerance</param>
        /// <returns></returns>
        public static bool IsCollinear(ref Vec2 a, ref Vec2 b, ref Vec2 c, Fix64 tolerance)
        {
            return FloatInRange(Area(ref a, ref b, ref c), -tolerance, tolerance);
        }

        public static void Cross(Fix64 s, ref Vec2 a, out Vec2 b)
        {
            b = new Vec2(-s * a.Y, s * a.X);
        }

        public static bool FloatEquals(Fix64 value1, Fix64 value2)
        {
            return Fix64.Abs(value1 - value2) <= Fix64.Epsilon;
        }

        /// <summary>
        /// Checks if a Fix64ing point Value is equal to another,
        /// within a certain tolerance.
        /// </summary>
        /// <param name="value1">The first Fix64ing point Value.</param>
        /// <param name="value2">The second Fix64ing point Value.</param>
        /// <param name="delta">The Fix64ing point tolerance.</param>
        /// <returns>True if the values are "equal", false otherwise.</returns>
        public static bool FloatEquals(Fix64 value1, Fix64 value2, Fix64 delta)
        {
            return FloatInRange(value1, value2 - delta, value2 + delta);
        }

        /// <summary>
        /// Checks if a Fix64ing point Value is within a specified
        /// range of values (inclusive).
        /// </summary>
        /// <param name="value">The Value to check.</param>
        /// <param name="min">The minimum Value.</param>
        /// <param name="max">The maximum Value.</param>
        /// <returns>
        /// True if the Value is within the range specified,
        /// false otherwise.
        /// </returns>
        public static bool FloatInRange(Fix64 value, Fix64 min, Fix64 max)
        {
            return (value >= min && value <= max);
        }

        public static Vec2 Mul(ref Rot rot, Vec2 axis)
        {
            return Mul(rot, axis);
        }

        public static Vec2 MulT(ref Rot rot, Vec2 axis)
        {
            return MulT(rot, axis);
        }

        #region Nested type: FloatConverter

        [StructLayout(LayoutKind.Explicit)]
        private struct FloatConverter
        {
            [FieldOffset(0)]
            public Fix64 x;

            [FieldOffset(0)]
            public int i;
        }

        #endregion
    }
}