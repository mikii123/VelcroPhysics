#if !XNA && !WINDOWS_PHONE && !XBOX && !ANDROID && !MONOGAME

#region License

/*
MIT License
Copyright © 2006 The Mono.Xna Team

All rights reserved.

Authors:
 * Alan McGovern

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#endregion License

using System;
using System.Runtime.InteropServices;

namespace FixedMath.Net
{
    [StructLayout(LayoutKind.Sequential)]
    public partial struct Vec3 : IEquatable<Vec3>
    {
        #region Private Fields

        private static Vec3 zero = new Vec3(Fix64.Zero, Fix64.Zero, Fix64.Zero);
        private static Vec3 one = new Vec3(Fix64.One, Fix64.One, Fix64.One);
        private static Vec3 unitX = new Vec3(Fix64.One, Fix64.Zero, Fix64.Zero);
        private static Vec3 unitY = new Vec3(Fix64.Zero, Fix64.One, Fix64.Zero);
        private static Vec3 unitZ = new Vec3(Fix64.Zero, Fix64.Zero, Fix64.One);
        private static Vec3 up = new Vec3(Fix64.Zero, Fix64.One, Fix64.Zero);
        private static Vec3 down = new Vec3(Fix64.Zero, -Fix64.One, Fix64.Zero);
        private static Vec3 right = new Vec3(Fix64.One, Fix64.Zero, Fix64.Zero);
        private static Vec3 left = new Vec3(-Fix64.One, Fix64.Zero, Fix64.Zero);
        private static Vec3 forward = new Vec3(Fix64.Zero, Fix64.Zero, -Fix64.One);
        private static Vec3 backward = new Vec3(Fix64.Zero, Fix64.Zero, Fix64.One);

        #endregion Private Fields
        
        #region Properties

        public static Vec3 Zero
        {
            get { return zero; }
        }

        public static Vec3 One
        {
            get { return one; }
        }

        public static Vec3 UnitX
        {
            get { return unitX; }
        }

        public static Vec3 UnitY
        {
            get { return unitY; }
        }

        public static Vec3 UnitZ
        {
            get { return unitZ; }
        }

        public static Vec3 Up
        {
            get { return up; }
        }

        public static Vec3 Down
        {
            get { return down; }
        }

        public static Vec3 Right
        {
            get { return right; }
        }

        public static Vec3 Left
        {
            get { return left; }
        }

        public static Vec3 Forward
        {
            get { return forward; }
        }

        public static Vec3 Backward
        {
            get { return backward; }
        }

        #endregion Properties

        #region Constructors
        
        public Vec3(Fix64 value)
        {
            X = value;
            Y = value;
            Z = value;
        }

        #endregion Constructors

        #region Public Methods

        public static Vec3 Add(Vec3 value1, Vec3 value2)
        {
            value1.X += value2.X;
            value1.Y += value2.Y;
            value1.Z += value2.Z;
            return value1;
        }

        public static void Add(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result.X = value1.X + value2.X;
            result.Y = value1.Y + value2.Y;
            result.Z = value1.Z + value2.Z;
        }

        public static Vec3 Barycentric(Vec3 value1, Vec3 value2, Vec3 value3, Fix64 amount1, Fix64 amount2)
        {
            return new Vec3(
                MathHelper.Barycentric(value1.X, value2.X, value3.X, amount1, amount2),
                MathHelper.Barycentric(value1.Y, value2.Y, value3.Y, amount1, amount2),
                MathHelper.Barycentric(value1.Z, value2.Z, value3.Z, amount1, amount2));
        }

        public static void Barycentric(ref Vec3 value1, ref Vec3 value2, ref Vec3 value3, Fix64 amount1,
                                       Fix64 amount2, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.Barycentric(value1.X, value2.X, value3.X, amount1, amount2),
                MathHelper.Barycentric(value1.Y, value2.Y, value3.Y, amount1, amount2),
                MathHelper.Barycentric(value1.Z, value2.Z, value3.Z, amount1, amount2));
        }

        public static Vec3 CatmullRom(Vec3 value1, Vec3 value2, Vec3 value3, Vec3 value4, Fix64 amount)
        {
            return new Vec3(
                MathHelper.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
                MathHelper.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount),
                MathHelper.CatmullRom(value1.Z, value2.Z, value3.Z, value4.Z, amount));
        }

        public static void CatmullRom(ref Vec3 value1, ref Vec3 value2, ref Vec3 value3, ref Vec3 value4,
                                      Fix64 amount, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
                MathHelper.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount),
                MathHelper.CatmullRom(value1.Z, value2.Z, value3.Z, value4.Z, amount));
        }

        public static Vec3 Clamp(Vec3 value1, Vec3 min, Vec3 max)
        {
            return new Vec3(
                MathHelper.Clamp(value1.X, min.X, max.X),
                MathHelper.Clamp(value1.Y, min.Y, max.Y),
                MathHelper.Clamp(value1.Z, min.Z, max.Z));
        }

        public static void Clamp(ref Vec3 value1, ref Vec3 min, ref Vec3 max, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.Clamp(value1.X, min.X, max.X),
                MathHelper.Clamp(value1.Y, min.Y, max.Y),
                MathHelper.Clamp(value1.Z, min.Z, max.Z));
        }

        public static void Cross(ref Vec3 vector1, ref Vec3 vector2, out Vec3 result)
        {
            result = new Vec3(vector1.Y * vector2.Z - vector2.Y * vector1.Z,
                                 -(vector1.X * vector2.Z - vector2.X * vector1.Z),
                                 vector1.X * vector2.Y - vector2.X * vector1.Y);
        }

        public static Fix64 Distance(Vec3 vector1, Vec3 vector2)
        {
            Fix64 result;
            DistanceSquared(ref vector1, ref vector2, out result);
            return (Fix64)Fix64.Sqrt(result);
        }

        public static void Distance(ref Vec3 value1, ref Vec3 value2, out Fix64 result)
        {
            DistanceSquared(ref value1, ref value2, out result);
            result = (Fix64)Fix64.Sqrt(result);
        }

        public static Fix64 DistanceSquared(Vec3 value1, Vec3 value2)
        {
            Fix64 result;
            DistanceSquared(ref value1, ref value2, out result);
            return result;
        }

        public static void DistanceSquared(ref Vec3 value1, ref Vec3 value2, out Fix64 result)
        {
            result = (value1.X - value2.X) * (value1.X - value2.X) +
                     (value1.Y - value2.Y) * (value1.Y - value2.Y) +
                     (value1.Z - value2.Z) * (value1.Z - value2.Z);
        }

        public static Vec3 Divide(Vec3 value1, Vec3 value2)
        {
            value1.X /= value2.X;
            value1.Y /= value2.Y;
            value1.Z /= value2.Z;
            return value1;
        }

        public static Vec3 Divide(Vec3 value1, Fix64 value2)
        {
            Fix64 factor = Fix64.One / value2;
            value1.X *= factor;
            value1.Y *= factor;
            value1.Z *= factor;
            return value1;
        }

        public static void Divide(ref Vec3 value1, Fix64 divisor, out Vec3 result)
        {
            Fix64 factor = Fix64.One / divisor;
            result.X = value1.X * factor;
            result.Y = value1.Y * factor;
            result.Z = value1.Z * factor;
        }

        public static void Divide(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result.X = value1.X / value2.X;
            result.Y = value1.Y / value2.Y;
            result.Z = value1.Z / value2.Z;
        }
        
        public static void Dot(ref Vec3 vector1, ref Vec3 vector2, out Fix64 result)
        {
            result = vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z;
        }

        public override bool Equals(object obj)
        {
            return (obj is Vec3) ? this == (Vec3)obj : false;
        }

        public bool Equals(Vec3 other)
        {
            return this == other;
        }

        public override int GetHashCode()
        {
            return (int)(X + Y + Z);
        }

        public static Vec3 Hermite(Vec3 value1, Vec3 tangent1, Vec3 value2, Vec3 tangent2, Fix64 amount)
        {
            Vec3 result = new Vec3();
            Hermite(ref value1, ref tangent1, ref value2, ref tangent2, amount, out result);
            return result;
        }

        public static void Hermite(ref Vec3 value1, ref Vec3 tangent1, ref Vec3 value2, ref Vec3 tangent2,
                                   Fix64 amount, out Vec3 result)
        {
            result.X = MathHelper.Hermite(value1.X, tangent1.X, value2.X, tangent2.X, amount);
            result.Y = MathHelper.Hermite(value1.Y, tangent1.Y, value2.Y, tangent2.Y, amount);
            result.Z = MathHelper.Hermite(value1.Z, tangent1.Z, value2.Z, tangent2.Z, amount);
        }

        public Fix64 Length()
        {
            Fix64 result;
            DistanceSquared(ref this, ref zero, out result);
            return (Fix64)Fix64.Sqrt(result);
        }

        public Fix64 LengthSquared()
        {
            Fix64 result;
            DistanceSquared(ref this, ref zero, out result);
            return result;
        }

        public static Vec3 Lerp(Vec3 value1, Vec3 value2, Fix64 amount)
        {
            return new Vec3(
                MathHelper.Lerp(value1.X, value2.X, amount),
                MathHelper.Lerp(value1.Y, value2.Y, amount),
                MathHelper.Lerp(value1.Z, value2.Z, amount));
        }

        public static void Lerp(ref Vec3 value1, ref Vec3 value2, Fix64 amount, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.Lerp(value1.X, value2.X, amount),
                MathHelper.Lerp(value1.Y, value2.Y, amount),
                MathHelper.Lerp(value1.Z, value2.Z, amount));
        }

        public static Vec3 Max(Vec3 value1, Vec3 value2)
        {
            return new Vec3(
                MathHelper.Max(value1.X, value2.X),
                MathHelper.Max(value1.Y, value2.Y),
                MathHelper.Max(value1.Z, value2.Z));
        }

        public static void Max(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.Max(value1.X, value2.X),
                MathHelper.Max(value1.Y, value2.Y),
                MathHelper.Max(value1.Z, value2.Z));
        }

        public static Vec3 Min(Vec3 value1, Vec3 value2)
        {
            return new Vec3(
                MathHelper.Min(value1.X, value2.X),
                MathHelper.Min(value1.Y, value2.Y),
                MathHelper.Min(value1.Z, value2.Z));
        }

        public static void Min(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.Min(value1.X, value2.X),
                MathHelper.Min(value1.Y, value2.Y),
                MathHelper.Min(value1.Z, value2.Z));
        }

        public static Vec3 Multiply(Vec3 value1, Vec3 value2)
        {
            value1.X *= value2.X;
            value1.Y *= value2.Y;
            value1.Z *= value2.Z;
            return value1;
        }

        public static Vec3 Multiply(Vec3 value1, Fix64 scaleFactor)
        {
            value1.X *= scaleFactor;
            value1.Y *= scaleFactor;
            value1.Z *= scaleFactor;
            return value1;
        }

        public static void Multiply(ref Vec3 value1, Fix64 scaleFactor, out Vec3 result)
        {
            result.X = value1.X * scaleFactor;
            result.Y = value1.Y * scaleFactor;
            result.Z = value1.Z * scaleFactor;
        }

        public static void Multiply(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result.X = value1.X * value2.X;
            result.Y = value1.Y * value2.Y;
            result.Z = value1.Z * value2.Z;
        }

        public static Vec3 Negate(Vec3 value)
        {
            value = new Vec3(-value.X, -value.Y, -value.Z);
            return value;
        }

        public static void Negate(ref Vec3 value, out Vec3 result)
        {
            result = new Vec3(-value.X, -value.Y, -value.Z);
        }

        public static Vec3 Normalize(Vec3 vector)
        {
            Normalize(ref vector, out vector);
            return vector;
        }

        public static void Normalize(ref Vec3 value, out Vec3 result)
        {
            Fix64 factor;
            Distance(ref value, ref zero, out factor);
            factor = Fix64.One / factor;
            result.X = value.X * factor;
            result.Y = value.Y * factor;
            result.Z = value.Z * factor;
        }

        public static Vec3 Reflect(Vec3 vector, Vec3 normal)
        {
            Vec3 result;
            Reflect(ref vector, ref normal, out result);
            return result;
        }

        public static void Reflect(ref Vec3 vector, ref Vec3 normal, out Vec3 result)
        {
            Fix64 dot = Dot(vector, normal);
            result.X = vector.X - ((Fix64.Two * dot) * normal.X);
            result.Y = vector.Y - ((Fix64.Two * dot) * normal.Y);
            result.Z = vector.Z - ((Fix64.Two * dot) * normal.Z);
        }

        public static Vec3 SmoothStep(Vec3 value1, Vec3 value2, Fix64 amount)
        {
            return new Vec3(
                MathHelper.SmoothStep(value1.X, value2.X, amount),
                MathHelper.SmoothStep(value1.Y, value2.Y, amount),
                MathHelper.SmoothStep(value1.Z, value2.Z, amount));
        }

        public static void SmoothStep(ref Vec3 value1, ref Vec3 value2, Fix64 amount, out Vec3 result)
        {
            result = new Vec3(
                MathHelper.SmoothStep(value1.X, value2.X, amount),
                MathHelper.SmoothStep(value1.Y, value2.Y, amount),
                MathHelper.SmoothStep(value1.Z, value2.Z, amount));
        }

        public static Vec3 Subtract(Vec3 value1, Vec3 value2)
        {
            value1.X -= value2.X;
            value1.Y -= value2.Y;
            value1.Z -= value2.Z;
            return value1;
        }

        public static void Subtract(ref Vec3 value1, ref Vec3 value2, out Vec3 result)
        {
            result.X = value1.X - value2.X;
            result.Y = value1.Y - value2.Y;
            result.Z = value1.Z - value2.Z;
        }

        public static Vec3 Transform(Vec3 position, Matrix matrix)
        {
            Transform(ref position, ref matrix, out position);
            return position;
        }

        public static void Transform(ref Vec3 position, ref Matrix matrix, out Vec3 result)
        {
            result =
                new Vec3((position.X * matrix.M11) + (position.Y * matrix.M21) + (position.Z * matrix.M31) + matrix.M41,
                            (position.X * matrix.M12) + (position.Y * matrix.M22) + (position.Z * matrix.M32) + matrix.M42,
                            (position.X * matrix.M13) + (position.Y * matrix.M23) + (position.Z * matrix.M33) + matrix.M43);
        }

        public static void Transform(Vec3[] sourceArray, ref Matrix matrix, Vec3[] destinationArray)
        {
            throw new NotImplementedException();
        }

        public static void Transform(Vec3[] sourceArray, int sourceIndex, ref Matrix matrix,
                                     Vec3[] destinationArray, int destinationIndex, int length)
        {
            throw new NotImplementedException();
        }

        public static void TransformNormal(Vec3[] sourceArray, ref Matrix matrix, Vec3[] destinationArray)
        {
            throw new NotImplementedException();
        }

        public static void TransformNormal(Vec3[] sourceArray, int sourceIndex, ref Matrix matrix,
                                           Vec3[] destinationArray, int destinationIndex, int length)
        {
            throw new NotImplementedException();
        }

        public static Vec3 TransformNormal(Vec3 normal, Matrix matrix)
        {
            TransformNormal(ref normal, ref matrix, out normal);
            return normal;
        }

        public static void TransformNormal(ref Vec3 normal, ref Matrix matrix, out Vec3 result)
        {
            result = new Vec3((normal.X * matrix.M11) + (normal.Y * matrix.M21) + (normal.Z * matrix.M31),
                                 (normal.X * matrix.M12) + (normal.Y * matrix.M22) + (normal.Z * matrix.M32),
                                 (normal.X * matrix.M13) + (normal.Y * matrix.M23) + (normal.Z * matrix.M33));
        }

        #endregion Public methods

        #region Operators

        public static bool operator ==(Vec3 value1, Vec3 value2)
        {
            return value1.X == value2.X
                   && value1.Y == value2.Y
                   && value1.Z == value2.Z;
        }

        public static bool operator !=(Vec3 value1, Vec3 value2)
        {
            return !(value1 == value2);
        }

        public static Vec3 operator *(Vec3 value1, Vec3 value2)
        {
            value1.X *= value2.X;
            value1.Y *= value2.Y;
            value1.Z *= value2.Z;
            return value1;
        }

        public static Vec3 operator /(Vec3 value1, Vec3 value2)
        {
            value1.X /= value2.X;
            value1.Y /= value2.Y;
            value1.Z /= value2.Z;
            return value1;
        }
        
        #endregion
    }
}

#endif