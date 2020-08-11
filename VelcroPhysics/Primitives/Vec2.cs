#if !XNA && !WINDOWS_PHONE && !XBOX && !ANDROID && !MONOGAME

#region License

/*
MIT License
Copyright © 2006 The Mono.Xna Team

All rights reserved.

Authors
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
    public partial struct Vec2 : IEquatable<Vec2>
    {
	    #region Constructors
        
        /// <summary>
        /// Constructor for "square" vector.
        /// </summary>
        /// <param name="value">
        /// A <see cref="System.Single"/>
        /// </param>
        public Vec2(Fix64 value)
        {
            X = value;
            Y = value;
        }

        #endregion Constructors

        #region Public Methods

        public static void Reflect(ref Vec2 vector, ref Vec2 normal, out Vec2 result)
        {
            Fix64 dot = Dot(vector, normal);
            result.X = vector.X - ((Fix64.Two * dot) * normal.X);
            result.Y = vector.Y - ((Fix64.Two * dot) * normal.Y);
        }

        public static Vec2 Add(Vec2 value1, Vec2 value2)
        {
            value1.X += value2.X;
            value1.Y += value2.Y;
            return value1;
        }

        public static void Add(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result.X = value1.X + value2.X;
            result.Y = value1.Y + value2.Y;
        }

        public static Vec2 Barycentric(Vec2 value1, Vec2 value2, Vec2 value3, Fix64 amount1, Fix64 amount2)
        {
            return new Vec2(
                MathHelper.Barycentric(value1.X, value2.X, value3.X, amount1, amount2),
                MathHelper.Barycentric(value1.Y, value2.Y, value3.Y, amount1, amount2));
        }

        public static void Barycentric(ref Vec2 value1, ref Vec2 value2, ref Vec2 value3, Fix64 amount1,
                                       Fix64 amount2, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.Barycentric(value1.X, value2.X, value3.X, amount1, amount2),
                MathHelper.Barycentric(value1.Y, value2.Y, value3.Y, amount1, amount2));
        }

        public static Vec2 CatmullRom(Vec2 value1, Vec2 value2, Vec2 value3, Vec2 value4, Fix64 amount)
        {
            return new Vec2(
                MathHelper.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
                MathHelper.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount));
        }

        public static void CatmullRom(ref Vec2 value1, ref Vec2 value2, ref Vec2 value3, ref Vec2 value4,
                                      Fix64 amount, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.CatmullRom(value1.X, value2.X, value3.X, value4.X, amount),
                MathHelper.CatmullRom(value1.Y, value2.Y, value3.Y, value4.Y, amount));
        }

        public static Vec2 Clamp(Vec2 value1, Vec2 min, Vec2 max)
        {
            return new Vec2(
                MathHelper.Clamp(value1.X, min.X, max.X),
                MathHelper.Clamp(value1.Y, min.Y, max.Y));
        }

        public static void Clamp(ref Vec2 value1, ref Vec2 min, ref Vec2 max, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.Clamp(value1.X, min.X, max.X),
                MathHelper.Clamp(value1.Y, min.Y, max.Y));
        }

        public static void Distance(ref Vec2 value1, ref Vec2 value2, out Fix64 result)
        {
            DistanceSquared(ref value1, ref value2, out result);
            result = (Fix64)Fix64.Sqrt(result);
        }

        public static void DistanceSquared(ref Vec2 value1, ref Vec2 value2, out Fix64 result)
        {
            result = (value1.X - value2.X) * (value1.X - value2.X) + (value1.Y - value2.Y) * (value1.Y - value2.Y);
        }

        /// <summary>
        /// Devide first vector with the secund vector
        /// </summary>
        /// <param name="value1">
        /// A <see cref="Vec2"/>
        /// </param>
        /// <param name="value2">
        /// A <see cref="Vec2"/>
        /// </param>
        /// <returns>
        /// A <see cref="Vec2"/>
        /// </returns>
        public static Vec2 Divide(Vec2 value1, Vec2 value2)
        {
            value1.X /= value2.X;
            value1.Y /= value2.Y;
            return value1;
        }

        public static void Divide(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result.X = value1.X / value2.X;
            result.Y = value1.Y / value2.Y;
        }

        public static Vec2 Divide(Vec2 value1, Fix64 divider)
        {
            Fix64 factor = Fix64.One / divider;
            value1.X *= factor;
            value1.Y *= factor;
            return value1;
        }

        public static void Divide(ref Vec2 value1, Fix64 divider, out Vec2 result)
        {
            Fix64 factor = Fix64.One / divider;
            result.X = value1.X * factor;
            result.Y = value1.Y * factor;
        }

        public static void Dot(ref Vec2 value1, ref Vec2 value2, out Fix64 result)
        {
            result = value1.X * value2.X + value1.Y * value2.Y;
        }

        public override bool Equals(object obj)
        {
            return (obj is Vec2 vec2) && this == vec2;
        }

        public override int GetHashCode()
        {
            return (int)(X + Y);
        }

        public static Vec2 Hermite(Vec2 value1, Vec2 tangent1, Vec2 value2, Vec2 tangent2, Fix64 amount)
        {
            Vec2 result = new Vec2();
            Hermite(ref value1, ref tangent1, ref value2, ref tangent2, amount, out result);
            return result;
        }

        public static void Hermite(ref Vec2 value1, ref Vec2 tangent1, ref Vec2 value2, ref Vec2 tangent2,
                                   Fix64 amount, out Vec2 result)
        {
            result.X = MathHelper.Hermite(value1.X, tangent1.X, value2.X, tangent2.X, amount);
            result.Y = MathHelper.Hermite(value1.Y, tangent1.Y, value2.Y, tangent2.Y, amount);
        }

        public static void Lerp(ref Vec2 value1, ref Vec2 value2, Fix64 amount, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.Lerp(value1.X, value2.X, amount),
                MathHelper.Lerp(value1.Y, value2.Y, amount));
        }

        public static Vec2 Max(Vec2 value1, Vec2 value2)
        {
            return new Vec2(
                MathHelper.Max(value1.X, value2.X),
                MathHelper.Max(value1.Y, value2.Y));
        }

        public static void Max(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.Max(value1.X, value2.X),
                MathHelper.Max(value1.Y, value2.Y));
        }

        public static Vec2 Min(Vec2 value1, Vec2 value2)
        {
            return new Vec2(
                MathHelper.Min(value1.X, value2.X),
                MathHelper.Min(value1.Y, value2.Y));
        }

        public static void Min(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.Min(value1.X, value2.X),
                MathHelper.Min(value1.Y, value2.Y));
        }

        public static Vec2 Multiply(Vec2 value1, Vec2 value2)
        {
            value1.X *= value2.X;
            value1.Y *= value2.Y;
            return value1;
        }

        public static Vec2 Multiply(Vec2 value1, Fix64 scaleFactor)
        {
            value1.X *= scaleFactor;
            value1.Y *= scaleFactor;
            return value1;
        }

        public static void Multiply(ref Vec2 value1, Fix64 scaleFactor, out Vec2 result)
        {
            result.X = value1.X * scaleFactor;
            result.Y = value1.Y * scaleFactor;
        }

        public static void Multiply(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result.X = value1.X * value2.X;
            result.Y = value1.Y * value2.Y;
        }

        public static Vec2 Negate(Vec2 value)
        {
            value.X = -value.X;
            value.Y = -value.Y;
            return value;
        }

        public static void Negate(ref Vec2 value, out Vec2 result)
        {
            result.X = -value.X;
            result.Y = -value.Y;
        }

        public static Vec2 SmoothStep(Vec2 value1, Vec2 value2, Fix64 amount)
        {
            return new Vec2(
                MathHelper.SmoothStep(value1.X, value2.X, amount),
                MathHelper.SmoothStep(value1.Y, value2.Y, amount));
        }

        public static void SmoothStep(ref Vec2 value1, ref Vec2 value2, Fix64 amount, out Vec2 result)
        {
            result = new Vec2(
                MathHelper.SmoothStep(value1.X, value2.X, amount),
                MathHelper.SmoothStep(value1.Y, value2.Y, amount));
        }

        public static Vec2 Subtract(Vec2 value1, Vec2 value2)
        {
            value1.X -= value2.X;
            value1.Y -= value2.Y;
            return value1;
        }

        public static void Subtract(ref Vec2 value1, ref Vec2 value2, out Vec2 result)
        {
            result.X = value1.X - value2.X;
            result.Y = value1.Y - value2.Y;
        }

        public static Vec2 Transform(Vec2 position, Matrix matrix)
        {
            Transform(ref position, ref matrix, out position);
            return position;
        }

        public static void Transform(ref Vec2 position, ref Matrix matrix, out Vec2 result)
        {
            result = new Vec2((position.X * matrix.M11) + (position.Y * matrix.M21) + matrix.M41,
                                 (position.X * matrix.M12) + (position.Y * matrix.M22) + matrix.M42);
        }

        public static void Transform(Vec2[] sourceArray, ref Matrix matrix, Vec2[] destinationArray)
        {
            throw new NotImplementedException();
        }

        public static void Transform(Vec2[] sourceArray, int sourceIndex, ref Matrix matrix,
                                     Vec2[] destinationArray, int destinationIndex, int length)
        {
            throw new NotImplementedException();
        }

        public static Vec2 TransformNormal(Vec2 normal, Matrix matrix)
        {
            TransformNormal(ref normal, ref matrix, out normal);
            return normal;
        }

        public static void TransformNormal(ref Vec2 normal, ref Matrix matrix, out Vec2 result)
        {
            result = new Vec2((normal.X * matrix.M11) + (normal.Y * matrix.M21),
                                 (normal.X * matrix.M12) + (normal.Y * matrix.M22));
        }

        public static void TransformNormal(Vec2[] sourceArray, ref Matrix matrix, Vec2[] destinationArray)
        {
            throw new NotImplementedException();
        }

        public static void TransformNormal(Vec2[] sourceArray, int sourceIndex, ref Matrix matrix,
                                           Vec2[] destinationArray, int destinationIndex, int length)
        {
            throw new NotImplementedException();
        }

        #endregion Public Methods

        #region Operators

        public static Vec2 operator /(Vec2 value1, Vec2 value2)
        {
            value1.X /= value2.X;
            value1.Y /= value2.Y;
            return value1;
        }

        #endregion Operators
    }
}

#endif