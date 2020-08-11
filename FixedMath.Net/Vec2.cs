using System;
using UnityEngine;

namespace FixedMath.Net
{
	/***
	* Missing functionality?
	* https://github.com/Unity-Technologies/UnityCsReference/blob/master/Runtime/Export/Vector2.cs#L159
	*/

	[Serializable]
	public partial struct Vec2 : IEquatable<Vec2>
	{
		public Fix64 X;
		public Fix64 Y;

		//private static Fix64 kEpsilon = (Fix64)0.0000001;  

		public Vec2(Fix64 x, Fix64 y)
		{
			this.X = x;
			this.Y = y;
		}

		public Vec2(int x = 0, int y = 0)
		{
			this.X = (Fix64)x;
			this.Y = (Fix64)y;
		}

		public Vec2(Vector2 vector2)
		{
			X = (Fix64)vector2.x;
			Y = (Fix64)vector2.y;
		}

		public Vec2(float x = 0f, float y = 0f)
		{
			this.X = (Fix64)x;
			this.Y = (Fix64)y;
		}

		public override string ToString()
		{
			return $"({X}, {Y})";
		}

		public Vector2 ToUnity()
		{
			return new Vector2((float)X, (float)Y);
		}

		public static explicit operator Vector2(Vec2 value)
		{
			return value.ToUnity();
		}
		
		public void Set(Fix64 x, Fix64 y)
		{
			this.X = x;
			this.Y = y;
		}

		public Fix64 Magnitude => Fix64.Sqrt(X * X + Y * Y);

		public Fix64 MagnitudeSquared => X * X + Y * Y;

		public static Fix64 Distance(Vec2 from, Vec2 to)
		{
			return (from - to).Magnitude;
		}

		public static Fix64 DistanceSquared(Vec2 from, Vec2 to)
		{
			return (from - to).MagnitudeSquared;
		}

		public static Fix64 Dot(Vec2 from, Vec2 to)
		{
			return from.X * to.X + from.Y * to.Y;
		}

		public static Vec2 FromAngle(Fix64 signedAngle)
		{
			return Right.Rotated(signedAngle.Deg2Rad());
		}

		public Vec2 Normalized
		{
			get
			{
				Fix64 mag = Magnitude;
				return mag == Fix64.Zero ? Zero : new Vec2(X / mag, Y / mag);
			}
		}

		public Vec2 RightHandNormal => new Vec2(Y, -X).Normalized;
		public Vec2 LeftHandNormal => new Vec2(-Y, X).Normalized;

		public void Normalize()
		{
			Fix64 mag = Magnitude;

			if (mag == Fix64.Zero)
			{
				X = Fix64.Zero;
				Y = Fix64.Zero;
				return;
			}

			X /= mag;
			Y /= mag;
		}

		public static Vec2 ClampMagnitude(Vec2 vector, Fix64 maxLength)
		{
			if (vector.MagnitudeSquared > maxLength * maxLength)
			{
				return vector.Normalized * maxLength;
			}

			return vector;
		}

		/// <returns> Angle between two Vectors in degrees </returns>
		public static Fix64 Angle(Vec2 from, Vec2 to)
		{
			Fix64 epsilon = Fix64.FromRaw(4);
			var num = Fix64.Sqrt(from.MagnitudeSquared * to.MagnitudeSquared);
			return num < epsilon ? Fix64.Zero : Fix64.Acos((Dot(from, to) / num).Clamp(-Fix64.One, Fix64.One)).Rad2Deg();
		}

		public static Vec2 Lerp(Vec2 a, Vec2 b, Fix64 t)
		{
			t = t.Clamp(Fix64.Zero, Fix64.One);
			return new Vec2(a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t);
		}

		/// <returns> Vector rotation in radians </returns>
		public Fix64 Rotation => Fix64.Atan2(Y, X);

		/// <summary>
		///     Returns signed angle in degrees.
		/// </summary>
		public static Fix64 SignedAngle(Vec2 from, Vec2 to)
		{
			Fix64 angle = Angle(from, to);
			Fix64 raw180 = Fix64.FromRaw(773094113280);
			if (angle == raw180)
			{
				return raw180;
			}
			return angle * Fix64.SignF(from.X * to.Y - from.Y * to.X);
		}

		public static Vec2 Reflect(Vec2 inDirection, Vec2 inNormal)
		{
			var value = new Fix64(-2);
			return value * Dot(inNormal, inDirection) * inNormal + inDirection;
		}

		public static Vec2 SmoothDamp
		(
			Vec2 current,
			Vec2 target,
			ref Vec2 currentVelocity,
			Fix64 smoothTime,
			Fix64 maxSpeed,
			Fix64 deltaTime)
		{
			smoothTime = Fix64.Max(Fix64.FromRaw(429496), smoothTime);
			Fix64 num1 = Fix64.Two / smoothTime;
			Fix64 num2 = num1 * deltaTime;
			Fix64 num3 = (Fix64)(Fix64.One / (Fix64.One + num2 + Fix64.FromRaw(2061584256) * num2 * num2 + Fix64.FromRaw(1009317312) * num2 * num2 * num2));
			Vec2 vector = current - target;
			Vec2 vector2_1 = target;
			Fix64 maxLength = maxSpeed * smoothTime;
			Vec2 vector2_2 = Vec2.ClampMagnitude(vector, maxLength);
			target = current - vector2_2;
			Vec2 vector2_3 = (currentVelocity + num1 * vector2_2) * deltaTime;
			currentVelocity = (currentVelocity - num1 * vector2_3) * num3;
			Vec2 vector2_4 = target + (vector2_2 + vector2_3) * num3;
			if (Vec2.Dot(vector2_1 - current, vector2_4 - vector2_1) > Fix64.Zero)
			{
				vector2_4 = vector2_1;
				currentVelocity = (vector2_4 - vector2_1) / deltaTime;
			}

			return vector2_4;
		}

		public static Fix64 CrossProduct(Vec2 a, Vec2 b)
		{
			return a.X * b.Y - a.Y * b.X;
		}

		public static Vec2 CrossProduct(Vec2 v, Fix64 s)
		{
			return new Vec2(s * v.Y, -s * v.X);
		}

		public static Vec2 CrossProduct(Fix64 s, Vec2 v)
		{
			return new Vec2(-s * v.Y, s * v.X);
		}

		public bool Equals(Vec2 other)
		{
			return X.Equals(other.X) && Y.Equals(other.Y);
		}

		public static Vec2 Perpendicular(Vec2 inDirection)
		{
			return new Vec2(-inDirection.Y, inDirection.X);
		}

		public static Vec2 PerpendicularClockwise(Vec2 inDirection)
		{
			return new Vec2(inDirection.Y, -inDirection.X);
		}

		/// <summary>
		///     Rotates the Vector by desired angle
		/// </summary>
		/// <param name="angle">Angle in radians</param>
		public void Rotate(Fix64 angle)
		{
			Fix64 x = this.X;
			Fix64 y = this.Y;

			this.X = Fix64.Cos(angle) * x - Fix64.Sin(angle) * y;
			this.Y = Fix64.Sin(angle) * x + Fix64.Cos(angle) * y;
		}

		/// <param name="angle">Angle in radians</param>
		public void RotateAround(Vec2 pivot, Fix64 angle)
		{
			Vec2 diff = this - pivot;
			diff.Rotate(angle);
			this = pivot + diff;
		}

		/// <param name="angle">Angle in radians</param>
		public Vec2 Rotated(Fix64 angle)
		{
			var v = new Vec2(X, Y);
			v.Rotate(angle);
			return v;
		}

		public void ClampMagnitude(Fix64 max)
		{
			if (MagnitudeSquared <= max.Squared())
			{
				return;
			}

			SetMagnitude(max);
		}

		public void SetMagnitude(Fix64 magnitude)
		{
			this = Normalized * Magnitude;
		}

		public static Vec2 Projection(Vec2 v, Vec2 target)
		{
			Fix64 targetSqrMag = target.MagnitudeSquared;
			Fix64 dot = Dot(v, target);

			Fix64 px = dot / targetSqrMag * target.X;
			Fix64 py = dot / targetSqrMag * target.Y;
			return new Vec2(px, py);
		}

		public void Project(Vec2 target)
		{
			this = Projection(this, target);
		}

		public void ProjectOnUnit(Vec2 target)
		{
			this = ProjectionOnUnit(this, target);
		}

		public Fix64 Cross(Vec2 v)
		{
			return X * v.Y - Y * v.X;
		}

		/// <summary>
		///     If you have a unit vector this is cheaper than Projection
		/// </summary>
		public static Vec2 ProjectionOnUnit(Vec2 v, Vec2 target)
		{
			Fix64 dot = Dot(v, target);

			Fix64 px = dot * target.X;
			Fix64 py = dot * target.Y;
			return new Vec2(px, py);
		}

		public static explicit operator Vector3(Vec2 a)
		{
			return new Vector3((float)a.X, 0f, (float)a.Y);
		}

		public static Vec2 operator+(Vec2 a, Vec2 b)
		{
			return new Vec2(a.X + b.X, a.Y + b.Y);
		}

		public static Vec2 operator-(Vec2 a, Vec2 b)
		{
			return new Vec2(a.X - b.X, a.Y - b.Y);
		}

		public static Vec2 operator-(Vec2 a)
		{
			return new Vec2(-a.X, -a.Y);
		}

		public static Vec2 operator*(Vec2 v1, Fix64 value)
		{
			return new Vec2(v1.X * value, v1.Y * value);
		}

		public static Vec2 operator*(Vec2 v1, int value)
		{
			return v1 * new Fix64(value);
		}

		public static Fix64 operator*(Vec2 v1, Vec2 v2)
		{
			return v1.X * v2.X + v1.Y * v2.Y;
		}

		public static Vec2 operator*(Fix64 value, Vec2 v)
		{
			return new Vec2(v.X * value, v.Y * value);
		}

		public static Vec2 operator/(Vec2 v, Fix64 value)
		{
			return new Vec2(v.X / value, v.Y / value);
		}

		public static Vec2 operator/(Vec2 v, int value)
		{
			return v / new Fix64(value);
		}

		public static bool operator==(Vec2 a, Vec2 b)
		{
			return a.X == b.X && a.Y == b.Y;
		}

		public static bool operator!=(Vec2 a, Vec2 b)
		{
			return !(a == b);
		}

		// Shorthand for writing @@Vector2(0, 0)@@
		public static Vec2 Zero { get; } = new Vec2(Fix64.Zero, Fix64.Zero);

		// Shorthand for writing @@Vector2(1, 1)@@
		public static Vec2 One { get; } = new Vec2(Fix64.One, Fix64.One);

		// Shorthand for writing @@Vector2(0, 1)@@
		public static Vec2 Up { get; } = new Vec2(Fix64.Zero, Fix64.One);

		// Shorthand for writing @@Vector2(0, -1)@@
		public static Vec2 Down { get; } = new Vec2(Fix64.Zero, -Fix64.One);

		// Shorthand for writing @@Vector2(-1, 0)@@
		public static Vec2 Left { get; } = new Vec2(-Fix64.One, Fix64.Zero);

		// Shorthand for writing @@Vector2(1, 0)@@
		public static Vec2 Right { get; } = new Vec2(Fix64.One, Fix64.Zero);
	}
}
