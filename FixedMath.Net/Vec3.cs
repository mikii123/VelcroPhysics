using UnityEngine;

namespace FixedMath.Net
{
	public partial struct Vec3
	{
		public Fix64 X;
		public Fix64 Y;

		/// <summary>
		///     extra dimension acting as the height
		/// </summary>
		public Fix64 Z;
		
		public Vec3(Fix64 x, Fix64 y, Fix64 z)
		{
			this.X = x;
			this.Y = y;
			this.Z = z;
		}

		public Vec3(Fix64 x, Fix64 y)
		{
			this.X = x;
			this.Y = y;
			Z = Fix64.Zero;
		}

		public override string ToString()
		{
			return $"( {X} , {Y} , {Z} )";
		}

		public Vec3(Vec2 v, Fix64 z)
		{
			X = v.X;
			Y = v.Y;
			this.Z = z;
		}
		
		public Vec3 xzy => new Vec3(X,Z,Y);

		public static Fix64 Dot(Vec3 lhs, Vec3 rhs)
		{
			return lhs.X * rhs.X + lhs.Y * rhs.Y + lhs.Z * rhs.Z;
		}

		public Fix64 Magnitude()
		{
			return Fix64.Sqrt(X * X + Y * Y + Z * Z);
		}

		public Fix64 MagnitudeSquared()
		{
			return X * X + Y * Y + Z * Z;
		}
		
		public static Vec3 ClampMagnitude(Vec3 vector, Fix64 maxLength)
		{
			if (vector.MagnitudeSquared() > maxLength * maxLength)
			{
				return vector.Normalized * maxLength;
			}

			return vector;
		}
		
		public void ClampMagnitude(Fix64 maxLength)
		{
			if (MagnitudeSquared() > maxLength * maxLength)
			{
				this = Normalized * maxLength;
			}
		}

		public Vec3 Normalized => MagnitudeSquared() == Fix64.Zero ? new Vec3() : this / Magnitude();
		
		public void Normalize()
		{
			this = MagnitudeSquared() == Fix64.Zero ? new Vec3() : this / Magnitude();
		}

		public void Set(Fix64 x, Fix64 y, Fix64 z)
		{
			this.X = x;
			this.Y = y;
			this.Z = z;
		}
		
		public void Set(Fix64 x, Fix64 y)
		{
			this.X = x;
			this.Y = y;
		}
		
		public void SetX(Fix64 value)
		{
			this.X = value;
		}
		
		public void SetY(Fix64 value)
		{
			this.Y = value;
		}
		
		public void SetZ(Fix64 value)
		{
			this.Z = value;
		}
		
		public Vec3 Rotated(Fix64 xRot, Fix64 yRot)
		{
			var forward = new Vec3(X, Y, Z);
			var rotation = new Vec2(xRot.Deg2Rad(), yRot.Deg2Rad());

			forward = new Vec3(
				forward.X,
				forward.Y * Fix64.FastCos(rotation.X) - forward.Z * Fix64.FastSin(rotation.X),
				forward.Y * Fix64.FastSin(rotation.X) + forward.Z * Fix64.FastCos(rotation.X)
			);
			forward = new Vec3(
				forward.X * Fix64.FastCos(rotation.Y) - forward.Y * Fix64.FastSin(rotation.Y),
				forward.X * Fix64.FastSin(rotation.Y) + forward.Y * Fix64.FastCos(rotation.Y),
				forward.Z
			);

			return forward;
		}
		
		/// <summary>
		///     Calculates rotation from directional vector.
		/// </summary>
		public static Vec2 LookRotation(Vec3 direction)
		{
			var flatForward = new Vec2(direction.X, direction.Y);
			Vec2 axis = flatForward.Rotated(Fix64.PiOver2);
			Fix64 yRot = Vec2.SignedAngle(Vec2.Right, flatForward.Rotated(-Fix64.PiOver2));
			Fix64 xRot = SignedAngle(new Vec3(flatForward, Fix64.Zero), direction, -new Vec3(axis, Fix64.Zero));

			return new Vec2(xRot, yRot);
		}

		/// <summary>
		///     Returns the angle in radians between /from/ and /to/. This is always the smallest.
		/// </summary>
		public static Fix64 Angle(Vec3 from, Vec3 to)
		{
			// sqrt(a) * sqrt(b) = sqrt(a * b) -- valid for real numbers
			Fix64 denominator = Fix64.Sqrt(from.MagnitudeSquared()* to.MagnitudeSquared());
			if (denominator <= Fix64.Zero)
			{
				return Fix64.Zero;
			}

			Fix64 dot = (Dot(from, to) / denominator).Clamp(-Fix64.One, Fix64.One);
			return Fix64.Acos(dot);
		}

		/// <summary>
		///     Returns signed angle in degrees.
		/// </summary>
		public static Fix64 SignedAngle(Vec3 from, Vec3 to, Vec3 axis)
		{
			Fix64 unsignedAngle = Angle(from, to).Rad2Deg();
			var sign = (Fix64)Fix64.Sign(Dot(axis, Cross(from, to)));
			return unsignedAngle * sign;
		}
		
		public static Vec3 Cross(Vec3 lhs, Vec3 rhs)
		{
			return new Vec3(
				lhs.Y * rhs.Z - lhs.Z * rhs.Y,
				lhs.Z * rhs.X - lhs.X * rhs.Z,
				lhs.X * rhs.Y - lhs.Y * rhs.X);
		}

		/// <summary>
		///     Moves a point /current/ in a straight line towards a /target/ point.
		/// </summary>
		public static Vec3 MoveTowards(Vec3 current, Vec3 target, Fix64 maxDistanceDelta)
		{
			Vec3 toVector = target - current;
			Fix64 dist = toVector.Magnitude();
			if (dist <= maxDistanceDelta || dist <= Fix64.Zero)
			{
				return target;
			}

			return current + toVector / dist * maxDistanceDelta;
		}
		
		public static Vec3 FromEulerAngles(Fix64 x, Fix64 y)
		{
			// X:
			// |x cos θ − y sin θ|
			// |x sin θ + y cos θ|
			// |        z        |

			// Y:
			// | x cos θ + z sin θ|
			// |         y        |
			// |−x sin θ + z cos θ|

			// X:
			// |        x        |
			// |y cos θ − z sin θ|
			// |y sin θ + z cos θ|

			var forward = new Vec3(Fix64.Zero, Fix64.One, Fix64.Zero);
			var rotation = new Vec2(x.Deg2Rad(), y.Deg2Rad());

			forward = new Vec3(
				forward.X,
				forward.Y * Fix64.FastCos(rotation.X) - forward.Z * Fix64.FastSin(rotation.X),
				forward.Y * Fix64.FastSin(rotation.X) + forward.Z * Fix64.FastCos(rotation.X)
			);
			forward = new Vec3(
				forward.X * Fix64.FastCos(rotation.Y) - forward.Y * Fix64.FastSin(rotation.Y),
				forward.X * Fix64.FastSin(rotation.Y) + forward.Y * Fix64.FastCos(rotation.Y),
				forward.Z
			);

			return forward;
		}

		public static Vec3 operator*(Vec3 v1, Fix64 value)
		{
			return new Vec3(v1.X * value, v1.Y * value, v1.Z * value);
		}
		
		public static Vec3 operator*(Fix64 value,Vec3 v1)
		{
			return new Vec3(v1.X * value, v1.Y * value, v1.Z * value);
		}

		public static Vec3 operator/(Vec3 v1, Fix64 value)
		{
			return new Vec3(v1.X / value, v1.Y / value, v1.Z / value);
		}

		public static Vec3 operator+(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.X + v2.X, v1.Y + v2.Y, v1.Z + v2.Z);
		}

		public static Vec3 operator-(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.X - v2.X, v1.Y - v2.Y, v1.Z - v2.Z);
		}

		public static explicit operator Vec2(Vec3 v)
		{
			return new Vec2(v.X, v.Y);
		}

		public static explicit operator Vec3(Vector3 v)
		{
			return new Vec3((Fix64)v.x, (Fix64)v.z, (Fix64)v.y);
		}

		public static explicit operator Vector3(Vec3 v)
		{
			return new Vector3((float)v.X, (float)v.Z, (float)v.Y);
		}

		public static explicit operator Vec3(Vec2 v)
		{
			return new Vec3(v.X, v.Y);
		}

		public static Vec3 operator-(Vec3 a)
		{
			return new Vec3(-a.X, -a.Y, -a.Z);
		}
	}
}
