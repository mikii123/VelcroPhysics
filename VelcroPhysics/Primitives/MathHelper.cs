#if !XNA && !WINDOWS_PHONE && !XBOX && !ANDROID && !MONOGAME

#region License

/*
MIT License
Copyright © 2006 The Mono.Xna Team

All rights reserved.

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

namespace FixedMath.Net
{
	public static class MathHelper
	{
		public static readonly Fix64 E = Fix64.FromRaw(11674931554);
		public static readonly Fix64 TwoPi = Fix64.Pi * Fix64.Two;

		public static Fix64 Barycentric(Fix64 value1, Fix64 value2, Fix64 value3, Fix64 amount1, Fix64 amount2)
		{
			return value1 + (value2 - value1) * amount1 + (value3 - value1) * amount2;
		}

		public static Fix64 CatmullRom(Fix64 value1, Fix64 value2, Fix64 value3, Fix64 value4, Fix64 amount)
		{
			// Using formula from http://www.mvps.org/directx/articles/catmull/
			// Internally using Fix64s not to lose precission
			Fix64 amountSquared = amount * amount;
			Fix64 amountCubed = amountSquared * amount;
			return (Fix64)(Fix64.Half * (Fix64.Two * value2 +
				(value3 - value1) * amount +
				(Fix64.Two * value1 - Fix64.Five * value2 + Fix64.Four * value3 - value4) * amountSquared +
				(Fix64.Three * value2 - value1 - Fix64.Three * value3 + value4) * amountCubed));
		}

		public static Fix64 Clamp(Fix64 value, Fix64 min, Fix64 max)
		{
			// First we check to see if we're greater than the max
			value = (value > max) ? max : value;

			// Then we check to see if we're less than the min.
			value = (value < min) ? min : value;

			// There's no check to see if min > max.
			return value;
		}

		public static Fix64 Distance(Fix64 value1, Fix64 value2)
		{
			return Fix64.Abs(value1 - value2);
		}

		public static Fix64 Hermite(Fix64 value1, Fix64 tangent1, Fix64 value2, Fix64 tangent2, Fix64 amount)
		{
			// All transformed to Fix64 not to lose precission
			// Otherwise, for high numbers of param:amount the result is NaN instead of Infinity
			Fix64 v1 = value1,
				v2 = value2,
				t1 = tangent1,
				t2 = tangent2,
				s = amount,
				result;
			Fix64 sCubed = s * s * s;
			Fix64 sSquared = s * s;

			if (amount == Fix64.Zero)
				result = value1;
			else if (amount == Fix64.One)
				result = value2;
			else
				result = (Fix64.Two * v1 - Fix64.Two * v2 + t2 + t1) * sCubed +
					(Fix64.Three * v2 - Fix64.Three * v1 - Fix64.Two * t1 - t2) * sSquared +
					t1 * s +
					v1;
			return (Fix64)result;
		}

		public static Fix64 Lerp(Fix64 value1, Fix64 value2, Fix64 amount)
		{
			return value1 + (value2 - value1) * amount;
		}

		public static Fix64 Max(Fix64 value1, Fix64 value2)
		{
			return Fix64.Max(value1, value2);
		}

		public static Fix64 Min(Fix64 value1, Fix64 value2)
		{
			return Fix64.Min(value1, value2);
		}

		public static Fix64 SmoothStep(Fix64 value1, Fix64 value2, Fix64 amount)
		{
			// It is expected that 0 < amount < 1
			// If amount < 0, return value1
			// If amount > 1, return value2
			Fix64 result = Clamp(amount, Fix64.Zero, Fix64.One);
			result = Hermite(value1, Fix64.Zero, value2, Fix64.Zero, result);
			return result;
		}
	}
}

#endif
