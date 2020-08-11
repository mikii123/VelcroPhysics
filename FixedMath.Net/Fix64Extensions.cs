using System;
using System.Text;

namespace FixedMath.Net
{
	public static class Fix64Extensions
	{
		private static readonly Fix64 m_180 = Fix64.FromRaw(773094113280);
		
		#region PublicMethods

		public static Fix64 Remap(this Fix64 value, Fix64 from1, Fix64 to1, Fix64 from2, Fix64 to2)
		{
			return (value - from1) / (from2 - from1) * (to2 - to1) + to1;
		}

		public static Fix64 Clamp(this Fix64 value, Fix64 min, Fix64 max)
		{
			if ((double)value < (double)min)
			{
				value = min;
			}
			else if ((double)value > (double)max)
			{
				value = max;
			}

			return value;
		}

		public static Fix64 Rad2Deg(this Fix64 value)
		{
			return value * m_180 / Fix64.Pi;
		}

		public static Fix64 Deg2Rad(this Fix64 value)
		{
			return value * Fix64.Pi / m_180;
		}

		public static Fix64 Squared(this Fix64 v)
		{
			return v * v;
		}

		public static Fix64 Max(this Fix64 obj, Fix64 other)
		{
			return obj > other ? obj : other;
		}

		public static Fix64 Min(this Fix64 obj, Fix64 other)
		{
			return obj < other ? obj : other;
		}

		public static string ByteArrayToString(this byte[] ba)
		{
			StringBuilder hex = new StringBuilder(ba.Length * 2);
			foreach (byte b in ba)
				hex.AppendFormat("({0:x2})", b);
			return hex.ToString();
		}

		public static byte[] StringToByteArray(this String hex)
		{
			int NumberChars = hex.Length;
			byte[] bytes = new byte[NumberChars / 2];
			for (int i = 0; i < NumberChars; i += 2)
				bytes[i / 2] = Convert.ToByte(hex.Substring(i, 2), 16);
			return bytes;
		}

		#endregion
	}
}
