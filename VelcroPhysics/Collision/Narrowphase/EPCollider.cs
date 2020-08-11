using System;
using FixedMath.Net;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Narrowphase
{
	public static class EPCollider
	{
		public static void Collide(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA, PolygonShape polygonB, ref Transform xfB)
		{
			// Algorithm:
			// 1. Classify v1 and v2
			// 2. Classify polygon centroid as front or back
			// 3. Flip normal if necessary
			// 4. Initialize normal range to [-pi, pi] about face normal
			// 5. Adjust normal range according to adjacent edges
			// 6. Visit each separating axes, only accept axes within the range
			// 7. Return if _any_ axis indicates separation
			// 8. Clip
			bool front;
			Vec2 lowerLimit,
				upperLimit;
			Vec2 normal;
			Vec2 normal0 = Vec2.Zero;
			Vec2 normal2 = Vec2.Zero;

			Transform xf = MathUtils.MulT(xfA, xfB);

			Vec2 centroidB = MathUtils.Mul(ref xf, polygonB.MassData.Centroid);

			Vec2 v0 = edgeA.Vertex0;
			Vec2 v1 = edgeA._vertex1;
			Vec2 v2 = edgeA._vertex2;
			Vec2 v3 = edgeA.Vertex3;

			bool hasVertex0 = edgeA.HasVertex0;
			bool hasVertex3 = edgeA.HasVertex3;

			Vec2 edge1 = v2 - v1;
			edge1.Normalize();
			Vec2 normal1 = new Vec2(edge1.Y, -edge1.X);
			Fix64 offset1 = Vec2.Dot(normal1, centroidB - v1);
			Fix64 offset0 = Fix64.Zero,
				offset2 = Fix64.Zero;
			bool convex1 = false,
				convex2 = false;

			// Is there a preceding edge?
			if (hasVertex0)
			{
				Vec2 edge0 = v1 - v0;
				edge0.Normalize();
				normal0 = new Vec2(edge0.Y, -edge0.X);
				convex1 = MathUtils.Cross(edge0, edge1) >= Fix64.Zero;
				offset0 = Vec2.Dot(normal0, centroidB - v0);
			}

			// Is there a following edge?
			if (hasVertex3)
			{
				Vec2 edge2 = v3 - v2;
				edge2.Normalize();
				normal2 = new Vec2(edge2.Y, -edge2.X);
				convex2 = MathUtils.Cross(edge1, edge2) > Fix64.Zero;
				offset2 = Vec2.Dot(normal2, centroidB - v2);
			}

			// Determine front or back collision. Determine collision normal limits.
			if (hasVertex0 && hasVertex3)
			{
				if (convex1 && convex2)
				{
					front = offset0 >= Fix64.Zero || offset1 >= Fix64.Zero || offset2 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = normal0;
						upperLimit = normal2;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal1;
						upperLimit = -normal1;
					}
				}
				else if (convex1)
				{
					front = offset0 >= Fix64.Zero || (offset1 >= Fix64.Zero && offset2 >= Fix64.Zero);
					if (front)
					{
						normal = normal1;
						lowerLimit = normal0;
						upperLimit = normal1;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal2;
						upperLimit = -normal1;
					}
				}
				else if (convex2)
				{
					front = offset2 >= Fix64.Zero || (offset0 >= Fix64.Zero && offset1 >= Fix64.Zero);
					if (front)
					{
						normal = normal1;
						lowerLimit = normal1;
						upperLimit = normal2;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal1;
						upperLimit = -normal0;
					}
				}
				else
				{
					front = offset0 >= Fix64.Zero && offset1 >= Fix64.Zero && offset2 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = normal1;
						upperLimit = normal1;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal2;
						upperLimit = -normal0;
					}
				}
			}
			else if (hasVertex0)
			{
				if (convex1)
				{
					front = offset0 >= Fix64.Zero || offset1 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = normal0;
						upperLimit = -normal1;
					}
					else
					{
						normal = -normal1;
						lowerLimit = normal1;
						upperLimit = -normal1;
					}
				}
				else
				{
					front = offset0 >= Fix64.Zero && offset1 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = normal1;
						upperLimit = -normal1;
					}
					else
					{
						normal = -normal1;
						lowerLimit = normal1;
						upperLimit = -normal0;
					}
				}
			}
			else if (hasVertex3)
			{
				if (convex2)
				{
					front = offset1 >= Fix64.Zero || offset2 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = -normal1;
						upperLimit = normal2;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal1;
						upperLimit = normal1;
					}
				}
				else
				{
					front = offset1 >= Fix64.Zero && offset2 >= Fix64.Zero;
					if (front)
					{
						normal = normal1;
						lowerLimit = -normal1;
						upperLimit = normal1;
					}
					else
					{
						normal = -normal1;
						lowerLimit = -normal2;
						upperLimit = normal1;
					}
				}
			}
			else
			{
				front = offset1 >= Fix64.Zero;
				if (front)
				{
					normal = normal1;
					lowerLimit = -normal1;
					upperLimit = -normal1;
				}
				else
				{
					normal = -normal1;
					lowerLimit = normal1;
					upperLimit = normal1;
				}
			}

			// Get polygonB in frameA
			Vec2[] normals = new Vec2[Settings.MaxPolygonVertices];
			Vec2[] vertices = new Vec2[Settings.MaxPolygonVertices];
			int count = polygonB.Vertices.Count;
			for (int i = 0; i < polygonB.Vertices.Count; ++i)
			{
				vertices[i] = MathUtils.Mul(ref xf, polygonB.Vertices[i]);
				normals[i] = MathUtils.Mul(xf.q, polygonB.Normals[i]);
			}

			Fix64 radius = polygonB.Radius + edgeA.Radius;

			manifold.PointCount = 0;

			//Velcro: ComputeEdgeSeparation() was manually inlined here
			EPAxis edgeAxis;
			edgeAxis.Type = EPAxisType.EdgeA;
			edgeAxis.Index = front ? 0 : 1;
			edgeAxis.Separation = Fix64.MaxValue;

			for (int i = 0; i < count; ++i)
			{
				Fix64 s = Vec2.Dot(normal, vertices[i] - v1);
				if (s < edgeAxis.Separation)
				{
					edgeAxis.Separation = s;
				}
			}

			// If no valid normal can be found than this edge should not collide.
			if (edgeAxis.Type == EPAxisType.Unknown)
			{
				return;
			}

			if (edgeAxis.Separation > radius)
			{
				return;
			}

			//Velcro: ComputePolygonSeparation() was manually inlined here
			EPAxis polygonAxis;
			polygonAxis.Type = EPAxisType.Unknown;
			polygonAxis.Index = -1;
			polygonAxis.Separation = -Fix64.MinValue;

			Vec2 perp = new Vec2(-normal.Y, normal.X);

			for (int i = 0; i < count; ++i)
			{
				Vec2 n = -normals[i];

				Fix64 s1 = Vec2.Dot(n, vertices[i] - v1);
				Fix64 s2 = Vec2.Dot(n, vertices[i] - v2);
				Fix64 s = Fix64.Min(s1, s2);

				if (s > radius)
				{
					// No collision
					polygonAxis.Type = EPAxisType.EdgeB;
					polygonAxis.Index = i;
					polygonAxis.Separation = s;
					break;
				}

				// Adjacency
				if (Vec2.Dot(n, perp) >= Fix64.Zero)
				{
					if (Vec2.Dot(n - upperLimit, normal) < -Settings.AngularSlop)
					{
						continue;
					}
				}
				else
				{
					if (Vec2.Dot(n - lowerLimit, normal) < -Settings.AngularSlop)
					{
						continue;
					}
				}

				if (s > polygonAxis.Separation)
				{
					polygonAxis.Type = EPAxisType.EdgeB;
					polygonAxis.Index = i;
					polygonAxis.Separation = s;
				}
			}

			if (polygonAxis.Type != EPAxisType.Unknown && polygonAxis.Separation > radius)
			{
				return;
			}

			// Use hysteresis for jitter reduction.
			Fix64 k_relativeTol = new Fix64(98) / new Fix64(100);
			Fix64 k_absoluteTol = Fix64.One / new Fix64(1000);

			EPAxis primaryAxis;
			if (polygonAxis.Type == EPAxisType.Unknown)
			{
				primaryAxis = edgeAxis;
			}
			else if (polygonAxis.Separation > k_relativeTol * edgeAxis.Separation + k_absoluteTol)
			{
				primaryAxis = polygonAxis;
			}
			else
			{
				primaryAxis = edgeAxis;
			}

			FixedArray2<ClipVertex> ie = new FixedArray2<ClipVertex>();
			ReferenceFace rf;
			if (primaryAxis.Type == EPAxisType.EdgeA)
			{
				manifold.Type = ManifoldType.FaceA;

				// Search for the polygon normal that is most anti-parallel to the edge normal.
				int bestIndex = 0;
				Fix64 bestValue = Vec2.Dot(normal, normals[0]);
				for (int i = 1; i < count; ++i)
				{
					Fix64 value = Vec2.Dot(normal, normals[i]);
					if (value < bestValue)
					{
						bestValue = value;
						bestIndex = i;
					}
				}

				int i1 = bestIndex;
				int i2 = i1 + 1 < count ? i1 + 1 : 0;

				ie.Value0.V = vertices[i1];
				ie.Value0.ID.ContactFeature.IndexA = 0;
				ie.Value0.ID.ContactFeature.IndexB = (byte)i1;
				ie.Value0.ID.ContactFeature.TypeA = ContactFeatureType.Face;
				ie.Value0.ID.ContactFeature.TypeB = ContactFeatureType.Vertex;

				ie.Value1.V = vertices[i2];
				ie.Value1.ID.ContactFeature.IndexA = 0;
				ie.Value1.ID.ContactFeature.IndexB = (byte)i2;
				ie.Value1.ID.ContactFeature.TypeA = ContactFeatureType.Face;
				ie.Value1.ID.ContactFeature.TypeB = ContactFeatureType.Vertex;

				if (front)
				{
					rf.i1 = 0;
					rf.i2 = 1;
					rf.v1 = v1;
					rf.v2 = v2;
					rf.Normal = normal1;
				}
				else
				{
					rf.i1 = 1;
					rf.i2 = 0;
					rf.v1 = v2;
					rf.v2 = v1;
					rf.Normal = -normal1;
				}
			}
			else
			{
				manifold.Type = ManifoldType.FaceB;

				ie.Value0.V = v1;
				ie.Value0.ID.ContactFeature.IndexA = 0;
				ie.Value0.ID.ContactFeature.IndexB = (byte)primaryAxis.Index;
				ie.Value0.ID.ContactFeature.TypeA = ContactFeatureType.Vertex;
				ie.Value0.ID.ContactFeature.TypeB = ContactFeatureType.Face;

				ie.Value1.V = v2;
				ie.Value1.ID.ContactFeature.IndexA = 0;
				ie.Value1.ID.ContactFeature.IndexB = (byte)primaryAxis.Index;
				ie.Value1.ID.ContactFeature.TypeA = ContactFeatureType.Vertex;
				ie.Value1.ID.ContactFeature.TypeB = ContactFeatureType.Face;

				rf.i1 = primaryAxis.Index;
				rf.i2 = rf.i1 + 1 < count ? rf.i1 + 1 : 0;
				rf.v1 = vertices[rf.i1];
				rf.v2 = vertices[rf.i2];
				rf.Normal = normals[rf.i1];
			}

			rf.SideNormal1 = new Vec2(rf.Normal.Y, -rf.Normal.X);
			rf.SideNormal2 = -rf.SideNormal1;
			rf.SideOffset1 = Vec2.Dot(rf.SideNormal1, rf.v1);
			rf.SideOffset2 = Vec2.Dot(rf.SideNormal2, rf.v2);

			// Clip incident edge against extruded edge1 side edges.
			FixedArray2<ClipVertex> clipPoints1;
			FixedArray2<ClipVertex> clipPoints2;
			int np;

			// Clip to box side 1
			np = Collision.ClipSegmentToLine(out clipPoints1, ref ie, rf.SideNormal1, rf.SideOffset1, rf.i1);

			if (np < Settings.MaxManifoldPoints)
			{
				return;
			}

			// Clip to negative box side 1
			np = Collision.ClipSegmentToLine(out clipPoints2, ref clipPoints1, rf.SideNormal2, rf.SideOffset2, rf.i2);

			if (np < Settings.MaxManifoldPoints)
			{
				return;
			}

			// Now clipPoints2 contains the clipped points.
			if (primaryAxis.Type == EPAxisType.EdgeA)
			{
				manifold.LocalNormal = rf.Normal;
				manifold.LocalPoint = rf.v1;
			}
			else
			{
				manifold.LocalNormal = polygonB.Normals[rf.i1];
				manifold.LocalPoint = polygonB.Vertices[rf.i1];
			}

			int pointCount = 0;
			for (int i = 0; i < Settings.MaxManifoldPoints; ++i)
			{
				Fix64 separation = Vec2.Dot(rf.Normal, clipPoints2[i].V - rf.v1);

				if (separation <= radius)
				{
					ManifoldPoint cp = manifold.Points[pointCount];

					if (primaryAxis.Type == EPAxisType.EdgeA)
					{
						cp.LocalPoint = MathUtils.MulT(ref xf, clipPoints2[i].V);
						cp.Id = clipPoints2[i].ID;
					}
					else
					{
						cp.LocalPoint = clipPoints2[i].V;
						cp.Id.ContactFeature.TypeA = clipPoints2[i].ID.ContactFeature.TypeB;
						cp.Id.ContactFeature.TypeB = clipPoints2[i].ID.ContactFeature.TypeA;
						cp.Id.ContactFeature.IndexA = clipPoints2[i].ID.ContactFeature.IndexB;
						cp.Id.ContactFeature.IndexB = clipPoints2[i].ID.ContactFeature.IndexA;
					}

					manifold.Points[pointCount] = cp;
					++pointCount;
				}
			}

			manifold.PointCount = pointCount;
		}
	}
}
