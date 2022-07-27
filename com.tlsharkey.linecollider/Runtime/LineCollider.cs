using System.Collections.Generic;
using UnityEngine;

public class LineCollider: MonoBehaviour
{

    private void OnEnable()
    {
        LineRenderer line = GetComponent<LineRenderer>();
        if (line)
        {
            LineCollider.CreateColliderForLine(line, 2);
            //ShowMesh(line.GetComponent<MeshCollider>().sharedMesh);
        }
    }

    /// <summary>
    /// Creates a 3D mesh (extruded square) around a LineRenderer.
    /// Uses the LineRenderer's thickness to determine the size of the square.
    /// </summary>
    /// <param name="line">the line to create a mesh from</param>
    /// <param name="multiplier">applied to the line thickness. value of 1 will use the line thickness, value of 2 will use twice the line thickness, etc.</param>
    /// <returns></returns>
    public static Mesh GenerateMeshFromLine(LineRenderer line, float multiplier=1)
    {
        if (line.positionCount == 0) return GenerateMeshFromPoint(Vector3.zero);
        else if (line.positionCount == 1) return GenerateMeshFromPoint(line.GetPosition(0));

        Mesh mesh = new Mesh();

        // Get points
        Point[] points = new Point[line.positionCount];
        for (int i = 0; i < line.positionCount; i++)
            points[i] = new Point(line, i);

        // Create Mesh
        for (int i=1; i<points.Length; i++)
        {
            Vector3[] quad = points[i].GenerateQuadAtPoint(multiplier);
            if (i == 1)
            {
                Vector3[] firstQuad = points[0].GenerateQuadAtPoint(multiplier);
                mesh = BoxFromQuads(firstQuad, quad);
            }
            else
            {
                AddFaceTo(mesh, quad);
            }
        }

        //ShowMesh(mesh, line.transform);
        return mesh;
    }

    /// <summary>
    /// Creates a 3D mesh (cube) from a point.
    /// </summary>
    /// <param name="point">the location of the center of the mesh</param>
    /// <param name="size">the side length of the cube</param>
    /// <returns></returns>
    public static Mesh GenerateMeshFromPoint(Vector3 point, float size=0.04f)
    {
        Mesh m = new Mesh();
        m.vertices = new[] { 
            point + new Vector3(-1, -1, -1)*size/2,
            point + new Vector3(-1, 1, -1)*size/2,
            point + new Vector3(1, 1, -1)*size/2,
            point + new Vector3(1, -1, -1)*size/2,

            point + new Vector3(-1, -1, 1)*size/2,
            point + new Vector3(-1, 1, 1)*size/2,
            point + new Vector3(1, 1, 1)*size/2,
            point + new Vector3(1, -1, 1)*size/2,
        };

        m.triangles = new int[]
        {
            0, 1, 2,
            0, 2, 3,
            0, 5, 1,
            0, 4, 5,
            1, 5, 6,
            1, 6, 2,
            2, 6, 7,
            2, 7, 3,
            0, 4, 7,
            0, 7, 3
        };

        return m;
    }

    private static Mesh BoxFromQuads(Vector3[] a, Vector3[] b)
    {
        Mesh m = new Mesh();

        List<Vector3> positions = new List<Vector3>();
        positions.AddRange(a);
        positions.AddRange(b);
        m.vertices = positions.ToArray();

        // Triangles
        List<int> triangles = new List<int>();
        // Add front
        triangles.AddRange(new int[]
        {
            1, 3, 2,
            0, 3, 1
        });
        // Add sides
        for (int i = 0; i < a.Length; i++)
        {
            int p = (i - 1) >= 0 ? i - 1 : a.Length - 1; // previous point

            triangles.AddRange(new int[] {
                i, i + a.Length, p + a.Length,
                i, p + a.Length, p
            });
        }
        // Add back
        triangles.AddRange(new int[]
        {
            0+a.Length, 1+a.Length, 2+a.Length,
            0+a.Length, 2+a.Length, 3+a.Length
        });

        m.triangles = triangles.ToArray();
        return m;
    }

    private static void AddFaceTo(Mesh m, Vector3[] quad)
    {
        List<Vector3> positions = new List<Vector3>(m.vertices);
        int offset = positions.Count - quad.Length;
        positions.AddRange(quad);
        m.vertices = positions.ToArray();

        // Triangles
        List<int> triangles = new List<int>(m.triangles);
        // remove last face
        triangles.RemoveRange(triangles.Count - 6, 6);
        // Add sides
        for (int i = offset; i < offset+quad.Length; i++)
        {
            int p = (i - 1) >= offset ? i - 1 : offset + quad.Length - 1; // previous point

            triangles.AddRange(new int[] {
                i, i + quad.Length, p + quad.Length,
                i, p + quad.Length, p
            });
        }
        // Add back
        triangles.AddRange(new int[]
        {
            offset+0+quad.Length, offset+1+quad.Length, offset+2+quad.Length,
            offset+0+quad.Length, offset+2+quad.Length, offset+3+quad.Length
        });

        m.triangles = triangles.ToArray();
    }

    private static void ShowMesh(Mesh m, Transform parent = null)
    {
        GameObject g = new GameObject();
        g.transform.SetParent(parent, false);

        MeshFilter mf = g.AddComponent<MeshFilter>();
        MeshCollider mc = g.AddComponent<MeshCollider>();
        MeshRenderer mr = g.AddComponent<MeshRenderer>();

        mf.sharedMesh = m;
        mc.sharedMesh = m;
    }

    /// <summary>
    /// Creates and adds a mesh collider to a LineRenderer
    /// </summary>
    /// <param name="line">the line renderer to add the collider to</param>
    /// <param name="multiplier">applied to the line thickness. value of 1 will use the line thickness, value of 2 will use twice the line thickness, etc.</param>
    public static void CreateColliderForLine(LineRenderer line, float multiplier=1)
    {
        MeshCollider collider = line.GetComponent<MeshCollider>();
        if (collider == null)
            collider = line.gameObject.AddComponent<MeshCollider>();

        Mesh m = GenerateMeshFromLine(line, multiplier);
        collider.sharedMesh = m;
    }






    private struct Point
    {
        public LineRenderer line;
        public Vector3 Position;
        public float Thickness;
        public Vector3 Normal;
        public float DistanceToNext;

        public Point(LineRenderer line, int index, bool useGlobal = false)
        {
            this.line = line;
            float t = (float)index / (float)line.positionCount;

            this.Position = line.GetPosition(index);
            if (useGlobal && !line.useWorldSpace)
                this.Position = line.transform.TransformPoint(this.Position);

            // Set thickness
            // If using start and end thicknesses
            //this.Thickness = Mathf.Lerp(line.startWidth, line.endWidth, t);
            // If using curve
            this.Thickness = line.widthCurve.Evaluate(t) * line.widthMultiplier;

            // Get direction to next point
            if (line.positionCount < 2)
            {
                this.DistanceToNext = 0;
                this.Normal = Vector3.forward;
                return;
            }

            if (index == 0) // first point
            {
                this.DistanceToNext = (this.Position - line.GetPosition(index + 1)).magnitude;
                this.Normal = (this.Position - line.GetPosition(index + 1)).normalized;
            }
            else if (index == line.positionCount - 1) // last point
            {
                this.DistanceToNext = -1 * (line.GetPosition(index - 1) - this.Position).magnitude;
                this.Normal = (line.GetPosition(index - 1) - this.Position).normalized;
            }
            else // all the points in between
            {
                this.DistanceToNext = (this.Position - line.GetPosition(index + 1)).magnitude;

                Vector3 comingFrom = (line.GetPosition(index - 1) - this.Position).normalized;
                Vector3 goingTowards = (this.Position - line.GetPosition(index + 1)).normalized;
                this.Normal = ((comingFrom + goingTowards) / 2).normalized; // average the coming and going directions
            }
        }

        public List<Vector3> GenerateCircleAtPoint(float multiplier = 1)
        {
            List<Vector3> circlePoints = new List<Vector3>();

            MathPlane mathPlane = new MathPlane(this.Normal, this.Position);
            for (float theta = 0; theta < Mathf.PI * 2; theta += Mathf.PI * 2 / 100)
            {
                float x = this.Thickness * multiplier * Mathf.Cos(theta);
                float y = this.Thickness * multiplier * Mathf.Sin(theta);

                circlePoints.Add(mathPlane.TransformPoint(new Vector2(x, y)));
            }

            return circlePoints;
        }

        public Vector3[] GenerateQuadAtPoint(float multiplier = 1)
        {
            Vector3[] quadPoints = new Vector3[4];

            MathPlane mathPlane = new MathPlane(this.Normal, this.Position);
            quadPoints[0] = mathPlane.TransformPoint(new Vector2(-this.Thickness * multiplier / 2, -this.Thickness * multiplier / 2));
            quadPoints[1] = mathPlane.TransformPoint(new Vector2(-this.Thickness * multiplier / 2, this.Thickness * multiplier / 2));
            quadPoints[2] = mathPlane.TransformPoint(new Vector2(this.Thickness * multiplier / 2, this.Thickness * multiplier / 2));
            quadPoints[3] = mathPlane.TransformPoint(new Vector2(this.Thickness * multiplier / 2, -this.Thickness * multiplier / 2));

            return quadPoints;
        }
    }

    private struct MathPlane
    {
        public UnityEngine.Plane plane { get; private set; }
        public Vector3 worldOrigin { get; private set; }
        public Vector2 localOrigin { get { return Vector2.zero; } }
        public Vector3 position { get { return worldOrigin; } }
        public Vector3 up { get; private set; }
        public Vector3 forward { get { return plane.normal; } }
        public Vector3 backward { get { return -forward; } }
        public Vector3 down { get { return -up; } }
        public Vector3 right { get { return Vector3.Cross(up, forward).normalized; } }
        public Vector3 left { get { return -right; } }
        public float A { get { return forward.x; } }
        public float B { get { return forward.y; } }
        public float C { get { return forward.z; } }
        public float D { get { return -(A * worldOrigin.x + B * worldOrigin.y + C * worldOrigin.z); } }
        public Quaternion Rotation { get { return Quaternion.LookRotation(forward, up); } }

        /// <summary>
        /// Creates a MathPlane object for converting between 3d and 2d coordinate systems
        /// </summary>
        /// <param name="pt1">a point on the plane (will become origin)</param>
        /// <param name="pt2">another point on the plane</param>
        /// <param name="pt3">a third point on the plane</param>
        /// <param name="up">an up direction vector to define the 2d coordinate y-axis</param>
        public MathPlane(Vector3 pt1, Vector3 pt2, Vector3 pt3, Vector3 up)
        {
            this.worldOrigin = pt1;
            this.plane = new UnityEngine.Plane(pt1, pt2, pt3);
            this.up = up;
        }

        /// <summary>
        /// Creates a MathPlane object for converting between 3d and 2d coordinate systems
        /// </summary>
        /// <param name="normal">vector normal to the plane</param>
        /// <param name="origin">a point on the plane that will serve as the 2d origin</param>
        /// <param name="up">an up direction vector to define the 2d coordinate y-axis</param>
        public MathPlane(Vector3 normal, Vector3 origin, Vector3 up)
        {
            this.worldOrigin = origin;
            this.plane = new UnityEngine.Plane(normal, origin);
            this.up = up;
        }

        /// <summary>
        /// Creates a MathPlane object for converting between 3d and 2d coordinate systems
        /// </summary>
        /// <param name="normal">vector normal to the plane</param>
        /// <param name="origin">a point on the plane that will serve as the 2d origin</param>
        public MathPlane(Vector3 normal, Vector3 origin)
        {
            this.worldOrigin = origin;
            this.plane = new UnityEngine.Plane(normal, origin);
            this.up = Vector3.up;
        }

        /// <summary>
        /// Creates a MathPlane object for converting between 3d and 2d coordinate systems
        /// </summary>
        /// <param name="normal">vector normal to the plane</param>
        public MathPlane(Vector3 normal)
        {
            this.worldOrigin = Vector3.zero;
            this.plane = new UnityEngine.Plane(normal, this.worldOrigin);
            this.up = Vector3.up;
        }

        /// <summary>
        /// Transforms a point from the local plane coordinate space
        /// to the world coordinate space
        /// </summary>
        /// <param name="point2D">the point on the plane to get the world coords for</param>
        /// <returns></returns>
        public Vector3 TransformPoint(Vector2 point2D)
        {
            float x = worldOrigin.x + point2D.x * right.x + point2D.y * up.x;
            float y = worldOrigin.y + point2D.x * right.y + point2D.y * up.y;
            float z = worldOrigin.z + point2D.x * right.z + point2D.y * up.z;
            return new Vector3(x, y, z);
        }

        /// <summary>
        /// Transforms a point from the 3d world coordinate space
        /// to the 2d plane coordinate space
        /// </summary>
        /// <param name="worldPoint">the point to project on the plane and get the local coords for</param>
        /// <returns></returns>
        public Vector2 InverseTransformPoint(Vector3 worldPoint)
        {
            worldPoint = plane.ClosestPointOnPlane(worldPoint);

            Vector3 relative = worldPoint - worldOrigin;

            float denom = right.x * up.y - up.x * right.y;
            float u = (relative.x * up.y - relative.y * up.x) / denom;
            float v = (relative.y * right.x - relative.x * right.y) / denom;

            return new Vector2(u, v);
        }

        public Vector3[] TransformPoints(Vector2[] points)
        {
            Vector3[] ret = new Vector3[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                ret[i] = this.TransformPoint(points[i]);
            }
            return ret;
        }

        public Vector2[] InverseTransformPoints(Vector3[] points)
        {
            Vector2[] ret = new Vector2[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                ret[i] = this.InverseTransformPoint(points[i]);
            }
            return ret;
        }
    }
}
