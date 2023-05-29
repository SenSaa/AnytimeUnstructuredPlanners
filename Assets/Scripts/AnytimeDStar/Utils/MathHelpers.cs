using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

namespace ADStar
{
    public static class MathHelpers
    {

        public static List<Vector3> CreateEvenlySpacedIntermediatePts(List<Vector3> dataPoints, float maxSpacing)
        {
            List<Vector3> points = new List<Vector3>();
            for (int i = 1; i < dataPoints.Count; i++) // As we are checking previous point relative to the current, we start from i=1.
            {
                // Get every 2 segments of point set (current and previous).
                Vector3[] pointsArray = new Vector3[2];
                pointsArray[0] = dataPoints[i - 1];
                pointsArray[1] = dataPoints[i];

                Vector3 pt1 = pointsArray[0];
                Vector3 pt2 = pointsArray[1];

                Vector3 dir = pt2 - pt1;
                float dist = dir.magnitude;
                dir.Normalize(); //make unit

                int count = Mathf.FloorToInt(dist / maxSpacing);
                if (count == 0) count = 1; //don't do this if you don't want it
                float n = dist / (count + 1);

                for (int j = 0; j < count; j++)
                {
                    Vector3 p = pt1 + dir * n * (j + 1);
                    //place point p
                    points.Add(p);
                }
            }
            return points;
        }


        // --------------------------------------------------------------------------------------------------------------------------------

        // Catmull Rom Spline.
        // https://www.habrador.com/tutorials/interpolation/1-catmull-rom-splines/
        // http://www.mvps.org/directx/articles/catmull/
        // https://www.iquilezles.org/www/articles/minispline/minispline.htm
        // http://www.mvps.org/directx/articles/catmull/

        // Display a spline between 2 points derived with the Catmull-Rom spline algorithm
        public static List<Vector3> CatmullRomSpline(List<Vector3> controlPointsList, int pos)
        {
            List<Vector3> splinePath = new List<Vector3>();
            // The 4 points we need to form a spline between p1 and p2
            Vector3 p0 = controlPointsList[ClampListPos(controlPointsList, pos - 1)];
            Vector3 p1 = controlPointsList[ClampListPos(controlPointsList, pos)];
            Vector3 p2 = controlPointsList[ClampListPos(controlPointsList, pos + 1)];
            Vector3 p3 = controlPointsList[ClampListPos(controlPointsList, pos + 2)];

            // The start position of the line
            Vector3 lastPos = p1;

            // The spline's resolution
            // Make sure it's is adding up to 1, so 0.3 will give a gap, but 0.2 will work
            // * The lower the value, the more control points per segment, the smoother the path, the more expensive the process!
            float resolution = 0.1f;

            //How many times should we loop?
            int loops = Mathf.FloorToInt(1f / resolution);

            for (int i = 1; i <= loops; i++)
            {
                // Which t position are we at?
                float t = i * resolution;

                // Find the coordinate between the end points with a Catmull-Rom spline
                Vector3 newPos = GetCatmullRomPosition(t, p0, p1, p2, p3);

                /*
                // Draw this line segment
                Gizmos.DrawLine(lastPos, newPos);

                // Save this pos so we can draw the next line segment
                lastPos = newPos;
                */

                splinePath.Add(newPos);
            }
            return splinePath;
        }


        //Clamp the list positions to allow looping
        public static int ClampListPos(List<Vector3> controlPointsList, int pos)
        {
            if (pos < 0)
            {
                pos = controlPointsList.Count - 1;
            }

            if (pos > controlPointsList.Count)
            {
                pos = 1;
            }
            else if (pos > controlPointsList.Count - 1)
            {
                pos = 0;
            }

            return pos;
        }


        // Returns a position between 4 Vector3 with Catmull-Rom spline algorithm
        //http://www.iquilezles.org/www/articles/minispline/minispline.htm
        public static Vector3 GetCatmullRomPosition(float t, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            // The coefficients of the cubic polynomial (except the 0.5f * which I added later for performance)
            Vector3 a = 2f * p1;
            Vector3 b = p2 - p0;
            Vector3 c = 2f * p0 - 5f * p1 + 4f * p2 - p3;
            Vector3 d = -p0 + 3f * p1 - 3f * p2 + p3;

            // The cubic polynomial: a + b * t + c * t^2 + d * t^3
            Vector3 pos = 0.5f * (a + (b * t) + (c * t * t) + (d * t * t * t));

            return pos;
        }

        // --------------------------------------------------------------------------------------------------------------------------------


        /// <summary>
        /// Create Rect from two points.
        /// </summary>
        /// <param name="pt1"> 1st point along length of rect. </param>
        /// <param name="pt2"> 2nd point along length of rect. </param>
        /// <returns> Rect corners V3 Tuple. </returns>
        public static Tuple<Vector3, Vector3, Vector3, Vector3> CreateRect(Vector3 pt1, Vector3 pt2)
        {
            float width = 4;

            float pt1X = pt1.x;
            float pt1Z = pt1.z;
            float pt2X = pt2.x;
            float pt2Z = pt2.z;

            Vector3 corner1 = Vector3.zero;
            Vector3 corner2 = Vector3.zero;
            Vector3 corner3 = Vector3.zero;
            Vector3 corner4 = Vector3.zero;

            if (Mathf.Abs(pt2X - pt1X) > Mathf.Abs(pt2Z - pt1Z))
            {
                corner1 = new Vector3(pt1X, 0, pt1Z - width / 2);
                corner2 = new Vector3(pt1X, 0, pt1Z + width / 2);
                corner3 = new Vector3(pt2X, 0, pt2Z + width / 2);
                corner4 = new Vector3(pt2X, 0, pt2Z - width / 2);
            }
            else
            {
                corner1 = new Vector3(pt1X - width / 2, 0, pt1Z);
                corner2 = new Vector3(pt1X + width / 2, 0, pt1Z);
                corner3 = new Vector3(pt2X + width / 2, 0, pt2Z);
                corner4 = new Vector3(pt2X - width / 2, 0, pt2Z);
            }

            Tuple<Vector3, Vector3, Vector3, Vector3> rectCorners = new Tuple<Vector3, Vector3, Vector3, Vector3>(corner1, corner2, corner3, corner4);
            return rectCorners;
        }


        /// <summary>
        /// Check if a point is within a polygon.
        /// </summary>
        /// <param name="polyPoints"> V2 point set of polygon. </param>
        /// <param name="p"> test point </param>
        /// <returns> Bool that is true when point is within polygon. </returns>
        // http://wiki.unity3d.com/index.php?title=PolyContainsPoint&oldid=20475
        public static bool ContainsPoint(Vector2[] polyPoints, Vector2 p)
        {
            var j = polyPoints.Length - 1;
            var inside = false;
            for (int i = 0; i < polyPoints.Length; j = i++)
            {
                var pi = polyPoints[i];
                var pj = polyPoints[j];
                if (((pi.y <= p.y && p.y < pj.y) || (pj.y <= p.y && p.y < pi.y)) &&
                    (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y) + pi.x))
                    inside = !inside;
            }
            return inside;
        }


        // Find hypotenuse of triangles, when given the remaining two sides.
        // Based on: https://www.c-sharpcorner.com/forums/hypotenuse
        // List<double> as Input and Output.
        public static List<double> Hypotenuse(List<double> side1, List<double> side2)
        {
            List<double> hypot = new List<double>();
            for (int i = 0; i < side1.Count; i++)
            {
                hypot.Add(Math.Sqrt(Math.Pow(side1[i], 2) + Math.Pow(side2[i], 2)));
            }
            return hypot;
        }
        // double data type as Input & Output. 
        public static double Hypotenuse(double side1, double side2)
        {
            double hypot = 0;
            {
                hypot = Math.Sqrt(Math.Pow(side1, 2) + Math.Pow(side2, 2));
            }
            return hypot;
        }

        // Sum Tuples of double.
        public static Tuple<double, double> SumDoubleTuples(Tuple<double, double> tpl1, Tuple<double, double> tpl2)
        {
            double item1 = tpl1.Item1 + tpl2.Item1;
            double item2 = tpl1.Item2 + tpl2.Item2;
            return Tuple.Create(item1, item2);
        }
        // Sum Tuples of int.
        public static Tuple<float, float> SumIntTuples(Tuple<float, float> tpl1, Tuple<float, float> tpl2)
        {
            float item1 = tpl1.Item1 + tpl2.Item1;
            float item2 = tpl1.Item2 + tpl2.Item2;
            return Tuple.Create(item1, item2);
        }
        public static Tuple<float, float> SumTuples(Tuple<float, float> tpl1, Tuple<float, float> tpl2)
        {
            float item1 = tpl1.Item1 + tpl2.Item1;
            float item2 = tpl1.Item2 + tpl2.Item2;
            return Tuple.Create(item1, item2);
        }


        public static T MinBy<T, C>(this IEnumerable<T> items, Func<T, C> projection) where C : IComparable<C>
        {
            return items.Aggregate((acc, e) => projection(acc).CompareTo(projection(e)) <= 0 ? acc : e);
        }

    }
}