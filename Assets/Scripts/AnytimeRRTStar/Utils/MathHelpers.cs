using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;

namespace ARRTStar
{
    public static class MathHelpers
    {
        public static T MinBy<T, C>(this IEnumerable<T> items, Func<T, C> projection) where C : IComparable<C>
        {
            return items.Aggregate((acc, e) => projection(acc).CompareTo(projection(e)) <= 0 ? acc : e);
        }

        public static Tuple<double, double> Pos2DToDoubleTuple(Position2D Pos2D)
        {
            return Tuple.Create(Pos2D.x, Pos2D.y);
        }
        public static Position2D DoubleTupleToPos2D(Tuple<double, double> Pos)
        {
            return new Position2D(Pos.Item1, Pos.Item2);
        }
        public static Vector3 DoubleTupleToVector3(Tuple<double, double> Pos)
        {
            return new Vector3((float)Pos.Item1, 0, (float)Pos.Item2);
        }
        public static Vector3 Pos2DToVector3(Position2D Pos2D)
        {
            return new Vector3((float)Pos2D.x, 0, (float)Pos2D.y);
        }
    }
}
