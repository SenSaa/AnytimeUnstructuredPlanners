using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;

namespace Common
{

    public class EventBroker
    {

        // ----------------------------------------
        // - Define Dynamic path event.
        public static event EventHandler<DynamicPathEventArgs> DynamicPath;
        // ----------------------------------------

        public static event EventHandler<AgentPosUpdateEventArgs> AgentPosUpdate;
        public static event EventHandler<RandomPointEventArgs> RRTRandomPoint;
        public static event EventHandler<NewPointEventArgs> RRTNewPoint;
        public static event EventHandler<LineToBestNeighbourEventArgs> LineToBestNeighbour;


        // ----------------------------------------
        // - Define dynamic path event publisher.
        public static void InvokeDynamicPath(object sender, DynamicPathEventArgs e)
        {
            DynamicPath?.Invoke(sender, e);
        }
        // ----------------------------------------

        public static void InvokeAgentPosUpdate(object sender, AgentPosUpdateEventArgs e)
        {
            AgentPosUpdate?.Invoke(sender, e);
        }

        public static void InvokeRandomPointUpdate(object sender, RandomPointEventArgs e)
        {
            RRTRandomPoint?.Invoke(sender, e);
        }

        public static void InvokeNewPointUpdate(object sender, NewPointEventArgs e)
        {
            RRTNewPoint?.Invoke(sender, e);
        }

        public static void InvokeLineToBestNeighbour(object sender, LineToBestNeighbourEventArgs e)
        {
            LineToBestNeighbour?.Invoke(sender, e);
        }


        // ----------------------------------------
        // - Define dynamic path event object.
        public class DynamicPathEventArgs
        {
            public List<Vector3> DynamicPathV3 { set; get; }
            public List<Tuple<double, double>> DynamicPathTuple { set; get; }
            public List<Tuple<float, float>> DynamicPathTupleFloat { set; get; }
        }
        // ----------------------------------------

        public class AgentPosUpdateEventArgs
        {
            public Vector3 AgentPos { set; get; }
        }

        public class RandomPointEventArgs
        {
            public Vector3 RandomPoint { set; get; }
        }

        public class NewPointEventArgs
        {
            public Vector3 NewPoint { set; get; }
        }

        public class LineToBestNeighbourEventArgs
        {
            public Vector3 NewPoint { set; get; }
            public Vector3 BestNeighbour { set; get; }
        }

    }

}