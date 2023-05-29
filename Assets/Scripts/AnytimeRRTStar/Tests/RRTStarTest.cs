using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Threading;
using Common;
using ARRTStar;
using System.Linq;

public class RRTStarTest : MonoBehaviour
{

    [SerializeField] private Transform agent;
    private Thread rrtStarThread;
    private bool endThreadFlag;
    private Vector3 updatedAgentPos;
    private Vector3 newRRTPoint;
    private bool newRRTPointFlag;
    private Vector3[] lineToBestNeighbour;
    private bool newRRTLineFlag;
    List<Tuple<double, double>> path;
    private List<Tuple<double, double>> finalPath;
    private bool finalPathFlag;
    [SerializeField] private Material blackMat;
    [SerializeField] private Material greyMat;
    [SerializeField] private Material redMat;
    [SerializeField] private Material greenMat;
    [SerializeField] private Material yellowMat;

    void Start()
    {
        rrtStarThread = new Thread(run);
        rrtStarThread.Start();
        EventBroker.AgentPosUpdate += handlePathVisualUpdate;
        EventBroker.RRTNewPoint += handleNewPointUpdate;
        lineToBestNeighbour = new Vector3[2];
        EventBroker.LineToBestNeighbour += handleLineToBestNeighbour;
    }

    private void handlePathVisualUpdate(object sender, EventBroker.AgentPosUpdateEventArgs e)
    {
        updatedAgentPos = e.AgentPos;
    }

    private void handleNewPointUpdate(object sender, EventBroker.NewPointEventArgs e)
    {
        newRRTPointFlag = true;
        newRRTPoint = e.NewPoint;
    }

    private void handleLineToBestNeighbour(object sender, EventBroker.LineToBestNeighbourEventArgs e)
    {
        newRRTLineFlag = true;
        lineToBestNeighbour[0] = e.NewPoint;
        lineToBestNeighbour[1] = e.BestNeighbour;
    }

    private void run()
    {
        run_RRT_Star(2000);
    }

    private void Update()
    {
        agent.position = updatedAgentPos;
        if (newRRTPointFlag)
        {
            GameObject newRRTPointGo = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            newRRTPointGo.GetComponent<MeshRenderer>().material = greyMat;
            newRRTPointGo.transform.position = newRRTPoint;
            newRRTPointFlag = false;
        }
        if (newRRTLineFlag)
        {
            GameObject rrtLineGo = new GameObject("RRT_Line");
            LineRenderer rrtLineRender = rrtLineGo.AddComponent<LineRenderer>();
            rrtLineRender.material = blackMat;
            rrtLineRender.widthMultiplier = 0.1f;
            rrtLineRender.positionCount = 2;
            rrtLineRender.SetPositions(lineToBestNeighbour);
            newRRTLineFlag = false;
        }
        if (finalPathFlag)
        {
            displayFinalPath();
            finalPathFlag = false;
        }
    }

    private void run_RRT_Star(int num_steps)
    {
        RRTStar rrt = new RRTStar(RRTConfig.AGENT_START_POSITION, RRTConfig.GOAL_POSITION);

        for (int i=0; i < num_steps; i++)
        {
            if (endThreadFlag) { break; }
            if (i % 500 == 0)
            {
                Debug.Log(i + "steps completed.");
            }
            rrt.step();
        }

        path = rrt.best_path_to_goal();
        path.Reverse();
        finalPath = path;
        finalPathFlag = true;
        Thread.Sleep(1000);
        rrt.move_along_path_until_done(path);
        Debug.Log("Done!");
    }

    private void printPath(List<Tuple<double, double>> path)
    {
        foreach(var p in path)
        {
            float x = (float) p.Item1;
            float y = (float) p.Item2;
            Debug.Log("p: " + x + " , " + y);
        }
    }

    private void displayFinalPath()
    {
        List<Vector3> pathPoints = new List<Vector3>();
        lock (path)
        {
            foreach (Tuple<double, double> p in path)
            {
                if (p == null) { continue; }
                Vector3 pos = MathHelpers.DoubleTupleToVector3(p);
                pathPoints.Add(pos);
            }
        }
        GameObject finalPathGo = new GameObject("FinalPath");
        LineRenderer finalPathRender = finalPathGo.AddComponent<LineRenderer>();
        finalPathRender.material = greenMat;
        finalPathRender.widthMultiplier = 0.2f;
        finalPathRender.positionCount = pathPoints.Count;
        finalPathRender.SetPositions(pathPoints.ToArray());
    }

    private void OnApplicationQuit()
    {
        endThreadFlag = true;
        rrtStarThread.Abort();
    }

}
