using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
//using Utils;
using Rendering;
using ARRTStar;
using Common;

public class AnytimeRRTStarTest : MonoBehaviour
{

    [SerializeField] private Transform agent;
    AnytimeRRTStar arrt;
    private Thread arrtThread;
    private bool arrtThreadRunning;

    List<Tuple<double, double>> trajectory;
    List<Tuple<double, double>> path;
    [SerializeField] private bool pathFlag;

    private Vector3 updatedAgentPos;

    DrawPath drawPath;

    [SerializeField] private Material obstacleMaterial;
    [SerializeField] private Material goalMaterial;
    [SerializeField] private Material pathMaterial;

    [SerializeField] private Vector3 mousePos;
    [SerializeField] private Vector3 goalWorldPosition;


    void Start()
    {
        arrtThreadRunning = true;
        arrtThread = new Thread(run);
        arrtThread.Start();
        EventBroker.AgentPosUpdate += handlePathVisualUpdate;

        StartCoroutine(agentMovement());

        DrawObstacles drawObstacles = new DrawObstacles();
        drawObstacles.Draw(obstacleMaterial);
        DrawGoal drawGoal = new DrawGoal();
        drawGoal.Draw(goalMaterial);
        GameObject finalPathGo = new GameObject("FinalPath"); // *
        drawPath = new DrawPath(finalPathGo);
    }

    private void handlePathVisualUpdate(object sender, EventBroker.AgentPosUpdateEventArgs e)
    {
        updatedAgentPos = e.AgentPos;
    }

    void Update()
    {
        if (pathFlag)
        {
            pathFlag = false;
            drawPath.Draw(path, pathMaterial);
        }

        mouseClick();
    }

    private void run()
    {
        run_rrt_star();
    }

    private void run_rrt_star()
    {
        arrt = new AnytimeRRTStar(RRTConfig.AGENT_START_POSITION, RRTConfig.GOAL_POSITION);
        trajectory = null;

        int prevPathCount = 0;

        while (arrtThreadRunning)
        {
            trajectory = arrt.best_path_to_goal();
            if (trajectory != null && trajectory.Count > RRTConfig.ANYTIME_TRAJECTORY_LENGTH - 1)
            {
                trajectory.RemoveRange((int)RRTConfig.ANYTIME_TRAJECTORY_LENGTH, trajectory.Count - (int)RRTConfig.ANYTIME_TRAJECTORY_LENGTH);
            }

            // Keep stepping the RRT until we find the goal.
            if (trajectory == null)
            {
                arrt.step_until_found_goal();
            }
            trajectory = arrt.best_path_to_goal();
            if (trajectory != null && trajectory.Count > RRTConfig.ANYTIME_TRAJECTORY_LENGTH - 1)
            {
                trajectory.RemoveRange((int)RRTConfig.ANYTIME_TRAJECTORY_LENGTH, trajectory.Count - (int)RRTConfig.ANYTIME_TRAJECTORY_LENGTH);
            }
            
            // Once we've set the current trajectory, prune the tree and set the new root.
            arrt.prune_tree_and_set_new_root(trajectory);
            if (arrt.redraw_path_to_goal() != null)
            {
                path = arrt.pathPoints;
            }

            if (path.Count != prevPathCount)
            {
                pathFlag = true;
                prevPathCount = path.Count;
            }

            // Move the agent along the trajectory, continuing to expand the RRT as we do so.
            trajectory.Reverse();
            arrt.move_along_path_until_done(trajectory, true, false);

            EventBroker.InvokeDynamicPath(this, new EventBroker.DynamicPathEventArgs
            {
                DynamicPathTuple = path
            });

            Thread.Sleep(100);
        }
    }


    IEnumerator agentMovement()
    {
        while (true)
        {
            agent.position = updatedAgentPos;
            yield return new WaitForSeconds(0.02f);
        }
    }

    // -------------------------------------------------------------------------------------

    // * Mouse Click -> World Pos -> New Goal Pos.

    private void mouseClick()
    {
        if(Input.GetMouseButtonUp(0)) // Left mouse click event
        {
            Vector3 goalWorldPos = mousePositionToWorldPosition();
            Position2D startPos = setNewStartPos(agent.transform.position); // *
            Position2D[] goalPos = setNewGoalPos(goalWorldPos);

            arrt = new AnytimeRRTStar(startPos, goalPos);

            renderGoal(goalWorldPos);
        }
    }

    private Vector3 mousePositionToWorldPosition()
    {
        mousePos = Input.mousePosition;
        mousePos.z = Camera.main.transform.position.y; // select distance = 100 units from the camera (this is actually u/down direction)
        goalWorldPosition = Camera.main.ScreenToWorldPoint(mousePos);
        goalWorldPosition.y = 0;
        return goalWorldPosition;
    }

    private Position2D setNewStartPos(Vector3 agentWorldPos)
    {
        Position2D agentPos = new Position2D(agentWorldPos.x, agentWorldPos.z);
        ///arrt.agent_pos = agentPos;
        return agentPos;
    }

    private Position2D[] setNewGoalPos(Vector3 goalWorldPos)
    {
        Vector3 goalPosCorner1 = new Vector3(goalWorldPos.x - 4, 0, goalWorldPos.z - 4);
        Vector3 goalPosCorner2 = new Vector3(goalWorldPos.x + 4, 0, goalWorldPos.z + 4);
        Position2D[] goalPos = new Position2D[] {
            new Position2D(goalPosCorner1.x, goalPosCorner1.z),
            new Position2D(goalPosCorner2.x, goalPosCorner2.z)
        };
        return goalPos;
    }

    private void renderGoal(Vector3 position)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = "Goal";
        go.transform.position = position;
        go.transform.localScale = new Vector3(4, 0.1f, 4);
        go.GetComponent<MeshRenderer>().material = goalMaterial;
    }

    // -------------------------------------------------------------------------------------

    private void OnApplicationQuit()
    {
        arrtThread.Abort();
        arrtThreadRunning = false;
    }

}
