using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ADStar;
using Visualisation;

public class RunAnytimeDStar : MonoBehaviour
{

    private AnytimeDStar dstar;
    private Tuple<float, float> s_start;
    private Tuple<float, float> s_goal;
    private HashSet<Tuple<float, float>> Obs;
    [SerializeField] private Material goalMaterial;
    [SerializeField] private Material obstacleMaterial;
    [SerializeField] private Material pathMaterial;
    [SerializeField] private Material pathPointsMaterial;
    [SerializeField] private Material visitedMaterial;
    [SerializeField] private Material obstacleRemovedMaterial;
    [SerializeField] private Vector3 mousePos;
    [SerializeField] private Vector3 goalWorldPosition;

    void Start()
    {
        // Define start and goal states.
        s_start = Tuple.Create(5f, 5f);
        s_goal = Tuple.Create(45f, 25f);
        double eps = 2.5;
        string heuristic_type = "euclidean";
        Obs = new HashSet<Tuple<float, float>>();
        // Run AD*.
        runDStar(s_start, s_goal, eps, heuristic_type);

        List<Tuple<float, float>> path = dstar.getPath();

        // Visualise Obstacles
        visualiseObstacles();

        // Visualise Path
        visualisePath(path);
    }

    void Update()
    {
        mouseClicks();
    }


    private void runDStar(Tuple<float, float> s_start, Tuple<float, float> s_goal, double eps, string heuristic_type)
    {
        dstar = new AnytimeDStar(s_start, s_goal, eps, heuristic_type, Obs);
        dstar.run();
    }


    private void visualisePath(List<Tuple<float, float>> path)
    {
        deletePreviousPath();
        HashSet<Tuple<float, float>> visitedStates = dstar.getVisitedStates();
        RenderPath renderingPath = new RenderPath();
        renderingPath.Draw(path, visitedStates, transform, pathMaterial, pathPointsMaterial, visitedMaterial);
    }
    private void deletePreviousPath()
    {
        if (transform.Find("PathLine") != null)
        {
            Transform previousPath = transform.Find("PathLine");
            Destroy(previousPath.gameObject);
        }
        if (transform.Find("PathPoints") != null)
        {
            Transform previousPoints = transform.Find("PathPoints");
            Destroy(previousPoints.gameObject);
        }
        if (transform.Find("VisitedPoints") != null)
        {
            Transform previousVisited = transform.Find("VisitedPoints");
            Destroy(previousVisited.gameObject);
        }
    }

    private void visualiseObstacles()
    {
        deletePreviousObstacleMap();
        HashSet<Tuple<float, float>> obs = dstar.getObs();
        RenderObstacles renderObstacles = new RenderObstacles();
        renderObstacles.Draw(obs, transform, obstacleMaterial);
    }
    private void deletePreviousObstacleMap()
    {
        if (transform.Find("Obstacles") != null)
        {
            Transform previousObstacle = transform.Find("Obstacles");
            Destroy(previousObstacle.gameObject);
        }
    }

    private void logPath(List<Tuple<float, float>> path)
    {
        string path_str = ": ";
        foreach (var point in path)
        {
            path_str += "(" + point.Item1 + ", " + point.Item2 + "), ";
        }
        Debug.Log("path" + path_str);
    }


    private void mouseClicks()
    {
        // Set new Goal for left mouse click.
        if (Input.GetMouseButtonUp(0)) // Left mouse click event
        {
            Vector3 goalWorldPos = mousePositionToWorldPosition();
            Vector3 goalPos = nearestIntPos(goalWorldPos); // Get the nearest grid cell of the goal world pos.
            renderGoal(goalPos);
            setNewGoalPos(goalPos);
            s_start = s_goal; // Move start pos to new goal after planning session.
        }

        // Add/Remove Obstacle with right mouse click (if clicked pos is obstacle remove it, if not add it).
        if (Input.GetMouseButton(1)) // Right mouse click event
        {
            Vector3 worldPos = mousePositionToWorldPosition();
            Vector3 newPos = nearestIntPos(worldPos); // Get the nearest grid cell of the goal world pos.
            renderObstacleUpdate(newPos);
            obstacleUpdatePos(newPos);
            Obs.Add(Tuple.Create(newPos.x, newPos.z));
        }
    }

    private Vector3 mousePositionToWorldPosition()
    {
        mousePos = Input.mousePosition;
        mousePos.z = 100; // select distance = 100 units from the camera (this is actually up/down direction)
        goalWorldPosition = Camera.main.ScreenToWorldPoint(mousePos);
        goalWorldPosition.y = 0;
        return goalWorldPosition;
    }

    private Vector3 nearestIntPos(Vector3 position)
    {
        return new Vector3(Mathf.Round(position.x), Mathf.Round(position.y), Mathf.Round(position.z));
    }

    private void renderGoal(Vector3 position)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = "Goal";
        go.transform.position = position;
        go.transform.localScale = new Vector3(1, 0.1f, 1);
        go.GetComponent<MeshRenderer>().material = goalMaterial;
    }

    private void renderObstacleUpdate(Vector3 position)
    {
        GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = "Obstacle";
        go.transform.position = position;
        go.transform.localScale = new Vector3(1, 0.1f, 1);
        go.GetComponent<MeshRenderer>().material = obstacleRemovedMaterial;
    }

    private void setNewGoalPos(Vector3 goalPos)
    {
        // New Goal
        s_goal = Tuple.Create(goalPos.x, goalPos.z);
        // Run DStar
        runDStar(s_start, s_goal, 2.5, "euclidean");
        // Visualise
        List<Tuple<float, float>> path = dstar.getPath();
        visualisePath(path);
        logPath(path);
    }

    private void obstacleUpdatePos(Vector3 newPos)
    {
        dstar.on_press(newPos.x, newPos.z);
        visualiseObstacles();
        List<Tuple<float, float>> path = dstar.getPath();
        visualisePath(path);
        logPath(path);
    }

}
