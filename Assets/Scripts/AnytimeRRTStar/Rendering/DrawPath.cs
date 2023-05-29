using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using ARRTStar;

public class DrawPath
{

    private GameObject PathRendererGameObject;
    private LineRenderer FinalPathRender;

    public DrawPath(GameObject pathRendererGameObject)
    {
        this.PathRendererGameObject = pathRendererGameObject;
        FinalPathRender = pathRendererGameObject.AddComponent<LineRenderer>();
    }

    public void Draw(List<Tuple<double,double>> pathPoints, Material mat)
    {
        List<Vector3> points = processPath(pathPoints);
        renderPath(points, mat);
    }

    private List<Vector3> processPath(List<Tuple<double,double>> pathPoints)
    {
        List<Vector3> path = new List<Vector3>();
        foreach (Tuple<double, double> p in pathPoints)
        {
            Vector3 pos = MathHelpers.DoubleTupleToVector3(p);
            path.Add(pos);
        }
        return path;
    }

    private void renderPath(List<Vector3> points, Material mat)
    {
        FinalPathRender.widthMultiplier = 0.2f;
        FinalPathRender.positionCount = points.Count;
        FinalPathRender.SetPositions(points.ToArray());
        FinalPathRender.material = mat;
    }
}
