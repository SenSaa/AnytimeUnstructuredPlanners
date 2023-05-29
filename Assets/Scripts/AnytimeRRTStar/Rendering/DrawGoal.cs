using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ARRTStar;

public class DrawGoal
{
    public void Draw(Material material)
    {
        processGoalPosition(material);
    }

    private void processGoalPosition(Material material)
    {
        Position2D[] goalPosition = RRTConfig.GOAL_POSITION;
        Position2D diagonalCorner1 = new Position2D(goalPosition[0].x, goalPosition[0].y); // One of the digonal rect corners e.g. bottom left
        Position2D diagonalCorner2 = new Position2D(goalPosition[1].x, goalPosition[1].y); // The other digonal rect corner e.g. top right
        double deltaX = (diagonalCorner2.x - diagonalCorner1.x);
        double deltaY = (diagonalCorner2.y - diagonalCorner1.y);
        double centreX = deltaX / 2;
        double centreY = deltaY / 2;
        Position2D centerPosition = new Position2D(diagonalCorner1.x + centreX, diagonalCorner1.y + centreY);
        Vector3 position = MathHelpers.Pos2DToVector3(centerPosition);
        double width = deltaY;
        double length = deltaX;
        renderGoalPosition(position, width, length, material);
    }

    private void renderGoalPosition(Vector3 position, double width, double length, Material material)
    {
        GameObject goalObject = GameObject.CreatePrimitive(PrimitiveType.Cube);
        goalObject.transform.position = position;
        goalObject.transform.localScale = new Vector3((float)length, 1, (float)width);
        goalObject.GetComponent<MeshRenderer>().material = material;
    }
}
