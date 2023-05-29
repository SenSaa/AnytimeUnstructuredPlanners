using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RectObstacle
{

    public Position2D pos2D;
    public double length;
    public double width;

    /// <summary>
    /// Rectangular shape: With bottom left corner 2D pos, length and width.
    /// </summary>
    /// <param name="x"> bottom left corner x </param>
    /// <param name="y"> bottom left corner y </param>
    /// <param name="length"> length of rectangle </param>
    /// <param name="width"> width of rectangle </param>
    public RectObstacle(Position2D pos2D, double length, double width)
    {
        this.pos2D = pos2D;
        this.length = length;
        this.width = width;
    }

}
