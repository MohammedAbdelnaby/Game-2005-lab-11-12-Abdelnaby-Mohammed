using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using UnityEngine;

public class PhysicsColliderSphere : PhysicsCollider
{
    public float raduis = 1;
    private CollistionShape shapeType = CollistionShape.Sphere;

    public override CollistionShape GetCollistionShape()
    {
        return shapeType;
    }

    public float getRaduis()
    {
        return raduis;
    }
}
