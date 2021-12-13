using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PhysicsColliderAABB : PhysicsCollider
{

    public Vector3 Dimensions = new Vector3(1, 1, 1);

    public Vector3 GetMin()
    {
        return transform.position - GetHalfSize();

    }

    public Vector3 GetMax()
    {
       return transform.position + GetHalfSize();
    }

    public Vector3 GetSize()
    {
        return Vector3.Scale(Dimensions, transform.lossyScale);
    }

    public Vector3 GetHalfSize()
    {
        return Vector3.Scale(Dimensions, transform.lossyScale) * 0.5f;

    }
    public override CollistionShape GetCollistionShape()
    {
        return CollistionShape.AABB;
    }
}
