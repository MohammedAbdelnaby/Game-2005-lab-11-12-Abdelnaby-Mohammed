using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum CollistionShape
{
    Sphere,
    Plane,
    AABB
}

struct CollisionInfo
{
    public PhysicsCollider objectA;
    public PhysicsCollider objectB;
    public Vector3 CollisionNormalAtoB;
    public Vector3 ContactPoint;
}

public abstract class PhysicsCollider : MonoBehaviour
{

    public abstract CollistionShape GetCollistionShape();
    public Lab8PhysicsObjects KinematicsObject;

    public void Start()
    {
        KinematicsObject = GetComponent<Lab8PhysicsObjects>();
        FindObjectOfType<Lab8PhysicsSystem>().ColliderShapes.Add(this);
    }
}
