using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public enum Material
{
    ice,
    steel,
    rubber,
    plastic
}
public class Lab8PhysicsObjects : MonoBehaviour
{
    public float mass = 1.0f;
    public Vector3 velocity = Vector3.zero;
    public PhysicsCollider shape = null;
    public float Bounciness = 0.0f;
    public Material FrcitionType;

    private float friction = 0.0f;
    public bool LockPosition = false;
    // Start is called before the first frame update
    void Start()
    {
        FindObjectOfType<Lab8PhysicsSystem>().lab8Physics.Add(this);
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        setFriction();
    }

    public float getFriction()
    {
        return friction;
    }
    void setFriction()
    {
        if (FrcitionType == Material.ice)
        {
            friction = 0.0f;
        }
        if (FrcitionType == Material.steel)
        {
            friction = 0.5f;
        }
        if (FrcitionType == Material.steel)
        {
            friction = 0.9f;
        }
        if (FrcitionType == Material.plastic)
        {
            friction = 1.0f;
        }
    }
}
