using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Lab8PhysicsSystem : MonoBehaviour
{
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public List<Lab8PhysicsObjects> lab8Physics = new List<Lab8PhysicsObjects>();

    public List<PhysicsCollider> ColliderShapes;
    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        for (int i = 0; i < lab8Physics.Count; i++)
        {
            if (!lab8Physics[i].LockPosition)
            {
                lab8Physics[i].velocity += gravity * Time.fixedDeltaTime;
                lab8Physics[i].transform.position = lab8Physics[i].transform.position + lab8Physics[i].velocity * Time.deltaTime;

            }
        }

        CollisionUpdate();
    }
    void CollisionUpdate()
    {
        for (int i = 0; i < lab8Physics.Count; i++)
        {
            for (int j = i + 1; j < lab8Physics.Count; j++)
            {
                Lab8PhysicsObjects ObjectA = lab8Physics[i];
                Lab8PhysicsObjects ObjectB = lab8Physics[j];

                Vector3 ObjectAPosition = ObjectA.transform.position;
                Vector3 ObjectBPosition = ObjectB.transform.position;

                if (ObjectA.shape == null || ObjectB.shape == null)
                {
                    continue;
                }

                if (ObjectA.shape.GetCollistionShape() == CollistionShape.Sphere
                    && ObjectB.shape.GetCollistionShape() == CollistionShape.Sphere)
                {
                    SphereSphereCollition((PhysicsColliderSphere)ObjectA.shape, (PhysicsColliderSphere)ObjectB.shape);
                }

                if (ObjectA.shape.GetCollistionShape() == CollistionShape.AABB
                    && ObjectB.shape.GetCollistionShape() == CollistionShape.AABB)
                {
                    AABBAABBCollision((PhysicsColliderAABB)ObjectA.shape, (PhysicsColliderAABB)ObjectB.shape);
                }
                if (ObjectB.shape.GetCollistionShape() == CollistionShape.AABB
                    && ObjectA.shape.GetCollistionShape() == CollistionShape.AABB)
                {
                    AABBAABBCollision((PhysicsColliderAABB)ObjectB.shape, (PhysicsColliderAABB)ObjectA.shape);
                }

                if (ObjectA.shape.GetCollistionShape() == CollistionShape.Sphere
                    && ObjectB.shape.GetCollistionShape() == CollistionShape.AABB)
                {
                    SphereAABBCollision((PhysicsColliderSphere)ObjectA.shape, (PhysicsColliderAABB)ObjectB.shape);
                }
                if (ObjectB.shape.GetCollistionShape() == CollistionShape.Sphere
                    && ObjectA.shape.GetCollistionShape() == CollistionShape.AABB)
                {
                    SphereAABBCollision((PhysicsColliderSphere)ObjectB.shape, (PhysicsColliderAABB)ObjectA.shape);
                }

                if (ObjectA.shape.GetCollistionShape() == CollistionShape.Sphere
                    && ObjectB.shape.GetCollistionShape() == CollistionShape.Plane)
                {
                    if (SpherePlaneCollision((PhysicsColliderSphere)ObjectA.shape, (PhysicsColliderPlane)ObjectB.shape))
                    {
                        Debug.Log(ObjectA.name + " and " + ObjectB.name + " collided");
                        Color colorSphere = ObjectA.GetComponent<Renderer>().material.color;
                        Color colorPlane = ObjectB.GetComponent<Renderer>().material.color;
                        ObjectA.GetComponent<Renderer>().material.color = Color.Lerp(colorSphere, colorPlane, 0.05f);
                        ObjectB.GetComponent<Renderer>().material.color = Color.Lerp(colorPlane, colorSphere, 0.05f);
                        ObjectA.LockPosition = true;
                    }


                }


                if (ObjectA.shape.GetCollistionShape() == CollistionShape.Plane
                    && ObjectB.shape.GetCollistionShape() == CollistionShape.Sphere)
                {
                    if (SpherePlaneCollision((PhysicsColliderSphere)ObjectB.shape, (PhysicsColliderPlane)ObjectA.shape))
                    {
                        Debug.Log(ObjectB.name + " and " + ObjectA.name + " collided");
                        Color colorSphere = ObjectB.GetComponent<Renderer>().material.color;
                        Color colorPlane = ObjectA.GetComponent<Renderer>().material.color;
                        ObjectB.GetComponent<Renderer>().material.color = Color.Lerp(colorPlane, colorSphere, 0.05f);
                        ObjectA.GetComponent<Renderer>().material.color = Color.Lerp(colorSphere, colorPlane, 0.05f);
                        ObjectB.LockPosition = true;
                    }
                
                }

            }
        }
    }



    void SphereSphereCollition(PhysicsColliderSphere sphere, PhysicsColliderSphere sphere1)
    {
        Vector3 displacement = sphere1.transform.position - sphere.transform.position;
        float distance = displacement.magnitude;
        float sumRadii = sphere.raduis + sphere1.raduis;
        float penetrationDepth = sumRadii - distance;
        bool IsOverLapping = penetrationDepth > 0;
        if (IsOverLapping)
        {
            Debug.Log("YEs");
        }
        else
        {
            return;
        }

        Vector3 CollisionNormalAtoB;
        CollisionNormalAtoB = displacement / distance;
        Vector3 minimumTranslationVectorAtoB = penetrationDepth * CollisionNormalAtoB * 0.5f;

        ApplyMinimumTranlationVector(sphere.KinematicsObject, sphere1.KinematicsObject, minimumTranslationVectorAtoB, CollisionNormalAtoB,(sphere.transform.position + CollisionNormalAtoB * sphere.raduis));
    }

    static bool SpherePlaneCollision(PhysicsColliderSphere sphere, PhysicsColliderPlane plane)
    {
        Vector3 PointonOnPlane = plane.transform.position;
        Vector3 CenterOfSphere = sphere.transform.position;
        Vector3 FromPlaneToSphere = CenterOfSphere - PointonOnPlane;
        float dot = Vector3.Dot(FromPlaneToSphere, plane.getNormal());
        float Distance = Mathf.Abs(dot);
        float penetrationdepth = sphere.getRaduis() - Distance;
        bool isOverLapping = penetrationdepth > 0;
        return isOverLapping;
    }

    void SphereAABBCollision(PhysicsColliderSphere sphere, PhysicsColliderAABB Box)
    {
        float raduis = sphere.raduis;
        Vector3 HalfSize2 = Box.GetHalfSize();

        Vector3 DisplacemetAB = Box.transform.position - sphere.transform.position;
        float PenetrationX = (raduis + HalfSize2.x - Math.Abs(DisplacemetAB.x));
        float PenetrationY = (raduis + HalfSize2.y - Math.Abs(DisplacemetAB.y));
        float PenetrationZ = (raduis + HalfSize2.z - Math.Abs(DisplacemetAB.z));
        Vector3 Normal = new Vector3(Mathf.Sign(DisplacemetAB.x), 0.0f, 0.0f); ;
        Vector3 MiniumTranslationVectorAtoB = Normal * PenetrationX;

        Vector3 ContactPoint;
        //Debug.Log(PenetrationX + " , " + PenetrationY + " , " + PenetrationZ);
        if (PenetrationX < 0 || PenetrationY < 0 || PenetrationZ < 0)
        {
            return;
        }

        if (PenetrationY < PenetrationX && PenetrationY < PenetrationZ) //if y is the shortest
        {
            Normal = new Vector3(0.0f, Mathf.Sign(DisplacemetAB.y), 0.0f);
            MiniumTranslationVectorAtoB = Normal * PenetrationY;
        }
        else if (PenetrationZ < PenetrationY && PenetrationZ < PenetrationX) //if z is the shortest
        {
            Normal = new Vector3(0.0f, 0.0f, Mathf.Sign(DisplacemetAB.z));
            MiniumTranslationVectorAtoB = Normal * PenetrationZ;
        }

        ContactPoint = sphere.transform.position + MiniumTranslationVectorAtoB;
        ApplyMinimumTranlationVector(sphere.KinematicsObject, Box.KinematicsObject, MiniumTranslationVectorAtoB, Normal, ContactPoint);
    }

    void AABBAABBCollision(PhysicsColliderAABB Box1, PhysicsColliderAABB Box2)
    {
        Vector3 HalfSize1 = Box1.GetHalfSize();
        Vector3 HalfSize2 = Box2.GetHalfSize();

        Vector3 DisplacemetAB = Box2.transform.position - Box1.transform.position;
        float PenetrationX = (HalfSize1.x + HalfSize2.x - Math.Abs(DisplacemetAB.x));
        float PenetrationY = (HalfSize1.y + HalfSize2.y - Math.Abs(DisplacemetAB.y));
        float PenetrationZ = (HalfSize1.z + HalfSize2.z - Math.Abs(DisplacemetAB.z));
        Vector3 Normal = new Vector3(Mathf.Sign(DisplacemetAB.x), 0.0f, 0.0f); ;
        Vector3 MiniumTranslationVectorAtoB = Normal * PenetrationX;

        Vector3 ContactPoint;
        //Debug.Log(PenetrationX + " , " + PenetrationY + " , " + PenetrationZ);
        if (PenetrationX < 0 || PenetrationY < 0 || PenetrationZ < 0)
        {
            return;
        }

        if (PenetrationY < PenetrationX && PenetrationY < PenetrationZ) //if y is the shortest
        {
            Normal = new Vector3(0.0f, Mathf.Sign(DisplacemetAB.y), 0.0f);
            MiniumTranslationVectorAtoB = Normal * PenetrationY;
        }
        else if (PenetrationZ < PenetrationY && PenetrationZ < PenetrationX) //if z is the shortest
        {
            Normal = new Vector3(0.0f, 0.0f, Mathf.Sign(DisplacemetAB.z));
            MiniumTranslationVectorAtoB = Normal * PenetrationZ;
        }

        ContactPoint = Box1.transform.position + MiniumTranslationVectorAtoB;
        ApplyMinimumTranlationVector(Box1.KinematicsObject, Box2.KinematicsObject, MiniumTranslationVectorAtoB, Normal, ContactPoint);
    }

    void ApplyMinimumTranlationVector(Lab8PhysicsObjects object1, Lab8PhysicsObjects object2, Vector3 miniumTranslationVectorAtoB, Vector3 normal, Vector3 contactPoint)
    {
        ComputeMovementScalars(object1, object2, out float ScalerA, out float ScalerB);
        CollisionInfo collisionInfo;
        collisionInfo.objectA = object1.shape;
        collisionInfo.objectB = object2.shape;
        collisionInfo.CollisionNormalAtoB = normal;
        collisionInfo.ContactPoint = contactPoint;

        Vector3 TranslationVectorA = -miniumTranslationVectorAtoB * ScalerA;
        Vector3 TranslationVectorB =  miniumTranslationVectorAtoB * ScalerB;
        //Debug.Log(ScalerB);
        object1.transform.Translate(TranslationVectorA);
        object2.transform.Translate(TranslationVectorB);
        KinematicsCollisionResponse(collisionInfo);
    }

    void KinematicsCollisionResponse(CollisionInfo info)
    {
        
        Lab8PhysicsObjects objectA = info.objectA.KinematicsObject;
        Lab8PhysicsObjects objectB = info.objectB.KinematicsObject;

        Vector3 RelativeVelocityAtoB = objectB.velocity - objectA.velocity;
        float RelativeNormalVelocityAtoB = Vector3.Dot(RelativeVelocityAtoB, info.CollisionNormalAtoB);
        if (RelativeNormalVelocityAtoB >= 0.0f)
        {
            return;
        }
        float Restitution = 0.5f * (objectA.Bounciness + objectB.Bounciness);
        float ChangeInVelocity;
        float minimumRelativeVelocityForBounce = 3.0f;

        if (RelativeNormalVelocityAtoB < -minimumRelativeVelocityForBounce)
        {
            ChangeInVelocity = (RelativeNormalVelocityAtoB * (1.0f + Restitution));
        }
        else
        {
            ChangeInVelocity = (RelativeNormalVelocityAtoB);
        }


        float impulse;

        if (objectA.LockPosition && !objectB.LockPosition)
        {
            impulse = -ChangeInVelocity * objectB.mass;
            objectB.velocity += info.CollisionNormalAtoB * (impulse / (objectB.mass));
        }
        else if (!objectA.LockPosition && objectB.LockPosition)
        {
            impulse = -ChangeInVelocity * objectA.mass;
            objectA.velocity -= info.CollisionNormalAtoB * (impulse / (objectA.mass));
        }
        else if (!objectA.LockPosition && !objectB.LockPosition)
        {
            impulse = ChangeInVelocity / ((1.0f / objectA.mass) + (1.0f / objectB.mass));
            objectA.velocity += info.CollisionNormalAtoB * (impulse / objectA.mass);
            objectB.velocity -= info.CollisionNormalAtoB * (impulse / objectB.mass);
        }

        Vector3 relativeSurfaceVelocity = RelativeVelocityAtoB - (RelativeNormalVelocityAtoB * info.CollisionNormalAtoB);
        applyFriction(info, relativeSurfaceVelocity);
    }

    static void ComputeMovementScalars(Lab8PhysicsObjects objects1, Lab8PhysicsObjects objects2, out float ScalarA, out float ScalarB)
    {
        if (objects1.LockPosition && !objects2.LockPosition)
        {
            ScalarA = 0.0f;
            ScalarB = 1.0f;
            return;
        }
        if (!objects1.LockPosition && objects2.LockPosition)
        {
            ScalarA = 1.0f;
            ScalarB = 0.0f;
            return;
        }
        if (!objects1.LockPosition && !objects2.LockPosition)
        {
            ScalarA = 0.5f;
            ScalarB = 0.5f;
            return;
        }
        ScalarA = 0.0f;
        ScalarB = 0.0f;
    }

    void applyFriction(CollisionInfo collision, Vector3 relativeSurfaceVelocity)
    {
        Lab8PhysicsObjects object1 = collision.objectA.KinematicsObject;
        Lab8PhysicsObjects object2 = collision.objectB.KinematicsObject;
        float relativeSpeed = relativeSurfaceVelocity.magnitude;
        if (relativeSpeed < 0.001f)
        {
            return;
        }

        Vector3 dirctionofFriction = relativeSurfaceVelocity / relativeSpeed;
        float gravityDotNormal = Vector3.Dot(gravity, collision.CollisionNormalAtoB);
        float FrictionCoefficent = (object1.getFriction() + object2.getFriction()) * 0.5f;
        Vector3 accelarationFriction = gravityDotNormal * FrictionCoefficent * dirctionofFriction;
        if (!object1.LockPosition)
        {
            object1.velocity += accelarationFriction * Time.fixedDeltaTime; 
        }
        if (!object2.LockPosition)
        {
            object2.velocity -= accelarationFriction * Time.fixedDeltaTime;
        }
    }
}
