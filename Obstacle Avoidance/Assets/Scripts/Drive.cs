using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Drive : MonoBehaviour
{
    Rigidbody2D rb;

    public float speed = 10.0f;
    public float rotationSpeed = 100.0f;
    public float currentSpeed = 0;

    private void Start()
    {
        rb = this.GetComponent<Rigidbody2D>();
    }

    void FixedUpdate()
    {
        // Get the horizontal and vertical axis.
        // By default they are mapped to the arrow keys.
        // The value is in the range -1 to 1
        float translation = Input.GetAxis("Vertical") * speed;
        float rotation = Input.GetAxis("Horizontal") * rotationSpeed;

        // Make it move 10 meters per second instead of 10 meters per frame...
        translation *= Time.fixedDeltaTime;
        rotation *= Time.fixedDeltaTime;

        // Move translation along the object's z-axis
        //transform.Translate(0, 0, translation);

        //currentSpeed = translation;
        rb.velocity = (Vector2)rb.transform.up * translation;

        // Rotate around our y-axis
        //transform.Rotate(0, rotation, 0);
        rb.rotation -= rotation;
    }
}
