using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class HelloWorld : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        LogManager.Instance.Log("Hello World!");

        // Fill 19 additional lines to give a sense of space taken on screen.
        for (int i = 1; i <= 19; i++)
        {
            LogManager.Instance.Log("Line " + i);
        }
    }
}