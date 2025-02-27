using UnityEngine;
using System.Collections.Generic;
public class LogManager : MonoBehaviour
{
    public static LogManager Instance { get; private set; }
    private List<string> logMessages = new List<string>();
    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }
    public void Log(string message)
    {
        logMessages.Add(message);
        Debug.Log(message);
    }
    public List<string> GetLogMessages()
    {
        return logMessages;
    }
}