using UnityEngine;
using UnityEngine.UI;
using TMPro;
public class LogDisplay : MonoBehaviour
{
    [SerializeField] private TMP_Text logText;
    [SerializeField] private int maxDisplayedMessages = 20;
    private void Update()
    {
        DisplayLog();
    }
    private void DisplayLog()
    {
        var logMessages = LogManager.Instance.GetLogMessages();
        int startIdx = Mathf.Max(0, logMessages.Count - maxDisplayedMessages);
        string displayedLog = "";
        for (int i = startIdx; i < logMessages.Count; i++)
        {
            displayedLog += logMessages[i] + "\n";
        }
        logText.text = displayedLog;
    }
}