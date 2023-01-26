using UnityEngine;
using UnityEngine.SceneManagement;

public class Init : MonoBehaviour
{
    public void GoToSim()
    {
        SceneManager.LoadScene("MovementTest");
    }
}
