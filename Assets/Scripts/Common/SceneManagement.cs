using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneManagement : MonoBehaviour
{

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void AnytimeDStarOnClick()
    {
        SceneManager.LoadScene("AnytimeDStar", LoadSceneMode.Single);
    }

    public void AnytimeRRTStarOnClick()
    {
        SceneManager.LoadScene("AnytimeRRTStar", LoadSceneMode.Single);
    }

    public void MainMenuOnClick()
    {
        SceneManager.LoadScene("MainMenu", LoadSceneMode.Single);
    }


}
