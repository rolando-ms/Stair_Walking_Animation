using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NoiseMenu : MonoBehaviour
{
    public bool ResetToOriginal = false;
    public bool UseEvenNoise = false;

    [Tooltip("0 = No noise, 1 = Height noise, 2 = Width noise, 3 = Height/Width noise")]
	[Range(0,3)]
	public int NoiseType = 0;
}
