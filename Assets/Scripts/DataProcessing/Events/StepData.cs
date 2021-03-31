#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class StepData : MonoBehaviour
{
    public Vector3 DefaultPosition = Vector3.zero;
    public Vector3 DefaultRotation = Vector3.zero;
    public Vector3 DefaultScale = Vector3.one;

	public Vector3 MinPositionNoise = Vector3.zero;
	public Vector3 MaxPositionNoise = Vector3.zero;
	public Vector3 MinRotationNoise = Vector3.zero;
	public Vector3 MaxRotationNoise = Vector3.zero;
	public Vector3 MinScaleNoise = Vector3.one;
	public Vector3 MaxScaleNoise = Vector3.one;

	public float MaxHorizontalOffset = 0f;
	public float OriginalXScale = 0f;

	void Update()
	{
		if(transform.GetComponent<NoiseSteps>() != null)
		{
			NoiseSteps treadData = transform.GetComponent<NoiseSteps>();
			DefaultPosition = treadData.DefaultPosition;
			DefaultRotation = treadData.DefaultRotation;
			DefaultScale = treadData.DefaultScale;
			MinPositionNoise = treadData.MinPositionNoise;
			MaxPositionNoise = treadData.MaxPositionNoise;
			MinRotationNoise = treadData.MinRotationNoise;
			MaxRotationNoise = treadData.MaxRotationNoise;
			MinScaleNoise = treadData.MinScaleNoise;
			MaxScaleNoise = treadData.MaxScaleNoise;
		}

		//DefaultPosition = this.transform.localPosition;
		//DefaultRotation = this.transform.localRotation.ToEuler();
		//DefaultScale = this.transform.localScale;
	}

	/*void Awake(){
		SetStepData();
	}*/

}
#endif