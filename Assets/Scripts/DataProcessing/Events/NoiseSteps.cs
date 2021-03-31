#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[System.Serializable]
public class NoiseSteps : SceneEvent {

    public Vector3 DefaultPosition = Vector3.zero;
    public Vector3 DefaultRotation = Vector3.zero;
    public Vector3 DefaultScale = Vector3.one;

	public Vector3 MinPositionNoise = Vector3.zero;
	public Vector3 MaxPositionNoise = Vector3.zero;
	public Vector3 MinRotationNoise = Vector3.zero;
	public Vector3 MaxRotationNoise = Vector3.zero;
	public Vector3 MinScaleNoise = Vector3.one;
	public Vector3 MaxScaleNoise = Vector3.one;
	//[Range(0,3)] public int noiseType = 3;
	private int noiseType = 0;
	private Vector3 NewPosition = Vector3.zero;
	private Vector3 localPosition = Vector3.zero;

	void Reset() {
		DefaultPosition = transform.localPosition;
        DefaultRotation = transform.localEulerAngles;
        DefaultScale = transform.localScale;
	}

	public override void Callback(MotionEditor editor) {
        if(Blocked) {
            Identity(editor);
            return;
        }

		RaycastHit treadHit;
		localPosition = transform.position;
		localPosition.y += 1f;
		Physics.Raycast(localPosition, Vector3.down, out treadHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));

		//Transform parentTransform = GetStairsParentTransform(treadHit.transform);

		//if(parentTransform.GetComponent<NoiseStepsEvenDifferent>() == null) noiseType = 0; 
		//else if(parentTransform.GetComponent<NoiseStepsEvenDifferent>().UseEvenNoise) noiseType = 0;
		//else noiseType = parentTransform.GetComponent<NoiseStepsEvenDifferent>().noiseType;
		
		// Default/0 = No noise ; 1 = Height noise ; 2 = Depth noise ; 3 = Height/Depth Noise
		noiseType = 0;

		MaxScaleNoise.x = 1;
		switch(noiseType){
			case 3:
				// Tread height/width Noise
				MaxScaleNoise.x = 1.25f;
				transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
				NewPosition = new Vector3(DefaultPosition.x + ((transform.localScale.x - DefaultScale.x)/2f), DefaultPosition.y, DefaultPosition.z); // Move by half the difference cause scale modifies position
				transform.localPosition = NewPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
				transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 2:
				// Tread width Noise
				MaxScaleNoise.x = 1.25f;
				transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
				//MinPositionNoise.y = 0f;
				//MaxPositionNoise.y = 0f;
				NewPosition = new Vector3(DefaultPosition.x + ((transform.localScale.x - DefaultScale.x)/2f), DefaultPosition.y, DefaultPosition.z); // Move by half the difference cause scale modifies position
				transform.localPosition = NewPosition + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 1:
				// Tread height Noise
				transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				MinPositionNoise.x = 0f;
				MaxPositionNoise.x = 0f;
				transform.localPosition = DefaultPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
				transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 0:
				// Original size
				transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				transform.localPosition = DefaultPosition + Utility.UniformVector3(Vector2.zero, Vector3.zero);
				transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			default:
				// Original size
				transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				transform.localPosition = DefaultPosition + Utility.UniformVector3(Vector2.zero, Vector3.zero);
				transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
		}
		/*transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
		NewPosition = new Vector3(DefaultPosition.x + ((transform.localScale.x - DefaultScale.x)/2f), DefaultPosition.y, DefaultPosition.z); // Move by half the difference cause scale modifies position
		transform.localPosition = NewPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
        transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(MinRotationNoise, MaxRotationNoise);*/
	}

	public override void Identity(MotionEditor editor) {
		transform.localPosition = DefaultPosition;
        transform.localEulerAngles = DefaultRotation;
        transform.localScale = DefaultScale;
	}

	private Transform GetStairsParentTransform(Transform transform){
		//Debug.Log(transform.name);
		//if(transform.name.StartsWith("Tread")) Debug.Log("Tread!!!!!!");
		if(transform.name.StartsWith("Stairs")){
			return transform;
		}
		else{
			return GetStairsParentTransform(transform.parent.transform);
		}
	}

}
#endif