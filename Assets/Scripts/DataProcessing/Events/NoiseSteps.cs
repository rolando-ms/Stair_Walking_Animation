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
		//Random.InitState(editor.GetCurrentSeed());
		
		//Object[] SceneObjects = GameObject.FindObjectsOfType(typeof(MonoBehaviour));
		//Debug.Log("A total of " + SceneObjects.Length + " objects.");
		/*foreach(Object obj in SceneObjects){
			Debug.Log("Objects = " + obj.name);	
		}
		Debug.Log("Objects = " + SceneObjects[1]);*/
		//Debug.Log("Scale x = " + transform.localScale.x);
        
		/* //Adds 2 Units to x scale and moves steps horizontaly
		//Random.InitState(editor.GetCurrentSeed());
		MaxScaleNoise.x = 1;
		//MaxScaleNoise.x = 2f / DefaultScale.x;
		Vector3 NewScale = new Vector3(DefaultScale.x + 2f, DefaultScale.y, DefaultScale.z);
		transform.localScale = Vector3.Scale(NewScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
		//MinPositionNoise.x = -(((MaxScaleNoise.x - 1) * transform.localScale.x)/2f) - 1f; // Move by half the scale cause scale modifies position
		//MinPositionNoise.x = 0;
		MaxPositionNoise.x = DefaultScale.x / 4f;
		Vector3 NewPosition = new Vector3(DefaultPosition.x - 1f, DefaultPosition.y, DefaultPosition.z);
		transform.localPosition = NewPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
        transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(MinRotationNoise, MaxRotationNoise);
		*/

		//MaxScaleNoise.x = 1.5f;
		//MinPositionNoise.x = 0;
		//MaxPositionNoise.x = 0;

		// Default/0 = No noise ; 1 = Height noise ; 2 = Depth noise ; 3 = Height/Depth Noise
		noiseType = 3;

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
}
#endif