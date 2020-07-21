#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

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
		/*
		Object[] SceneObjects = GameObject.FindObjectsOfType(typeof(MonoBehaviour));
		//Debug.Log("A total of " + SceneObjects.Length + " objects.");
		foreach(Object obj in SceneObjects){
			Debug.Log("Objects = " + obj.name);	
		}
		Debug.Log("Objects = " + SceneObjects[1]);
        */
		transform.localPosition = DefaultPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
        transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(MinRotationNoise, MaxRotationNoise);
        transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
	}

	public override void Identity(MotionEditor editor) {
		transform.localPosition = DefaultPosition;
        transform.localEulerAngles = DefaultRotation;
        transform.localScale = DefaultScale;
	}

}
#endif