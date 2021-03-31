#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class NoiseStepsEvenDifferent : SceneEvent {
    [Tooltip("If true, uses homogeneous noise for all steps, otherwise uses independent noise given in Steps Data")]
	public bool UseEvenNoise = true;
	public bool ResetToOriginal = false;
	public float HeightHomogeneousNoise = 0.2f;
	//public float WidthHomogeneousNoise = 1.25f;
    public Vector3 DefaultPosition = Vector3.zero;
    public Vector3 DefaultRotation = Vector3.zero;
    public Vector3 DefaultScale = Vector3.one;

	public Vector3 MinPositionNoise = Vector3.zero;
	public Vector3 MaxPositionNoise = Vector3.zero;
	public Vector3 MinRotationNoise = Vector3.zero;
	public Vector3 MaxRotationNoise = Vector3.zero;
	public Vector3 MinScaleNoise = Vector3.one;
	public Vector3 MaxScaleNoise = Vector3.one;

    [Tooltip("0 = No noise, 1 = Height noise, 2 = Width noise, 3 = Height/Width noise")]
	[Range(0,3)]
	public int NoiseType = 0;
    private Vector3 NewPosition = Vector3.zero;

	private float randomNumber = 0f;

	private bool stop = false;

	void Reset() {
		DefaultPosition = transform.localPosition;
        DefaultRotation = transform.localEulerAngles;
        DefaultScale = transform.localScale;
	}

	public float GetRandomNumberOfStairs(){
		return randomNumber;
	}

	public override void Callback(MotionEditor editor) {
        if(Blocked) {
            Identity(editor);
            return;
        }

		NoiseMenu noiseMenu = GameObject.Find("StairsNoise").GetComponent<NoiseMenu>();
		ResetToOriginal = noiseMenu.ResetToOriginal;
		UseEvenNoise = noiseMenu.UseEvenNoise;
		NoiseType = noiseMenu.NoiseType;

        if(ResetToOriginal){
			//Debug.Log("Starting reset!!!!!!");
			for(int i = 0; i < transform.childCount; i++){
                Transform childTransform = GetChildTransform(transform.GetChild(i));
				//UpdateStairsHeight(childTransform);
				ResetChild(childTransform);
                //Debug.Log("Child reseted = " + childTransform.name);
            }
		}else if(UseEvenNoise){
			randomNumber = Random.value;
			for(int i = 0; i < transform.childCount; i++){
                Transform childTransform = GetChildTransform(transform.GetChild(i));
				//!!!if(!stop) UpdateStairsHeight(childTransform);
				//ResetChild(childTransform);
                //Debug.Log("Child name = " + childTransform.name);
                ApplyHomogeneousStepNoise(editor, childTransform, randomNumber);
            }
			/*for(int i = 0; i < transform.childCount; i++){
				Transform childTransform = GetChildTransform(transform.GetChild(i));
				if(childTransform.name == "Tread"){
					StepData stepData = childTransform.GetComponent<StepData>();
					if(stepData.DefaultPosition.x < 5.779f && stepData.DefaultPosition.x > 5.777f){
						stop = true;
					}
				}
			}*/
			//Vector3 newPosition = Vector3.zero;
			//Random.InitState(editor.GetCurrentSeed());
            //transform.localPosition = DefaultPosition + Utility.UniformVector3(MinPositionNoise, MaxPositionNoise);
            //transform.localEulerAngles = DefaultRotation + Utility.UniformVector3(MinRotationNoise, MaxRotationNoise);
            //transform.localScale = Vector3.Scale(DefaultScale, Utility.UniformVector3(MinScaleNoise, MaxScaleNoise));
			//if(MaxScaleNoise.x > 1f){
				//transform.localPosition.x = DefaultPosition.x + MaxScaleNoise.x/2f;
				//newPosition.x = ;
			//}
        }else{
            //Random.InitState(1);
            //Debug.Log("Child count = " + transform.childCount);
            for(int i = 0; i < transform.childCount; i++){
                Transform childTransform = GetChildTransform(transform.GetChild(i));
                //Debug.Log("Child name = " + childTransform.name);
                ApplyIndividualStepNoise(childTransform);
            }
        }

	}

	public override void Identity(MotionEditor editor) {
		transform.localPosition = DefaultPosition;
        transform.localEulerAngles = DefaultRotation;
        transform.localScale = DefaultScale;
	}

    private Transform GetChildTransform(Transform childTransform){
        if(childTransform.childCount == 0) return childTransform;
        else return GetChildTransform(childTransform.GetChild(0));
    }

	private void ApplySequentialEvenStepNoise(Transform treadTransform, int treadNumber){
		//
	}

	private void ApplyHomogeneousStepNoise(MotionEditor editor, Transform childTransform, float randomNumber){
		// Height noise
		StepData stepData = childTransform.GetComponent<StepData>();
		Vector3 childTransformWithYOffset = childTransform.position;
		childTransformWithYOffset.y += 10f;
		RaycastHit TreadHit;
		Physics.Raycast(childTransformWithYOffset, Vector3.down, out TreadHit, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
		GameObject ground = GameObject.Find("Ground");
		float difference = Mathf.Abs(TreadHit.point.y - ground.transform.position.y);
		Vector3 newPos = childTransform.localPosition;
		//newPos.y = stepData.DefaultPosition.y - difference * randomNumber * HeightHomogeneousNoise;
		
		// Width noise
		Vector3 scaleVector = stepData.DefaultScale;
		float XNoise = (MaxScaleNoise.x - 1f) * randomNumber;
		scaleVector.x = scaleVector.x * (1f + XNoise); // * stepData.DefaultScale.x;

		Vector3 initialDirection = editor.GetData().GetFrame(0).GetBoneTransformation("Hips", editor.Mirror).GetForward();
		RaycastHit[] TreadHits = Physics.RaycastAll(childTransform.position, -initialDirection, float.PositiveInfinity, LayerMask.GetMask("Ground", "Interaction"));
		float cummulativeDistanceOffset = 0f;
		foreach(RaycastHit hit in TreadHits){
			//Debug.Log("Hit = " + hit.collider.name);
			if(hit.collider.name != "Ground"){
				StepData hitData = hit.collider.GetComponent<StepData>();
				cummulativeDistanceOffset += ((1f + XNoise) * hitData.DefaultScale.x - hitData.DefaultScale.x) / 2f;
			}
		}
		/*if(XNoise > 0f){
			newPos.x = stepData.DefaultPosition.x - (((1f + XNoise) * stepData.DefaultScale.x - stepData.DefaultScale.x) / 2f) - cummulativeDistanceOffset;
		}*/
		//Debug.Log("Cummulative offset = " + cummulativeDistanceOffset);
		
		// Update
		switch(NoiseType){
			case 3:
				newPos.y = stepData.DefaultPosition.y - difference * randomNumber * HeightHomogeneousNoise;
				if(XNoise > 0f){
					newPos.x = stepData.DefaultPosition.x - (((1f + XNoise) * stepData.DefaultScale.x - stepData.DefaultScale.x) / 2f) - cummulativeDistanceOffset;
				}
				childTransform.localScale = new Vector3(scaleVector.x, stepData.DefaultScale.y, stepData.DefaultScale.z);
				break;
			case 2:
				//if(XNoise > 0f){
				newPos.x = stepData.DefaultPosition.x - (((1f + XNoise) * stepData.DefaultScale.x - stepData.DefaultScale.x) / 2f) - cummulativeDistanceOffset;
				//}
				childTransform.localScale = new Vector3(scaleVector.x, stepData.DefaultScale.y, stepData.DefaultScale.z);
				break;
			case 1:
				newPos.y = stepData.DefaultPosition.y - difference * randomNumber * HeightHomogeneousNoise;
				break;
			case 0:
				break;
			default:
				break;
		}
		childTransform.localPosition = newPos;
		//Debug.Log("Scaled vector = " + new Vector3(scaleVector.x, stepData.DefaultScale.y, stepData.DefaultScale.z));
		//childTransform.localScale = new Vector3(scaleVector.x, stepData.DefaultScale.y, stepData.DefaultScale.z);
	}

	private void ResetChild(Transform childTransform){
		StepData stepData = childTransform.GetComponent<StepData>();
		childTransform.localPosition = stepData.DefaultPosition;
		childTransform.localEulerAngles = stepData.DefaultRotation;
		childTransform.localScale = stepData.DefaultScale;
	}

	private void UpdateStairsHeight(Transform childTransform){
		StepData stepData = childTransform.GetComponent<StepData>();
		if(stepData.DefaultScale.y == 4f){
			stepData.DefaultScale.y = 8f;
			stepData.DefaultPosition.y -= 2f;
		}
		
		//if()
		//stepData.DefaultPosition.x += 0.03f;
		//stepData.DefaultPosition.z += 0.001f;
	}

    private void ApplyIndividualStepNoise(Transform childTransform){
        StepData stepData = childTransform.GetComponent<StepData>();

        //stepData.MaxScaleNoise.x = 1;
		switch(NoiseType){
			case 3:
				// Tread height/width Noise
				//stepData.MaxScaleNoise.x = 1.25f;
				childTransform.localScale = Vector3.Scale(stepData.DefaultScale, Utility.UniformVector3(stepData.MinScaleNoise, stepData.MaxScaleNoise));
				NewPosition = new Vector3(stepData.DefaultPosition.x + ((childTransform.localScale.x - stepData.DefaultScale.x)/2f), stepData.DefaultPosition.y, stepData.DefaultPosition.z); // Move by half the difference cause scale modifies position
				childTransform.localPosition = NewPosition + Utility.UniformVector3(stepData.MinPositionNoise, stepData.MaxPositionNoise);
				childTransform.localEulerAngles = stepData.DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 2:
				// Tread width Noise
				//stepData.MaxScaleNoise.x = 1.25f;
				childTransform.localScale = Vector3.Scale(stepData.DefaultScale, Utility.UniformVector3(stepData.MinScaleNoise, stepData.MaxScaleNoise));
				//MinPositionNoise.y = 0f;
				//MaxPositionNoise.y = 0f;
				NewPosition = new Vector3(stepData.DefaultPosition.x + ((childTransform.localScale.x - stepData.DefaultScale.x)/2f), stepData.DefaultPosition.y, stepData.DefaultPosition.z); // Move by half the difference cause scale modifies position
				childTransform.localPosition = NewPosition + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				childTransform.localEulerAngles = stepData.DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 1:
				// Tread height Noise
				childTransform.localScale = Vector3.Scale(stepData.DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				MinPositionNoise.x = 0f;
				MaxPositionNoise.x = 0f;
				childTransform.localPosition = stepData.DefaultPosition + Utility.UniformVector3(stepData.MinPositionNoise, stepData.MaxPositionNoise);
				childTransform.localEulerAngles = stepData.DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			case 0:
				// Original size
				childTransform.localScale = Vector3.Scale(stepData.DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				childTransform.localPosition = stepData.DefaultPosition + Utility.UniformVector3(Vector2.zero, Vector3.zero);
				childTransform.localEulerAngles = stepData.DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
			default:
				// Original size
				childTransform.localScale = Vector3.Scale(stepData.DefaultScale, Utility.UniformVector3(Vector3.one, Vector3.one));
				childTransform.localPosition = stepData.DefaultPosition + Utility.UniformVector3(Vector2.zero, Vector3.zero);
				childTransform.localEulerAngles = stepData.DefaultRotation + Utility.UniformVector3(Vector3.zero, Vector3.zero);
				break;
		}
    }

}
#endif
