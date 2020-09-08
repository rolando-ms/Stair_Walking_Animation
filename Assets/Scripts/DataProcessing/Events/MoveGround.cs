#if UNITY_EDITOR
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveGround : SceneEvent
{
    Vector3 DefaultPosition = new Vector3(0f, -0.08f, 0f);
    Vector3 DefaultRotation = Vector3.zero;
    Vector3 DefaultScale = new Vector3(50f, 0.1f, 50f);    
    float OffsetY = 0f;
    MotionData data;
    Frame FirstFrame, LastFrame;
    //Matrix4x4[] worldMatrices;

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
        
        //Frame currentFrame = editor.GetCurrentFrame();
        data = editor.GetData();
        FirstFrame = data.GetFrame(1);
        FirstFrame = FirstFrame.GetFirstFrame();
        Matrix4x4 RightToeTransformationFirstFrame = FirstFrame.GetBoneTransformation("RightToe", editor.Mirror);
        //Debug.Log("Root in frame 1 = " + RightToeTransformationFirstFrame.GetPosition().y);
        float RightToeYFirstFrame = RightToeTransformationFirstFrame.GetPosition().y;

        LastFrame = data.GetFrame(1);
        LastFrame = LastFrame.GetLastFrame();
        Matrix4x4 RightToeTransformationLastFrame = LastFrame.GetBoneTransformation("RightToe", editor.Mirror);
        //Debug.Log("Root in last = " + RightToeTransformationLastFrame.GetPosition().y);
        float RightToeYLastFrame = RightToeTransformationLastFrame.GetPosition().y;

        OffsetY = RightToeYFirstFrame <= RightToeYLastFrame? RightToeYFirstFrame : RightToeYLastFrame;

        /*
        Vector3 rightToePosition = rightToeTransformation.GetPosition();
        Debug.Log("Right Toe position 0 = " + rightToePosition);
        worldMatrices = frame1.World;
        Debug.Log("World position 0 " + worldMatrices[21].GetPosition());
        */
        
        /*
        Actor actor = editor.GetActor();
        if(actor.FindTransform("RightToe").position.y < OffsetY){
            OffsetY = actor.FindTransform("RightToe").position.y;
        }
        Debug.Log("Local position before= " + transform.localPosition);
        */
        transform.localPosition = new Vector3(DefaultPosition.x, DefaultPosition.y + OffsetY, DefaultPosition.z);
        //Debug.Log("Local position after= " + transform.localPosition);
        transform.localEulerAngles = DefaultRotation;
        transform.localScale = DefaultScale;
	}

	public override void Identity(MotionEditor editor) {
		transform.localPosition = DefaultPosition;
        transform.localEulerAngles = DefaultRotation;
        transform.localScale = DefaultScale;
	}
}
#endif