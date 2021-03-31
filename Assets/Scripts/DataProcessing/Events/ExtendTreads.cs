using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class ExtendTreads : MonoBehaviour
{
    
    // Update is called once per frame
    void Update()
    {
        
        // Ascending
        for(int i=0; i<transform.childCount; i++)
        {
            Transform treadTransform = GetChildTransform(transform.GetChild(i).transform);
            StepData treadData = treadTransform.GetComponent<StepData>();
            StepData previousTreadData;
            if(treadData.DefaultScale.x < 1) 
            {
                if(i == 0) previousTreadData = GetChildTransform(transform.GetChild(i).transform).GetComponent<StepData>();
                else previousTreadData = GetChildTransform(transform.GetChild(i-1).transform).GetComponent<StepData>();

                // Saving horizontal offset
                if(i == 0) treadData.MaxHorizontalOffset = treadData.DefaultScale.x;
                else if(previousTreadData.DefaultScale.x != 1) treadData.MaxHorizontalOffset = previousTreadData.DefaultScale.x;
                else treadData.MaxHorizontalOffset = previousTreadData.OriginalXScale;

                // Save original X value
                treadData.OriginalXScale = treadData.DefaultScale.x;

                // Extend tread width
                treadData.DefaultScale.x = 1;
                
                // Correct Position
                treadData.DefaultPosition -= treadTransform.right.normalized * ((1f - treadData.OriginalXScale)/2f);

                // Correct Max Scale Noise
                treadData.MaxScaleNoise.x = 1.08f;

                Debug.Log("Tread " + treadTransform.name + " updated");
            } else if(treadData.DefaultScale.x == 4)
            {
                previousTreadData = GetChildTransform(transform.GetChild(i-1).transform).GetComponent<StepData>();
                treadData.MaxHorizontalOffset = previousTreadData.OriginalXScale;
                treadData.OriginalXScale = 4;
            }
        }
        

        /*
        // Descending
        for(int i=0; i<transform.childCount; i++)
        {
            Transform treadTransform = GetChildTransform(transform.GetChild(i).transform);
            StepData treadData = treadTransform.GetComponent<StepData>();
            StepData nextTreadData;
            if(treadData.DefaultScale.x < 1) 
            {
                if(i == transform.childCount - 1) nextTreadData = GetChildTransform(transform.GetChild(i).transform).GetComponent<StepData>();
                else nextTreadData = GetChildTransform(transform.GetChild(i+1).transform).GetComponent<StepData>();

                // Saving horizontal offset
                if(i == transform.childCount) treadData.MaxHorizontalOffset = treadData.DefaultScale.x;
                else if(nextTreadData.DefaultScale.x != 1) treadData.MaxHorizontalOffset = nextTreadData.DefaultScale.x;
                else treadData.MaxHorizontalOffset = nextTreadData.OriginalXScale;

                // Save original X value
                treadData.OriginalXScale = treadData.DefaultScale.x;

                // Extend tread width
                treadData.DefaultScale.x = 1;
                
                // Correct Position
                treadData.DefaultPosition -= treadTransform.right.normalized * ((1f - treadData.OriginalXScale)/2f);

                // Correct Max Scale Noise
                treadData.MaxScaleNoise.x = 1.08f;

                Debug.Log("Tread " + treadTransform.name + " updated");
            } else if(treadData.DefaultScale.x == 4)
            {
                nextTreadData = GetChildTransform(transform.GetChild(i+1).transform).GetComponent<StepData>();
                if(nextTreadData.DefaultScale.x != 1) treadData.MaxHorizontalOffset = nextTreadData.DefaultScale.x;
                treadData.OriginalXScale = 4;
            }
        }
        */    
    }

    private Transform GetChildTransform(Transform childTransform){
        if(childTransform.childCount == 0) return childTransform;
        else return GetChildTransform(childTransform.GetChild(0));
    }
}
