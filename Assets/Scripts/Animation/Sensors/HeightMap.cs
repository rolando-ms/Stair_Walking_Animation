﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class HeightMap {

	public Matrix4x4 Pivot = Matrix4x4.identity;

	public Vector3[] Map = new Vector3[0];
	public Vector3[] Points = new Vector3[0];

	public float Size = 1f;
	public int Resolution = 25;
	public LayerMask Mask = -1;

	public float Width = 0.28f;
	public float Length = 0.12f;
	public int WidthResolution = 8;
	public int LengthResolution = 5;

	public HeightMap(float size, int resolution, LayerMask mask) {
		Size = size;
		Resolution = resolution;
		Mask = mask;
		GenerateArc();
	}

	public HeightMap(float width, float length, int widthResolution, int lengthResolution, LayerMask mask) {
		Width = width;
		Length = length;
		WidthResolution = widthResolution;
		LengthResolution = lengthResolution;
		Mask = mask;
		GenerateRectangular();
	}

	public HeightMap(float size, int resolution, LayerMask mask, bool circular) {
		Size = size;
		Resolution = resolution;
		Mask = mask;
		if(circular){
			GenerateCircular();
		}else{
			Generate();
		}
	}

	private void Generate() {
		Map = new Vector3[Resolution*Resolution];
		Points = new Vector3[Resolution*Resolution];
		for(int x=0; x<Resolution; x++) {
			for(int y=0; y<Resolution; y++) {
				Map[y*Resolution + x] = new Vector3(-Size/2f + (float)x/(float)(Resolution-1)*Size, 0f, -Size/2f + (float)y/(float)(Resolution-1)*Size);
			}
		}
	}

	private void GenerateRectangular() {
		Map = new Vector3[WidthResolution*LengthResolution];
		Points = new Vector3[WidthResolution*LengthResolution];
		for(int x=0; x<LengthResolution; x++) {
			for(int y=0; y<WidthResolution; y++) {
				Map[y*LengthResolution + x] = new Vector3(-Length/2f + (float)x/(float)(LengthResolution-1)*Length, 0f,(float)y/(float)(WidthResolution-1)*Width);
			}
		}
	}

	private void GenerateCircular() {
		float diameter = GetDiameter();
		List<Vector3> map = new List<Vector3>();
		//List<Vector3> points = new List<Vector3>();
		//List<float> radius = new List<float>();
		for(int y=0; y<Resolution; y++) {
			float coverage = 0.5f * diameter;
			float distance = (float)y * coverage;
			float arc = 2f * Mathf.PI * distance;
			int count = Mathf.RoundToInt(arc / coverage);
			for(int x=0; x<count; x++) {
				float degrees = (float)x/(float)count*360f;
				map.Add(
					new Vector3(
					distance*Mathf.Cos(Mathf.Deg2Rad*degrees),
					0f,
					distance*Mathf.Sin(Mathf.Deg2Rad*degrees)
					)
				);
				/*if(Overlap) {
					radius.Add(0.5f*Mathf.Sqrt(2f)*coverage);
				} else {
					radius.Add(0.5f*coverage);
				}*/
			}
		}
		Map = map.ToArray();
		Points = new Vector3[map.Count];
		/*Radius = radius.ToArray();
		References = new Vector3[Points.Length];
		
		Occupancies = new float[Points.Length];*/
	}

	private void GenerateArc() {
		float diameter = GetDiameter();
		List<Vector3> map = new List<Vector3>();
		//List<Vector3> points = new List<Vector3>();
		//List<float> radius = new List<float>();
		for(int y=0; y<Resolution; y++) {
			float coverage = 0.5f * diameter;
			float distance = (float)y * coverage;
			float arc = Mathf.PI * distance;
			int count = Mathf.RoundToInt(arc / coverage);
			for(int x=0; x<count; x++) {
				float degrees = (float)x/(float)count*180f;
				map.Add(
					new Vector3(
					distance*Mathf.Cos(Mathf.Deg2Rad*degrees),
					0f,
					distance*Mathf.Sin(Mathf.Deg2Rad*degrees)
					)
				);
				/*if(Overlap) {
					radius.Add(0.5f*Mathf.Sqrt(2f)*coverage);
				} else {
					radius.Add(0.5f*coverage);
				}*/
			}
		}
		Map = map.ToArray();
		Points = new Vector3[map.Count];
		/*Radius = radius.ToArray();
		References = new Vector3[Points.Length];
		Occupancies = new float[Points.Length];*/
	}


	public void SetSize(float value) {
		if(Size != value) {
			Size = value;
			Generate();
		}
	}

	public void SetResolution(int value) {
		if(Resolution != value) {
			Resolution = value;
			Generate();
		}
	}

	public void Sense(Matrix4x4 pivot) {
		Pivot = pivot;
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Quaternion.AngleAxis(Pivot.GetRotation().eulerAngles.y, Vector3.up);
		for(int i=0; i<Map.Length; i++) {
			Points[i] = Project(position + rotation * Map[i]);
		}
	}

	public float[] GetHeights() {
		float[] heights = new float[Points.Length];
		for(int i=0; i<heights.Length; i++) {
			heights[i] = Points[i].y;
		}
		return heights;
	}

	public float[] GetHeights(float maxHeight) {
		float[] heights = new float[Points.Length];
		for(int i=0; i<heights.Length; i++) {
			heights[i] = Mathf.Clamp(Points[i].y, 0f, maxHeight);
		}
		return heights;
	}

	private float GetDiameter() {
		return Size / (float)(Resolution-1);
	}

	private Vector3 Project(Vector3 position) {
		//Vector3.Distance()
		RaycastHit hit;
		Physics.Raycast(new Vector3(position.x, 100f, position.z), Vector3.down, out hit, float.PositiveInfinity, Mask);
		position = hit.point;
		return position;
	}

	public void Draw(float[] mean=null, float[] std=null) {
		//return;
		UltiDraw.Begin();

		Quaternion rotation = Pivot.GetRotation() * Quaternion.Euler(90f, 0f, 0f);
		Color color = UltiDraw.IndianRed.Transparent(0.5f);
		//Quaternion rotation = new Quaternion(1,1,1,1);
		//float area = (float)Size/(float)(Resolution-1);
		for(int i=0; i<Points.Length; i++) {
			//UltiDraw.DrawCircle(Points[i], 0.025f, color);
			UltiDraw.DrawSphere(Points[i], rotation, 0.05f, color);
			//UltiDraw.DrawQuad(Points[i], rotation, area, area, color);
		}

		UltiDraw.End();
	}

	public void DrawFootMap(Color color, float[] mean=null, float[] std=null) {
		//return;
		UltiDraw.Begin();

		Quaternion rotation = Pivot.GetRotation() * Quaternion.Euler(90f, 0f, 0f);
		//color = UltiDraw.IndianRed.Transparent(0.75f);
		//Quaternion rotation = new Quaternion(1,1,1,1);
		//float area = (float)Size/(float)(Resolution-1);
		for(int i=0; i<Points.Length; i++) {
			//UltiDraw.DrawCircle(Points[i], 0.025f, color);
			UltiDraw.DrawSphere(Points[i], rotation, 0.05f, color);
			//UltiDraw.DrawQuad(Points[i], rotation, area, area, color);
		}

		UltiDraw.End();
	}

	public void DrawFootPoint(Color color, int iterator, float[] mean=null, float[] std=null) {
		//return;
		UltiDraw.Begin();

		Quaternion rotation = Pivot.GetRotation() * Quaternion.Euler(90f, 0f, 0f);
		
		UltiDraw.DrawSphere(Points[iterator], rotation, 0.05f, color);

		UltiDraw.End();
	}
	
	public void Render(Vector2 center, Vector2 size, int width, int height, float maxHeight) {
		UltiDraw.Begin();
		UltiDraw.DrawGUIGreyscaleImage(center, size, width, height, GetHeights(maxHeight));
		UltiDraw.End();
	}

}

/*
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CircleMap {

	public Matrix4x4 Pivot = Matrix4x4.identity;

	public Vector3[] Map = new Vector3[0];
	public Vector3[] Points = new Vector3[0];

	public float Radius = 1f;
	public int Samples = 10;
	public int Rays = 10;
	public LayerMask Mask = -1;

	public CircleMap(float radius, int samples, int rays, LayerMask mask) {
		Radius = radius;
		Samples = samples;
		Rays = rays;
		Mask = mask;
		Map = new Vector3[Rays*Samples];
		Points = new Vector3[Rays*Samples];
		for(int i=0; i<Rays; i++) {
			float angle = 360f*(float)i/(float)Rays;
			for(int j=0; j<Samples; j++) {
				float distance = Radius*(float)(j+1)/(float)Samples;
				Map[i*Samples+j] = Quaternion.AngleAxis(angle, Vector3.up) * new Vector3(0f, 0f, distance);
			}
		}
	}

	public void Sense(Matrix4x4 pivot) {
		Pivot = pivot;
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Quaternion.AngleAxis(Pivot.GetRotation().eulerAngles.y, Vector3.up);
		for(int i=0; i<Map.Length; i++) {
			Points[i] = Project(position + rotation * Map[i]);
		}
	}

	private Vector3 Project(Vector3 position) {
		RaycastHit hit;
		Physics.Raycast(new Vector3(position.x, 100f, position.z), Vector3.down, out hit, float.PositiveInfinity, Mask);
		position = hit.point;
		return position;
	}

	public void Draw() {
		UltiDraw.Begin();
		for(int i=0; i<Points.Length; i++) {
			UltiDraw.DrawCircle(Points[i], 0.025f, UltiDraw.Mustard.Transparent(0.5f));
		}
		UltiDraw.End();
	}

}
*/

/*
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CircleMap {

	public Matrix4x4 Pivot = Matrix4x4.identity;

	public Vector3[] Map = new Vector3[0];
	public Vector3[] Points = new Vector3[0];

	public float Radius = 1f;
	public int Samples = 10;
	public int Rays = 10;
	public LayerMask Mask = -1;

	public CircleMap(float radius, int samples, int rays, LayerMask mask) {
		Radius = radius;
		Samples = samples;
		Rays = rays;
		Mask = mask;
		List<Vector3> map = new List<Vector3>();
		for(int i=0; i<Rays; i++) {
			float r = Radius*(float)(i+1)/(float)Rays;
			int count = Mathf.RoundToInt(Samples * r / Radius);
			float step = 360f / count;
			//float step = 360f / (r/Radius) / Samples;
			//int count = Mathf.RoundToInt(360f / step);
			for(int j=0; j<count; j++) {
				map.Add(Quaternion.AngleAxis(j*step, Vector3.up) * new Vector3(0f, 0f, r));
			}
		}
		Map = map.ToArray();
		Points = new Vector3[Map.Length];
		//Debug.Log(Map.Length);
	}

	private float GetArc(float r) {
		return 2f*Mathf.PI*r;
	}

	public float[] GetHeights() {
		float[] heights = new float[Points.Length];
		for(int i=0; i<heights.Length; i++) {
			heights[i] = Points[i].y;
		}
		return heights;
	}

	public void Sense(Matrix4x4 pivot) {
		Pivot = pivot;
		Vector3 position = Pivot.GetPosition();
		Quaternion rotation = Quaternion.AngleAxis(Pivot.GetRotation().eulerAngles.y, Vector3.up);
		for(int i=0; i<Map.Length; i++) {
			Points[i] = Project(position + rotation * Map[i]);
		}
	}

	private Vector3 Project(Vector3 position) {
		RaycastHit hit;
		Physics.Raycast(new Vector3(position.x, 100f, position.z), Vector3.down, out hit, float.PositiveInfinity, Mask);
		position = hit.point;
		return position;
	}

	public void Draw() {
		UltiDraw.Begin();
		for(int i=0; i<Points.Length; i++) {
			UltiDraw.DrawCircle(Points[i], 0.025f, UltiDraw.Mustard.Transparent(0.5f));
		}
		UltiDraw.End();
	}

}
*/