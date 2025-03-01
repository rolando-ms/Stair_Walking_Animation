﻿#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using System;
using System.Threading;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Globalization;

public class MotionExporter : EditorWindow {

	public static EditorWindow Window;
	public static Vector2 Scroll;

	public string Directory = string.Empty;
	public float Framerate = 60f;
	public int BatchSize = 10;
	public int numberOfLaps = 1;

	public List<LabelGroup> Actions = new List<LabelGroup>();
	public List<LabelGroup> Styles = new List<LabelGroup>();

	public bool ShowFiles = false;
	public List<MotionData> Files = new List<MotionData>();
	public List<bool> Export = new List<bool>();

	public MotionEditor Editor = null;

	private int Index = -1;
	private float Progress = 0f;
	private float Performance = 0f;

	public bool WriteMirror = true;
	public bool LoadActiveOnly = true;

	public bool UseDataSteps = false;
	public bool UseHeightMap = false;

	private int Start = 0;
	private int End = 0;

	private static bool Exporting = false;
	private static string Separator = " ";
	private static string Accuracy = "F5";

	[MenuItem ("AI4Animation/Motion Exporter")]
	static void Init() {
		Window = EditorWindow.GetWindow(typeof(MotionExporter));
		Scroll = Vector3.zero;
	}
	
	public void OnInspectorUpdate() {
		Repaint();
	}

	void OnGUI() {
		Scroll = EditorGUILayout.BeginScrollView(Scroll);

		Editor = GameObject.FindObjectOfType<MotionEditor>();

		if(Editor == null) {
			Utility.SetGUIColor(UltiDraw.Black);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				Utility.ResetGUIColor();
				Utility.SetGUIColor(UltiDraw.Grey);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					Utility.ResetGUIColor();
					Utility.SetGUIColor(UltiDraw.Orange);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						EditorGUILayout.LabelField("Exporter");
					}
					EditorGUILayout.LabelField("No Motion Editor found in scene.");
				}
			}
		} else {
			Utility.SetGUIColor(UltiDraw.Black);
			using(new EditorGUILayout.VerticalScope ("Box")) {
				Utility.ResetGUIColor();

				Utility.SetGUIColor(UltiDraw.Grey);
				using(new EditorGUILayout.VerticalScope ("Box")) {
					Utility.ResetGUIColor();

					Utility.SetGUIColor(UltiDraw.Orange);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						EditorGUILayout.LabelField("Exporter");
					}

					Framerate = EditorGUILayout.FloatField("Framerate", Framerate);
					BatchSize = Mathf.Max(1, EditorGUILayout.IntField("Batch Size", BatchSize));
					WriteMirror = EditorGUILayout.Toggle("Write Mirror", WriteMirror);
					LoadActiveOnly = EditorGUILayout.Toggle("Load Active Only", LoadActiveOnly);
					UseDataSteps = EditorGUILayout.Toggle("Use Data Steps", UseDataSteps);
					UseHeightMap = EditorGUILayout.Toggle("Use Height Map", UseHeightMap);
					numberOfLaps = Mathf.Max(1, EditorGUILayout.IntField("Number of Laps", numberOfLaps));

					Utility.SetGUIColor(UltiDraw.White);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						EditorGUILayout.LabelField("Export Path: " + GetExportPath());
					}

					Utility.SetGUIColor(UltiDraw.LightGrey);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						Utility.SetGUIColor(UltiDraw.Cyan);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							Utility.ResetGUIColor();
							EditorGUILayout.LabelField("Files" + " [" + Files.Count + "]");
						}
						ShowFiles = EditorGUILayout.Toggle("Show Files", ShowFiles);
						if(Files.Count == 0) {
							EditorGUILayout.LabelField("No files found.");
						} else {
							if(ShowFiles) {
								EditorGUILayout.BeginHorizontal();
								if(Utility.GUIButton("Export All", UltiDraw.DarkGrey, UltiDraw.White)) {
									for(int i=0; i<Export.Count; i++) {
										if(Files[i].Export) {
											Export[i] = Files[i].Export;
										}
									}
								}
								if(Utility.GUIButton("Export None", UltiDraw.DarkGrey, UltiDraw.White)) {
									for(int i=0; i<Export.Count; i++) {
										Export[i] = false;
									}
								}
								EditorGUILayout.EndHorizontal();
								EditorGUILayout.BeginHorizontal();
								Start = EditorGUILayout.IntField("Start", Start);
								End = EditorGUILayout.IntField("End", End);
								if(Utility.GUIButton("Toggle", UltiDraw.DarkGrey, UltiDraw.White)) {
									for(int i=Start-1; i<=End-1; i++) {
										if(Files[i].Export) {
											Export[i] = !Export[i];
										}
									}
								}
								EditorGUILayout.EndHorizontal();
								for(int i=0; i<Files.Count; i++) {
									Utility.SetGUIColor(Index == i ? UltiDraw.Cyan : Export[i] ? UltiDraw.Gold : UltiDraw.White);
									using(new EditorGUILayout.VerticalScope ("Box")) {
										Utility.ResetGUIColor();
										EditorGUILayout.BeginHorizontal();
										EditorGUILayout.LabelField((i+1) + " - " + Files[i].GetName(), GUILayout.Width(200f));
										if(Files[i].Export) {
											EditorGUILayout.BeginVertical();
											if(Files[i].Export) {
												string info = " Scene - ";
												if(Files[i].Symmetric) {
													info += "[Default / Mirror]";
												} else {
													info += "[Default]";
												}
												EditorGUILayout.LabelField(info, GUILayout.Width(200f));
											}
											EditorGUILayout.EndVertical();
											GUILayout.FlexibleSpace();
											if(Utility.GUIButton("O", Export[i] ? UltiDraw.DarkGreen : UltiDraw.DarkRed, UltiDraw.White, 50f)) {
												Export[i] = !Export[i];
											}
										}
										EditorGUILayout.EndHorizontal();
									}
								}
							}
						}
					}

					Utility.SetGUIColor(UltiDraw.LightGrey);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						Utility.SetGUIColor(UltiDraw.Cyan);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							Utility.ResetGUIColor();
							EditorGUILayout.LabelField("Actions" + " [" + Actions.Count + "]");
						}
						if(Actions.Count == 0) {
							EditorGUILayout.LabelField("No actions found.");
						} else {
							for(int i=0; i<Actions.Count; i++) {
								Utility.SetGUIColor(UltiDraw.Grey);
								using(new EditorGUILayout.VerticalScope ("Box")) {
									Utility.ResetGUIColor();
									EditorGUILayout.BeginHorizontal();
									EditorGUILayout.LabelField("Group " + (i+1));
									if(Utility.GUIButton("X", UltiDraw.DarkRed, UltiDraw.White, 20f, 20f)) {
										Actions.RemoveAt(i);
										EditorGUIUtility.ExitGUI();
									}
									EditorGUILayout.EndHorizontal();
									for(int j=0; j<Actions[i].Labels.Length; j++) {
										Actions[i].Labels[j] = EditorGUILayout.TextField(Actions[i].Labels[j]);
									}
									EditorGUILayout.BeginHorizontal();
									if(Utility.GUIButton("+", UltiDraw.DarkGrey, UltiDraw.White)) {
										ArrayExtensions.Expand(ref Actions[i].Labels);
									}
									if(Utility.GUIButton("-", UltiDraw.DarkGrey, UltiDraw.White)) {
										ArrayExtensions.Shrink(ref Actions[i].Labels);
									}
									EditorGUILayout.EndHorizontal();
								}
							}
						}
					}

					Utility.SetGUIColor(UltiDraw.LightGrey);
					using(new EditorGUILayout.VerticalScope ("Box")) {
						Utility.ResetGUIColor();
						Utility.SetGUIColor(UltiDraw.Cyan);
						using(new EditorGUILayout.VerticalScope ("Box")) {
							Utility.ResetGUIColor();
							EditorGUILayout.LabelField("Styles" + " [" + Styles.Count + "]");
						}
						if(Styles.Count == 0) {
							EditorGUILayout.LabelField("No styles found.");
						} else {
							for(int i=0; i<Styles.Count; i++) {
								Utility.SetGUIColor(UltiDraw.Grey);
								using(new EditorGUILayout.VerticalScope ("Box")) {
									Utility.ResetGUIColor();
									EditorGUILayout.BeginHorizontal();
									EditorGUILayout.LabelField("Group " + (i+1));
									if(Utility.GUIButton("X", UltiDraw.DarkRed, UltiDraw.White, 20f, 20f)) {
										Styles.RemoveAt(i);
										EditorGUIUtility.ExitGUI();
									}
									EditorGUILayout.EndHorizontal();
									for(int j=0; j<Styles[i].Labels.Length; j++) {
										Styles[i].Labels[j] = EditorGUILayout.TextField(Styles[i].Labels[j]);
									}
									EditorGUILayout.BeginHorizontal();
									if(Utility.GUIButton("+", UltiDraw.DarkGrey, UltiDraw.White)) {
										ArrayExtensions.Expand(ref Styles[i].Labels);
									}
									if(Utility.GUIButton("-", UltiDraw.DarkGrey, UltiDraw.White)) {
										ArrayExtensions.Shrink(ref Styles[i].Labels);
									}
									EditorGUILayout.EndHorizontal();
								}
							}
						}
					}

					if(!Exporting) {
						if(Utility.GUIButton("Reload", UltiDraw.DarkGrey, UltiDraw.White)) {
							Load();
						}
						if(Utility.GUIButton("Export Data", UltiDraw.DarkGrey, UltiDraw.White)) {
							if(!UseDataSteps) this.StartCoroutine(ExportDataSIGGRAPHAsia());
							else this.StartCoroutine(ExportDataSteps());
						}
					} else {
						EditorGUILayout.LabelField("File: " + Editor.GetData().GetName());

						EditorGUI.DrawRect(new Rect(EditorGUILayout.GetControlRect().x, EditorGUILayout.GetControlRect().y, Progress * EditorGUILayout.GetControlRect().width, 25f), UltiDraw.Green.Transparent(0.75f));

						EditorGUILayout.LabelField("Frames Per Second: " + Performance.ToString("F3"));

						if(Utility.GUIButton("Stop", UltiDraw.DarkRed, UltiDraw.White)) {
							Exporting = false;
						}
					}
				}
			}
		}

		EditorGUILayout.EndScrollView();
	}

	public void Load() {
		if(Editor != null) {
			Actions = new List<LabelGroup>();
			Styles = new List<LabelGroup>();

			Files = new List<MotionData>();
			Export = new List<bool>();
			for(int i=0; i<Editor.Files.Length; i++) {
				Files.Add(Editor.Files[i]);
				if(Editor.Files[i].Export || !LoadActiveOnly) {
					Export.Add(true);
					if(Editor.Files[i].GetModule(Module.ID.Goal) != null) {
						GoalModule module = (GoalModule)Editor.Files[i].GetModule(Module.ID.Goal);
						for(int j=0; j<module.Functions.Length; j++) {
							if(Actions.Find(x => ArrayExtensions.Contains(ref x.Labels, module.Functions[j].Name)) == null) {
								Actions.Add(new LabelGroup(module.Functions[j].Name));
							}
						}
					}
					if(Editor.Files[i].GetModule(Module.ID.Style) != null) {
						StyleModule module = (StyleModule)Editor.Files[i].GetModule(Module.ID.Style);
						for(int j=0; j<module.Functions.Length; j++) {
							if(Styles.Find(x => ArrayExtensions.Contains(ref x.Labels, module.Functions[j].Name)) == null) {
								Styles.Add(new LabelGroup(module.Functions[j].Name));
							}
						}
					}
				} else {
					Export.Add(false);
				}
			}
		}
	}

	private StreamWriter CreateFile(string name) {
		string filename = string.Empty;
		string folder = Application.dataPath + "/../../Export/";
		if(!File.Exists(folder+name+".txt")) {
			filename = folder+name;
		} else {
			int i = 1;
			while(File.Exists(folder+name+" ("+i+").txt")) {
				i += 1;
			}
			filename = folder+name+" ("+i+")";
		}
		return File.CreateText(filename+".txt");
	}

	public class Data {
		public StreamWriter File, Norm, Labels;

		public RunningStatistics[] Statistics = null;

		private Queue<float[]> Buffer = new Queue<float[]>();
		private Task Writer = null;

		private float[] Values = new float[0];
		private string[] Names = new string[0];
		private float[] Weights = new float[0];
		private int Dim = 0;

		private bool Finished = false;
		private bool Setup = false;

		public Data(StreamWriter file, StreamWriter norm, StreamWriter labels) {
			File = file;
			Norm = norm;
			Labels = labels;
			Writer = Task.Factory.StartNew(() => WriteData());
		}

		public void Feed(float value, string name, float weight=1f) {
			if(!Setup) {
				ArrayExtensions.Add(ref Values, value);
				ArrayExtensions.Add(ref Names, name);
				ArrayExtensions.Add(ref Weights, weight);
			} else {
				Dim += 1;
				Values[Dim-1] = value;
			}
		}

		public void Feed(float[] values, string name, float weight=1f) {
			for(int i=0; i<values.Length; i++) {
				Feed(values[i], name + (i+1), weight);
			}
		}

		public void Feed(bool[] values, string name, float weight=1f) {
			for(int i=0; i<values.Length; i++) {
				Feed(values[i] ? 1f : 0f, name + (i+1), weight);
			}
		}

		public void Feed(float[,] values, string name, float weight=1f) {
			for(int i=0; i<values.GetLength(0); i++) {
				for(int j=0; j<values.GetLength(1); j++) {
					Feed(values[i,j], name+(i*values.GetLength(1)+j+1), weight);
				}
			}
		}

		public void Feed(bool[,] values, string name, float weight=1f) {
			for(int i=0; i<values.GetLength(0); i++) {
				for(int j=0; j<values.GetLength(1); j++) {
					Feed(values[i,j] ? 1f : 0f, name+(i*values.GetLength(1)+j+1), weight);
				}
			}
		}

		public void Feed(Vector2 value, string name, float weight=1f) {
			Feed(value.x, name+"X", weight);
			Feed(value.y, name+"Y", weight);
		}

		public void Feed(Vector3 value, string name, float weight=1f) {
			Feed(value.x, name+"X", weight);
			Feed(value.y, name+"Y", weight);
			Feed(value.z, name+"Z", weight);
		}

		public void FeedXY(Vector3 value, string name, float weight=1f) {
			Feed(value.x, name+"X", weight);
			Feed(value.y, name+"Y", weight);
		}

		public void FeedXZ(Vector3 value, string name, float weight=1f) {
			Feed(value.x, name+"X", weight);
			Feed(value.z, name+"Z", weight);
		}

		public void FeedYZ(Vector3 value, string name, float weight=1f) {
			Feed(value.y, name+"Y", weight);
			Feed(value.z, name+"Z", weight);
		}

		private void WriteData() {
			while(Exporting && (!Finished || Buffer.Count > 0)) {
				if(Buffer.Count > 0) {
					float[] item;
					lock(Buffer) {
						item = Buffer.Dequeue();	
					}
					//Update Mean and Std
					for(int i=0; i<item.Length; i++) {
						Statistics[i].Add(item[i]);
					}
					//Write to File
					File.WriteLine(String.Join(Separator, Array.ConvertAll(item, x => x.ToString(Accuracy,new CultureInfo("en-US")))));
				} else {
					Thread.Sleep(1);
				}
			}
		}

		public void Store() {
			if(!Setup) {
				//Setup Mean and Std
				Statistics = new RunningStatistics[Values.Length];
				for(int i=0; i<Statistics.Length; i++) {
					Statistics[i] = new RunningStatistics();
				}

				//Write Labels
				for(int i=0; i<Names.Length; i++) {
					Labels.WriteLine("[" + i + "]" + " " + Names[i]);
				}
				Labels.Close();

				Setup = true;
			}

			//Enqueue Sample
			float[] item = (float[])Values.Clone();
			lock(Buffer) {
				Buffer.Enqueue(item);
			}

			//Reset Running Index
			Dim = 0;
		}

		public void Finish() {
			Finished = true;

			Task.WaitAll(Writer);

			File.Close();

			if(Setup) {
				//Write Mean
				float[] mean = new float[Statistics.Length];
				for(int i=0; i<mean.Length; i++) {
					mean[i] = Statistics[i].Mean();
				}
				Norm.WriteLine(String.Join(Separator, Array.ConvertAll(mean, x => x.ToString(Accuracy,new CultureInfo("en-US")))));

				//Write Std
				float[] std = new float[Statistics.Length];
				for(int i=0; i<std.Length; i++) {
					std[i] = Statistics[i].Std();
				}
				Norm.WriteLine(String.Join(Separator, Array.ConvertAll(std, x => x.ToString(Accuracy,new CultureInfo("en-US")))));
			}

			Norm.Close();
		}
	}

	[Serializable]
	public class LabelGroup {
		
		public string[] Labels;
		
		private int[] Indices;
		
		public LabelGroup(params string[] labels) {
			Labels = labels;
		}

		public string GetID() {
			string id = string.Empty;
			for(int i=0; i<Labels.Length; i++) {
				id += Labels[i];
			}
			return id;
		}

		public void Setup(string[] references) {
			List<int> indices = new List<int>();
			for(int i=0; i<references.Length; i++) {
				if(ArrayExtensions.Contains(ref Labels, references[i])) {
					indices.Add(i);
				}
			}
			Indices = indices.ToArray();
		}

		public float Filter(float[] values) {
			float value = 0f;
			for(int i=0; i<Indices.Length; i++) {
				value += values[Indices[i]];
			}
			if(value > 1f) {
				Debug.Log("Value larger than expected.");
			}
			return value;
		}

	}

	private string GetExportPath() {
		string path = Application.dataPath;
		path = path.Substring(0, path.LastIndexOf("/"));
		path = path.Substring(0, path.LastIndexOf("/"));
		path += "/Export";
		return path;
	}

	private IEnumerator ExportDataSIGGRAPHAsia() {
		if(Editor == null) {
			Debug.Log("No editor found.");
		} else if(!System.IO.Directory.Exists(Application.dataPath + "/../../Export")) {
			Debug.Log("No export folder found at " + GetExportPath() + ".");
		} else {
			Exporting = true;

			Progress = 0f;

			int total = 0;
			int items = 0;
			int sequence = 0;
			DateTime timestamp = Utility.GetTimestamp();

			Data X = new Data(CreateFile("Input"), CreateFile("InputNorm"), CreateFile("InputLabels"));
			Data Y = new Data(CreateFile("Output"), CreateFile("OutputNorm"), CreateFile("OutputLabels"));

			StreamWriter S = CreateFile("Sequences");

			bool editorSave = Editor.Save;
			bool editorMirror = Editor.Mirror;
			float editorRate = Editor.TargetFramerate;
			int editorSeed = Editor.RandomSeed;
			Editor.Save = false;
			Editor.SetTargetFramerate(Framerate);
			for(int i=0; i<Files.Count; i++) {
				if(!Exporting) {
					break;
				}
				if(Export[i]) {
					Index = i;
					Editor.LoadData(Files[i]);
					while(!Editor.GetData().GetScene().isLoaded) {
						Debug.Log("Waiting for scene to be loaded.");
						yield return new WaitForSeconds(0f);
					}
					for(int m=1; m<=2; m++) {
						if(!Exporting) {
							break;
						}
						if(m==1) {
							Editor.SetMirror(false);
						}
						if(m==2) {
							Editor.SetMirror(true);
						}
						if(!Editor.Mirror || WriteMirror && Editor.Mirror && Editor.GetData().Symmetric) {
							Debug.Log("File: " + Editor.GetData().GetName() + " Scene: " + Editor.GetData().GetName() + " " + (Editor.Mirror ? "[Mirror]" : "[Default]"));

							//foreach(Sequence seq in Editor.GetData().Sequences) {
							Sequence seq = Editor.GetData().GetUnrolledSequence(); {
								sequence += 1;

								//Precomputations
								for(int j=0; j<Actions.Count; j++) {
									Actions[j].Setup(((GoalModule)Editor.GetData().GetModule(Module.ID.Goal)).GetNames());
								}
								for(int j=0; j<Styles.Count; j++) {
									Styles[j].Setup(((StyleModule)Editor.GetData().GetModule(Module.ID.Style)).GetNames());
								}

								if(
									(((GoalModule)Editor.GetData().GetModule(Module.ID.Goal)).GetGoalFunction("Sit") != null && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
									||
									(((StyleModule)Editor.GetData().GetModule(Module.ID.Style)).GetStyleFunction("Climb") != null && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
									||
									(Editor.GetData().name.Contains("Shelf") && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
								) {
									Debug.LogError("No editing in file " + Editor.GetData().name + "!");								
								}

								//Exporting
								float start = Editor.CeilToTargetTime(Editor.GetData().GetFrame(seq.Start).Timestamp);
								float end = Editor.FloorToTargetTime(Editor.GetData().GetFrame(seq.End).Timestamp);
								int sample = 0;
								while(start+(sample+1)/Framerate <= end) {
									if(!Exporting) {
										break;
									}
									Editor.SetRandomSeed(sample+1);
									InputSIGGRAPHAsia current = new InputSIGGRAPHAsia(Editor, start+sample/Framerate);
									sample += 1;
									OutputSIGGRAPHAsia next = new OutputSIGGRAPHAsia(Editor, start+sample/Framerate);
									
									//Write Sequence
									S.WriteLine(sequence.ToString());

									if(current.Frame.Index+2 != next.Frame.Index) {
										Debug.Log("Oups! Something went wrong with frame sampling from " + current.Frame.Index + " to " + next.Frame.Index + " at target framerate " + Framerate + ". This should not have happened!");
									}
									
									//Input
									//Auto-Regressive Posture
									for(int k=0; k<current.Posture.Length; k++) {
										X.Feed(current.Posture[k].GetPosition().GetRelativePositionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
										X.Feed(current.Posture[k].GetForward().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Forward");
										X.Feed(current.Posture[k].GetUp().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Up");
										X.Feed(current.Velocities[k].GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Velocity");
									}

									//Auto-Regressive Trajectory
									for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
										X.FeedXZ(current.RootSeries.GetPosition(k).GetRelativePositionTo(current.Root), "Trajectory"+(k+1)+"Position");
										X.FeedXZ(current.RootSeries.GetDirection(k).GetRelativeDirectionTo(current.Root), "Trajectory"+(k+1)+"Direction");
										for(int c=0; c<Styles.Count; c++) {
											X.Feed(Styles[c].Filter(current.StyleSeries.Values[k]), "Trajectory"+(k+1)+"Style"+"-"+Styles[c].GetID());
										}
									}

									//Goals
									for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
										X.Feed(current.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
										X.Feed(current.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
										for(int c=0; c<Actions.Count; c++) {
											X.Feed(Actions[c].Filter(current.GoalSeries.Values[k]), "Action"+(k+1)+"-"+Actions[c].GetID());
										}
									}


									if(UseHeightMap){
										//Height map
										for(int k=0; k<current.EnvironmentMap.Points.Length; k++) {
											X.Feed(current.EnvironmentMap.Points[k].GetRelativePositionTo(current.Root), "HeightMapPoint"+(k+1));
										}
									}else{
										//Environment Geometry
										X.Feed(current.Environment.Occupancies, "Environment-");
										
										//Interaction Geometry
										for(int k=0; k<current.Interaction.Points.Length; k++) {
											X.Feed(current.Interaction.References[k].GetRelativePositionTo(current.Root), "InteractionPosition"+(k+1));
											X.Feed(current.Interaction.Occupancies[k], "InteractionOccupancy"+(k+1));
										}
									}

									//Gating Variables
									X.Feed(GenerateGatingInteractionSIGGRAPHAsia(current), "Gating-");

									//Output
									//Auto-Regressive Posture
									for(int k=0; k<next.Posture.Length; k++) {
										Y.Feed(next.Posture[k].GetPosition().GetRelativePositionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
										Y.Feed(next.Posture[k].GetForward().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Forward");
										Y.Feed(next.Posture[k].GetUp().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Up");
										Y.Feed(next.Velocities[k].GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Velocity");
									}

									//Inverse Posture
									for(int k=0; k<next.Posture.Length; k++) {
										Y.Feed(next.Posture[k].GetPosition().GetRelativePositionTo(current.RootSeries.Transformations.Last()), "InverseBone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
									}

									//Auto-Regressive Trajectory
									for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
										Y.FeedXZ(next.RootSeries.GetPosition(k).GetRelativePositionTo(current.Root), "Trajectory"+(k+1)+"Position");
										Y.FeedXZ(next.RootSeries.GetDirection(k).GetRelativeDirectionTo(current.Root), "Trajectory"+(k+1)+"Direction");
										for(int c=0; c<Styles.Count; c++) {
											Y.Feed(Styles[c].Filter(next.StyleSeries.Values[k]), "Trajectory"+(k+1)+"Style"+"-"+Styles[c].GetID());
										}
									}

									//Inverse Trajectory
									for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
										Y.FeedXZ(next.RootSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.GoalSeries.Transformations[next.TimeSeries.Pivot]), "InverseTrajectoryPosition"+"-"+(k+1));
										Y.FeedXZ(next.RootSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.GoalSeries.Transformations[next.TimeSeries.Pivot]), "InverseTrajectoryDirection"+"-"+(k+1));
									}

									//Goals
									for(int k=0; k<next.TimeSeries.Samples.Length; k++) {
										Y.Feed(next.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
										Y.Feed(next.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
										for(int c=0; c<Actions.Count; c++) {
											Y.Feed(Actions[c].Filter(next.GoalSeries.Values[k]), "Action"+(k+1)+"-"+Actions[c].GetID());
										}
									}

									//Key Contacts
									//Y.Feed(next.ContactSeries.GetContacts(next.TimeSeries.Pivot, "Hips", "RightWrist", "LeftWrist", "RightAnkle", "LeftAnkle"), "Contact-");
									Y.Feed(next.ContactSeries.GetContacts(next.TimeSeries.Pivot, "RightAnkle", "LeftAnkle", "RightWrist", "LeftWrist"), "Contact-");

									//Phase Update
									List<float> values = new List<float>();
									values.Add(current.PhaseSeries.Values[current.TimeSeries.Pivot]);
									for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
										values.Add(next.PhaseSeries.Values[k]);
										Y.Feed(Utility.PhaseUpdate(values.ToArray()), "PhaseUpdate-"+(k+1));
									}

									//Write Line
									X.Store();
									Y.Store();

									Progress = (sample/Framerate) / (end-start);
									total += 1;
									items += 1;
									if(items >= BatchSize) {
										Performance = items / (float)Utility.GetElapsedTime(timestamp);
										timestamp = Utility.GetTimestamp();
										items = 0;
										yield return new WaitForSeconds(0f);
									}
								}

								//Reset Progress
								Progress = 0f;

								//Collect Garbage
								EditorUtility.UnloadUnusedAssetsImmediate();
								Resources.UnloadUnusedAssets();
								GC.Collect();
							}
						}
					}
				}
			}
			Editor.Save = editorSave;
			Editor.SetMirror(editorMirror);
			Editor.SetTargetFramerate(editorRate);
			Editor.SetRandomSeed(editorSeed);

			S.Close();

			X.Finish();
			Y.Finish();

			Index = -1;
			Exporting = false;
			yield return new WaitForSeconds(0f);

			Debug.Log("Exported " + total + " samples.");
		}
	}

	private IEnumerator ExportDataSteps() {
		if(Editor == null) {
			Debug.Log("No editor found.");
		} else if(!System.IO.Directory.Exists(Application.dataPath + "/../../Export")) {
			Debug.Log("No export folder found at " + GetExportPath() + ".");
		} else {
			Exporting = true;

			Progress = 0f;

			int total = 0;
			int items = 0;
			int sequence = 0;
			DateTime timestamp = Utility.GetTimestamp();

			Data X = new Data(CreateFile("Input"), CreateFile("InputNorm"), CreateFile("InputLabels"));
			Data Y = new Data(CreateFile("Output"), CreateFile("OutputNorm"), CreateFile("OutputLabels"));

			StreamWriter S = CreateFile("Sequences");

			bool editorSave = Editor.Save;
			bool editorMirror = Editor.Mirror;
			float editorRate = Editor.TargetFramerate;
			int editorSeed = Editor.RandomSeed;
			Editor.Save = false;
			Editor.SetTargetFramerate(Framerate);
			//int numberOfLaps = 5;
			bool exclude = false;
			if(numberOfLaps < 1) numberOfLaps = 1;
			if(numberOfLaps > 10) numberOfLaps = 10;

			for(int lap=0; lap < numberOfLaps; lap++){
				for(int i=0; i<Files.Count; i++) {
					if(!Exporting) {
						break;
					}
					// "WalkRandom" only exported once
					if((Files[i].GetName() == "WalkRandom") && (exclude == true)) continue;
					if((Files[i].GetName() == "WalkRandom") && (exclude == false)) exclude = true;
					if(Export[i]) {
						Index = i;
						Editor.LoadData(Files[i]);
						while(!Editor.GetData().GetScene().isLoaded) {
							Debug.Log("Waiting for scene to be loaded.");
							yield return new WaitForSeconds(0f);
						}
						for(int m=1; m<=2; m++) {
							if(!Exporting) {
								break;
							}
							if(m==1) {
								Editor.SetMirror(false);
							}
							if(m==2) {
								Editor.SetMirror(true);
							}
							if(!Editor.Mirror || WriteMirror && Editor.Mirror && Editor.GetData().Symmetric) {
								Debug.Log("File: " + Editor.GetData().GetName() + " Scene: " + Editor.GetData().GetName() + " " + (Editor.Mirror ? "[Mirror]" : "[Default]"));

								//foreach(Sequence seq in Editor.GetData().Sequences) {
								Sequence seq = Editor.GetData().GetUnrolledSequence(); {
									sequence += 1;

									//Precomputations
									for(int j=0; j<Actions.Count; j++) {
										Actions[j].Setup(((GoalModule)Editor.GetData().GetModule(Module.ID.Goal)).GetNames());
									}
									for(int j=0; j<Styles.Count; j++) {
										Styles[j].Setup(((StyleModule)Editor.GetData().GetModule(Module.ID.Style)).GetNames());
									}

									if(
										(((GoalModule)Editor.GetData().GetModule(Module.ID.Goal)).GetGoalFunction("Sit") != null && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
										||
										(((StyleModule)Editor.GetData().GetModule(Module.ID.Style)).GetStyleFunction("Climb") != null && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
										||
										(Editor.GetData().name.Contains("Shelf") && ((ContactModule)Editor.GetData().GetModule(Module.ID.Contact)).EditMotion == false)
									) {
										Debug.LogError("No editing in file " + Editor.GetData().name + "!");								
									}

									//Exporting
									float start = Editor.CeilToTargetTime(Editor.GetData().GetFrame(seq.Start).Timestamp);
									float end = Editor.FloorToTargetTime(Editor.GetData().GetFrame(seq.End).Timestamp);
									int sample = 0;
									while(start+(sample+1)/Framerate <= end) {
										if(!Exporting) {
											break;
										}
										Editor.SetRandomSeed(sample+1);
										InputSteps current = new InputSteps(Editor, start+sample/Framerate);
										sample += 1;
										OutputSteps next = new OutputSteps(Editor, start+sample/Framerate);
										
										//Write Sequence
										S.WriteLine(sequence.ToString());

										if(current.Frame.Index+2 != next.Frame.Index) {
											Debug.Log("Oups! Something went wrong with frame sampling from " + current.Frame.Index + " to " + next.Frame.Index + " at target framerate " + Framerate + ". This should not have happened!");
										}
										
										//Input
										//Auto-Regressive Posture
										for(int k=0; k<current.Posture.Length; k++) {
											X.Feed(current.Posture[k].GetPosition().GetRelativePositionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
											X.Feed(current.Posture[k].GetForward().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Forward");
											X.Feed(current.Posture[k].GetUp().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Up");
											X.Feed(current.Velocities[k].GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Velocity");
										}

										//Auto-Regressive Trajectory
										for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
											X.FeedXZ(current.RootSeries.GetPosition(k).GetRelativePositionTo(current.Root), "Trajectory"+(k+1)+"Position");
											X.FeedXZ(current.RootSeries.GetDirection(k).GetRelativeDirectionTo(current.Root), "Trajectory"+(k+1)+"Direction");
											for(int c=0; c<Styles.Count; c++) {
												X.Feed(Styles[c].Filter(current.StyleSeries.Values[k]), "Trajectory"+(k+1)+"Style"+"-"+Styles[c].GetID());
											}
										}

										//Auto-Regressive Feet Trajectories
										//If there is a contact, post-process point depending on tread variation, otherwise leave it as is
										ContactModule CModule = (ContactModule)Editor.GetData().GetModule(Module.ID.Contact);
										Vector3 leftFootTrajectoryPos = Vector3.zero;
										Vector3 rightFootTrajectoryPos = Vector3.zero;
										Frame timeseriesFrameRelativeToCurrent;
										float[] leftContacts = new float[0];
										float[] rightContacts = new float[0]; //new float[current.TimeSeries.Samples.Length];
										float stepIncrement = Mathf.Max(CModule.Step, 1)/Editor.GetData().Framerate;
										int currentFrameIndex = current.TimeSeries.GetPivot().Index;
										float frameTimestamp = current.Frame.Timestamp;
										/*if(Editor.Mirror){
											leftContacts = CModule.GetSensor("LeftAnkle").InverseContacts;
											rightContacts = CModule.GetSensor("RightAnkle").InverseContacts;
										}else{
											leftContacts = CModule.GetSensor("LeftAnkle").RegularContacts;
											rightContacts = CModule.GetSensor("RightAnkle").RegularContacts;
										}*/
										for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
											leftContacts = current.ContactSeries.GetContacts(k, "LeftAnkle");
											rightContacts = current.ContactSeries.GetContacts(k, "RightAnkle");
											/*if(leftContacts[0] > 0f || rightContacts[0] > 0f){
												//leftFootTrajectoryPos = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), current.FeetSeries.GetLeftFootPosition(k), Editor.Mirror);
												//leftFootTrajectoryPos = CModule.GetCorrectedStraightStairWalkingStep(current.Frame, Editor.GetActor().GetRoot(), current.FeetSeries.GetLeftFootPosition(k), "LeftAnkle", Editor.Mirror);
												leftFootTrajectoryPos = current.FeetSeries.GetLeftFootPosition(k);
												leftFootTrajectoryPos += CModule.GetFrameYCorrectionVector(Editor, current.FeetSeries.GetLeftFootPosition(current.TimeSeries.Pivot), Editor.Mirror, "LeftAnkle");
											}else{
												leftFootTrajectoryPos = current.FeetSeries.GetLeftFootPosition(k);
											}*/
											//Debug.Log("LeftFoot input = " + leftFootTrajectoryPos);
											leftFootTrajectoryPos = current.FeetSeries.GetLeftFootPosition(k);
											if(Files[i].GetName() != "WalkRandom"){
												timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(frameTimestamp + (k - currentFrameIndex) * stepIncrement);
												leftFootTrajectoryPos = CModule.GetCorrectedPointWithRelativeContacts(timeseriesFrameRelativeToCurrent, "LeftAnkle", Editor.Mirror);
											}
											X.Feed(leftFootTrajectoryPos.GetRelativePositionTo(current.Root), "LeftFootTrajectory"+(k+1)+"Position");
											//X.Feed(current.FeetSeries.GetLeftFootDirection(k).GetRelativeDirectionTo(current.Root), "LeftFootTrajectory"+(k+1)+"Direction");
											
											/*if(rightContacts[0] > 0f || rightContacts[0] > 0f){
												//rightFootTrajectoryPos = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), current.FeetSeries.GetRightFootPosition(k), Editor.Mirror);
												//rightFootTrajectoryPos = CModule.GetCorrectedStraightStairWalkingStep(current.Frame, Editor.GetActor().GetRoot(), current.FeetSeries.GetRightFootPosition(k), "RightAnkle", Editor.Mirror);
												rightFootTrajectoryPos = current.FeetSeries.GetRightFootPosition(k);
												rightFootTrajectoryPos += CModule.GetFrameYCorrectionVector(Editor, current.FeetSeries.GetRightFootPosition(current.TimeSeries.Pivot), Editor.Mirror, "RightAnkle");
											}else{
												rightFootTrajectoryPos = current.FeetSeries.GetRightFootPosition(k);
											}*/
											//Debug.Log("RightFoot input = " + rightFootTrajectoryPos);
											//Frame timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(current.Frame.Timestamp + (k - currentFrameIndex) * stepIncrement);
											rightFootTrajectoryPos = current.FeetSeries.GetRightFootPosition(k);
											if(Files[i].GetName() != "WalkRandom"){
												timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(frameTimestamp + (k - currentFrameIndex) * stepIncrement);
												rightFootTrajectoryPos = CModule.GetCorrectedPointWithRelativeContacts(timeseriesFrameRelativeToCurrent, "RightAnkle", Editor.Mirror);
											}
											X.Feed(rightFootTrajectoryPos.GetRelativePositionTo(current.Root), "RightFootTrajectory"+(k+1)+"Position");
											//X.Feed(current.FeetSeries.GetRightFootDirection(k).GetRelativeDirectionTo(current.Root), "RightFootTrajectory"+(k+1)+"Direction");
										}

										//Goals
										for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
											X.Feed(current.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
											X.Feed(current.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
											for(int c=0; c<Actions.Count; c++) {
												X.Feed(Actions[c].Filter(current.GoalSeries.Values[k]), "Action"+(k+1)+"-"+Actions[c].GetID());
											}
										}

										//Feet Goals (Preprocessing with current Step Offset before writing)
										//ContactModule CModule = (ContactModule)Editor.GetData().GetModule(Module.ID.Contact);
										//ContactModule.Sensor LeftAnkleSensor = CModule.GetSensor("LeftAnkle");
										//ContactModule.Sensor RightAnkleSensor = CModule.GetSensor("RightAnkle");
										/*for(int k=0; k<current.TimeSeries.Samples.Length; k++) {
											Vector3 LeftFutureGoalPoint = current.FeetSeries.GetFutureLeftFootPosition(k);
											//Vector3 LeftHorizontalOffset = LeftAnkleSensor.GetHorizontalStepOffsetVector(current.Frame, Editor.Mirror);
											//LeftFutureGoalPoint.y = Utility.GetHeight(LeftFutureGoalPoint, LayerMask.GetMask("Ground","Interaction")); // Adapting according to vertical noise
											LeftFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), LeftFutureGoalPoint, Editor.Mirror);
											X.Feed((LeftFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureLeftFootGoalPosition"+"-"+(k+1));
											//X.FeedXZ(current.FeetSeries.GetFutureLeftFootDirection(k).GetRelativeDirectionTo(current.Root), "FutureLeftFootGoalDirection"+"-"+(k+1));
											//Debug.Log("Motion exporter Left Future Goal Direction "  + k + " = " + current.FeetSeries.GetFutureLeftFootDirection(k));
											Vector3 RightFutureGoalPoint = current.FeetSeries.GetFutureRightFootPosition(k);
											//Vector3 RightHorizontalOffset = RightAnkleSensor.GetHorizontalStepOffsetVector(current.Frame, Editor.Mirror);
											//RightFutureGoalPoint.y = Utility.GetHeight(LeftFutureGoalPoint, LayerMask.GetMask("Ground","Interaction"));
											RightFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), RightFutureGoalPoint, Editor.Mirror);
											X.Feed((RightFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureRightFootGoalPosition"+"-"+(k+1));
											//X.FeedXZ(current.FeetSeries.GetFutureRightFootDirection(k).GetRelativeDirectionTo(current.Root), "FutureRightFootGoalDirection"+"-"+(k+1));
											//Debug.Log("Motion exporter Right Future Goal Direction "  + k + " = " + current.FeetSeries.GetFutureRightFootDirection(k));
											//X.Feed(current.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
											//X.Feed(current.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
										}*/

										//Feet Goal 3D position of current frame
										//Vector3 LeftFutureGoalPoint = current.FeetSeries.GetFutureLeftFootPosition(current.TimeSeries.Pivot);
										//LeftFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), LeftFutureGoalPoint, Editor.Mirror);
										//LeftFutureGoalPoint = CModule.GetCorrectedStraightStairWalkingStep(current.Frame, Editor.GetActor().GetRoot(), LeftFutureGoalPoint, "LeftAnkle", Editor.Mirror);
										//LeftFutureGoalPoint = CModule.GetCorrectedFutureStairWalkingGoal(current.Frame, Editor.GetActor().GetRoot(), LeftFutureGoalPoint, "LeftAnkle", Editor.Mirror);
										Vector3 LeftFutureGoalPoint = CModule.GetRelativeToTreadFutureContactPoint(current.Frame, "LeftAnkle", Editor.Mirror);
										string treadName = CModule.GetFutureContactColliderName(current.Frame, "LeftAnkle", Editor.Mirror);
										Matrix4x4 treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
										LeftFutureGoalPoint = LeftFutureGoalPoint.GetRelativePositionFrom(treadWorldMatrix);
										LeftFutureGoalPoint = CModule.GetProjectedPoint(LeftFutureGoalPoint);
										X.Feed((LeftFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureLeftFootGoalPosition"+"-"+(current.TimeSeries.Pivot+1));
										//Vector3 RightFutureGoalPoint = current.FeetSeries.GetFutureRightFootPosition(current.TimeSeries.Pivot);
										//RightFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), RightFutureGoalPoint, Editor.Mirror);
										//RightFutureGoalPoint = CModule.GetCorrectedStraightStairWalkingStep(current.Frame, Editor.GetActor().GetRoot(), RightFutureGoalPoint, "RightAnkle", Editor.Mirror);
										//RightFutureGoalPoint = CModule.GetCorrectedFutureStairWalkingGoal(current.Frame, Editor.GetActor().GetRoot(), RightFutureGoalPoint, "RightAnkle", Editor.Mirror);
										Vector3 RightFutureGoalPoint = CModule.GetRelativeToTreadFutureContactPoint(current.Frame, "RightAnkle", Editor.Mirror);
										treadName = CModule.GetFutureContactColliderName(current.Frame, "RightAnkle", Editor.Mirror);
										treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
										RightFutureGoalPoint = RightFutureGoalPoint.GetRelativePositionFrom(treadWorldMatrix);
										RightFutureGoalPoint = CModule.GetProjectedPoint(RightFutureGoalPoint);
										X.Feed((RightFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureRightFootGoalPosition"+"-"+(current.TimeSeries.Pivot+1));

										if(UseHeightMap){
											//Height map
											for(int k=0; k<current.EnvironmentMap.Points.Length; k++) {
												X.Feed(current.EnvironmentMap.Points[k].GetRelativePositionTo(current.Root), "HeightMapPoint"+(k+1));
											}
										}else{
											//Environment Geometry
											X.Feed(current.Environment.Occupancies, "Environment-");
										}

										/*//Interaction Geometry
										for(int k=0; k<current.Interaction.Points.Length; k++) {
											X.Feed(current.Interaction.References[k].GetRelativePositionTo(current.Root), "InteractionPosition"+(k+1));
											X.Feed(current.Interaction.Occupancies[k], "InteractionOccupancy"+(k+1));
										}*/

										//Gating Variables
										X.Feed(GenerateGatingInteractionSteps(current), "Gating-");

										//Output
										//Auto-Regressive Posture
										for(int k=0; k<next.Posture.Length; k++) {
											Y.Feed(next.Posture[k].GetPosition().GetRelativePositionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
											Y.Feed(next.Posture[k].GetForward().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Forward");
											Y.Feed(next.Posture[k].GetUp().GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Up");
											Y.Feed(next.Velocities[k].GetRelativeDirectionTo(current.Root), "Bone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Velocity");
										}

										//Inverse Posture
										for(int k=0; k<next.Posture.Length; k++) {
											Y.Feed(next.Posture[k].GetPosition().GetRelativePositionTo(current.RootSeries.Transformations.Last()), "InverseBone"+(k+1)+Editor.GetActor().Bones[k].GetName()+"Position");
										}

										//Auto-Regressive Feet Trajectories
										leftFootTrajectoryPos = Vector3.zero;
										rightFootTrajectoryPos = Vector3.zero;
										int nextFrameIndex = next.TimeSeries.Pivot;
										float nextFrameTimestamp = next.Frame.Timestamp;
										for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
											leftContacts = next.ContactSeries.GetContacts(k, "LeftAnkle");
											/*if(leftContacts[0] > 0f || rightContacts[0] > 0f){
												//leftFootTrajectoryPos = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), next.FeetSeries.GetLeftFootPosition(k), Editor.Mirror);
												//leftFootTrajectoryPos = CModule.GetCorrectedStraightStairWalkingStep(next.Frame, Editor.GetActor().GetRoot(), next.FeetSeries.GetLeftFootPosition(k), "LeftAnkle", Editor.Mirror);
												leftFootTrajectoryPos = next.FeetSeries.GetLeftFootPosition(k);
												leftFootTrajectoryPos += CModule.GetFrameYCorrectionVector(Editor, next.FeetSeries.GetLeftFootPosition(next.TimeSeries.Pivot), Editor.Mirror, "LeftAnkle");
											}else{
												leftFootTrajectoryPos = next.FeetSeries.GetLeftFootPosition(k);
											}*/
											//Debug.Log("LeftFoot output = " + leftFootTrajectoryPos);
											leftFootTrajectoryPos = next.FeetSeries.GetLeftFootPosition(k);
											if(Files[i].GetName() != "WalkRandom"){
												timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(nextFrameTimestamp + (k - nextFrameIndex) * stepIncrement);
												leftFootTrajectoryPos = CModule.GetCorrectedPointWithRelativeContacts(timeseriesFrameRelativeToCurrent, "LeftAnkle", Editor.Mirror);
											}
											Y.Feed(leftFootTrajectoryPos.GetRelativePositionTo(current.Root), "LeftFootTrajectory"+(k+1)+"Position");
											//Y.Feed(next.FeetSeries.GetLeftFootDirection(k).GetRelativeDirectionTo(current.Root), "LeftFootTrajectory"+(k+1)+"Direction");
											rightContacts = next.ContactSeries.GetContacts(k, "RightAnkle");
											/*if(rightContacts[0] > 0f || leftContacts[0] > 0f){
												//rightFootTrajectoryPos = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), next.FeetSeries.GetRightFootPosition(k), Editor.Mirror);
												//rightFootTrajectoryPos = CModule.GetCorrectedStraightStairWalkingStep(next.Frame, Editor.GetActor().GetRoot(), next.FeetSeries.GetRightFootPosition(k), "RightAnkle", Editor.Mirror);
												rightFootTrajectoryPos = next.FeetSeries.GetRightFootPosition(k);
												rightFootTrajectoryPos += CModule.GetFrameYCorrectionVector(Editor, next.FeetSeries.GetRightFootPosition(next.TimeSeries.Pivot), Editor.Mirror, "RightAnkle");
											}else{
												rightFootTrajectoryPos = next.FeetSeries.GetRightFootPosition(k);
											}*/
											//Debug.Log("RightFoot output = " + rightFootTrajectoryPos);
											//timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(nextFrameTimestamp + (k - nextFrameIndex) * stepIncrement);
											rightFootTrajectoryPos = next.FeetSeries.GetRightFootPosition(k);
											if(Files[i].GetName() != "WalkRandom"){
												timeseriesFrameRelativeToCurrent = Editor.GetData().GetFrame(nextFrameTimestamp + (k - nextFrameIndex) * stepIncrement);
												rightFootTrajectoryPos = CModule.GetCorrectedPointWithRelativeContacts(timeseriesFrameRelativeToCurrent, "RightAnkle", Editor.Mirror);
											}
											Y.Feed(rightFootTrajectoryPos.GetRelativePositionTo(current.Root), "RightFootTrajectory"+(k+1)+"Position");
											//Y.Feed(next.FeetSeries.GetRightFootDirection(k).GetRelativeDirectionTo(current.Root), "RightFootTrajectory"+(k+1)+"Direction");
										}

										//Auto-Regressive Trajectory
										for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
											Y.FeedXZ(next.RootSeries.GetPosition(k).GetRelativePositionTo(current.Root), "Trajectory"+(k+1)+"Position");
											Y.FeedXZ(next.RootSeries.GetDirection(k).GetRelativeDirectionTo(current.Root), "Trajectory"+(k+1)+"Direction");
											for(int c=0; c<Styles.Count; c++) {
												Y.Feed(Styles[c].Filter(next.StyleSeries.Values[k]), "Trajectory"+(k+1)+"Style"+"-"+Styles[c].GetID());
											}
										}

										//Inverse Trajectory
										for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
											Y.FeedXZ(next.RootSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.GoalSeries.Transformations[next.TimeSeries.Pivot]), "InverseTrajectoryPosition"+"-"+(k+1));
											Y.FeedXZ(next.RootSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.GoalSeries.Transformations[next.TimeSeries.Pivot]), "InverseTrajectoryDirection"+"-"+(k+1));
										}

										//Goals
										for(int k=0; k<next.TimeSeries.Samples.Length; k++) {
											Y.Feed(next.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
											Y.Feed(next.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
											for(int c=0; c<Actions.Count; c++) {
												Y.Feed(Actions[c].Filter(next.GoalSeries.Values[k]), "Action"+(k+1)+"-"+Actions[c].GetID());
											}
										}

										/*//Feet Goals (Preprocessing with current Step Offset before writing)
										for(int k=0; k<next.TimeSeries.Samples.Length; k++) {
											Vector3 LeftFutureGoalPoint = next.FeetSeries.GetFutureLeftFootPosition(k);
											Vector3 LeftHorizontalOffset = LeftAnkleSensor.GetHorizontalStepOffsetVector(next.Frame, Editor.Mirror);
											LeftFutureGoalPoint.y = Utility.GetHeight(LeftFutureGoalPoint, LayerMask.GetMask("Ground","Interaction")); // Adapting according to vertical noise
											Y.Feed((LeftFutureGoalPoint + LeftHorizontalOffset).GetRelativePositionTo(current.Root), "FutureLeftFootGoalPosition"+"-"+(k+1));
											Y.FeedXZ(next.FeetSeries.GetFutureLeftFootDirection(k).GetRelativeDirectionTo(current.Root), "FutureLeftFootGoalDirection"+"-"+(k+1));
											//Debug.Log("Motion exporter Left Future Goal Direction "  + k + " = " + next.FeetSeries.GetFutureLeftFootDirection(k));
											Vector3 RightFutureGoalPoint = next.FeetSeries.GetFutureRightFootPosition(k);
											Vector3 RightHorizontalOffset = RightAnkleSensor.GetHorizontalStepOffsetVector(next.Frame, Editor.Mirror);
											RightFutureGoalPoint.y = Utility.GetHeight(LeftFutureGoalPoint, LayerMask.GetMask("Ground","Interaction"));
											Y.Feed((RightFutureGoalPoint + RightHorizontalOffset).GetRelativePositionTo(current.Root), "FutureRightFootGoalPosition"+"-"+(k+1));
											Y.FeedXZ(next.FeetSeries.GetFutureRightFootDirection(k).GetRelativeDirectionTo(current.Root), "FutureRightFootGoalDirection"+"-"+(k+1));
											//Debug.Log("Motion exporter Right Future Goal Direction "  + k + " = " + next.FeetSeries.GetFutureRightFootDirection(k));
											//X.Feed(current.GoalSeries.Transformations[k].GetPosition().GetRelativePositionTo(current.Root), "GoalPosition"+"-"+(k+1));
											//X.Feed(current.GoalSeries.Transformations[k].GetForward().GetRelativeDirectionTo(current.Root), "GoalDirection"+"-"+(k+1));
										}*/

										//Feet Goal 3D position of current frame
										//LeftFutureGoalPoint = next.FeetSeries.GetFutureLeftFootPosition(next.TimeSeries.Pivot);
										//LeftFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), LeftFutureGoalPoint, Editor.Mirror);
										//LeftFutureGoalPoint = CModule.GetCorrectedStraightStairWalkingStep(next.Frame, Editor.GetActor().GetRoot(), LeftFutureGoalPoint, "LeftAnkle", Editor.Mirror);
										//LeftFutureGoalPoint = CModule.GetCorrectedFutureStairWalkingGoal(next.Frame, Editor.GetActor().GetRoot(), LeftFutureGoalPoint, "LeftAnkle", Editor.Mirror);
										LeftFutureGoalPoint = CModule.GetRelativeToTreadFutureContactPoint(next.Frame, "LeftAnkle", Editor.Mirror);
										treadName = CModule.GetFutureContactColliderName(next.Frame, "LeftAnkle", Editor.Mirror);
										treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
										LeftFutureGoalPoint = LeftFutureGoalPoint.GetRelativePositionFrom(treadWorldMatrix);
										LeftFutureGoalPoint = CModule.GetProjectedPoint(LeftFutureGoalPoint);
										Y.Feed((LeftFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureLeftFootGoalPosition"+"-"+(next.TimeSeries.Pivot+1));
										//RightFutureGoalPoint = next.FeetSeries.GetFutureRightFootPosition(next.TimeSeries.Pivot);
										//RightFutureGoalPoint = CModule.GetCorrectedPointWithStepNoise(Editor.GetActor().GetRoot(), RightFutureGoalPoint, Editor.Mirror);
										//RightFutureGoalPoint = CModule.GetCorrectedStraightStairWalkingStep(next.Frame, Editor.GetActor().GetRoot(), RightFutureGoalPoint, "RightAnkle", Editor.Mirror);
										//RightFutureGoalPoint = CModule.GetCorrectedFutureStairWalkingGoal(next.Frame, Editor.GetActor().GetRoot(), RightFutureGoalPoint, "RightAnkle", Editor.Mirror);
										RightFutureGoalPoint = CModule.GetRelativeToTreadFutureContactPoint(next.Frame, "RightAnkle", Editor.Mirror);
										treadName = CModule.GetFutureContactColliderName(next.Frame, "RightAnkle", Editor.Mirror);
										treadWorldMatrix = GameObject.Find(treadName).transform.GetWorldMatrix();
										RightFutureGoalPoint = RightFutureGoalPoint.GetRelativePositionFrom(treadWorldMatrix);
										RightFutureGoalPoint = CModule.GetProjectedPoint(RightFutureGoalPoint);
										Y.Feed((RightFutureGoalPoint).GetRelativePositionTo(current.Root), "FutureRightFootGoalPosition"+"-"+(next.TimeSeries.Pivot+1));

										//Key Contacts
										//Y.Feed(next.ContactSeries.GetContacts(next.TimeSeries.Pivot, "Hips", "RightWrist", "LeftWrist", "RightAnkle", "LeftAnkle"), "Contact-");
										Y.Feed(next.ContactSeries.GetContacts(next.TimeSeries.Pivot, "RightAnkle", "LeftAnkle", "RightWrist", "LeftWrist"), "Contact-");

										//Phase Update
										List<float> values = new List<float>();
										values.Add(current.PhaseSeries.Values[current.TimeSeries.Pivot]);
										for(int k=next.TimeSeries.Pivot; k<next.TimeSeries.Samples.Length; k++) {
											values.Add(next.PhaseSeries.Values[k]);
											Y.Feed(Utility.PhaseUpdate(values.ToArray()), "PhaseUpdate-"+(k+1));
										}

										//Write Line
										X.Store();
										Y.Store();

										Progress = (sample/Framerate) / (end-start);
										total += 1;
										items += 1;
										if(items >= BatchSize) {
											Performance = items / (float)Utility.GetElapsedTime(timestamp);
											timestamp = Utility.GetTimestamp();
											items = 0;
											yield return new WaitForSeconds(0f);
										}
									}

									//Reset Progress
									Progress = 0f;

									//Collect Garbage
									EditorUtility.UnloadUnusedAssetsImmediate();
									Resources.UnloadUnusedAssets();
									GC.Collect();
								}
							}
						}
					}
				}
			}
			
			Editor.Save = editorSave;
			Editor.SetMirror(editorMirror);
			Editor.SetTargetFramerate(editorRate);
			Editor.SetRandomSeed(editorSeed);

			S.Close();

			X.Finish();
			Y.Finish();

			Index = -1;
			Exporting = false;
			yield return new WaitForSeconds(0f);

			Debug.Log("Exported " + total + " samples.");
		}
	}

	private float[] GenerateGatingInteractionSIGGRAPHAsia(InputSIGGRAPHAsia current) {
		List<float> values = new List<float>();
		for(int i=0; i<current.TimeSeries.Samples.Length; i++) {
			Vector2 phase = Utility.PhaseVector(current.PhaseSeries.Values[i]);
			for(int j=0; j<Styles.Count; j++) {
				float magnitude = Styles[j].Filter(current.StyleSeries.Values[i]);
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f, 1f);
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
			}
			for(int j=0; j<Actions.Count; j++) {
				float magnitude = Actions[j].Filter(current.GoalSeries.Values[i]);
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f, 1f);
				Matrix4x4 root = current.RootSeries.Transformations[i];
				root[1,3] = 0f;
				Matrix4x4 goal = current.GoalSeries.Transformations[i];
				goal[1,3] = 0f;
				float distance = Vector3.Distance(root.GetPosition(), goal.GetPosition());
				float angle = Quaternion.Angle(root.GetRotation(), goal.GetRotation());
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
				values.Add(magnitude * distance * phase.x);
				values.Add(magnitude * distance * phase.y);
				values.Add(magnitude * angle * phase.x);
				values.Add(magnitude * angle * phase.y);
			}
		}
		return values.ToArray();
	}

	private float[] GenerateGatingInteractionSteps(InputSteps current) {
		List<float> values = new List<float>();
		for(int i=0; i<current.TimeSeries.Samples.Length; i++) {
			Vector2 phase = Utility.PhaseVector(current.PhaseSeries.Values[i]);
			for(int j=0; j<Styles.Count; j++) {
				float magnitude = Styles[j].Filter(current.StyleSeries.Values[i]);
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f, 1f);
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
			}
			for(int j=0; j<Actions.Count; j++) {
				float magnitude = Actions[j].Filter(current.GoalSeries.Values[i]);
				magnitude = Utility.Normalise(magnitude, 0f, 1f, -1f, 1f);
				Matrix4x4 root = current.RootSeries.Transformations[i];
				root[1,3] = 0f;
				Matrix4x4 goal = current.GoalSeries.Transformations[i];
				goal[1,3] = 0f;
				float distance = Vector3.Distance(root.GetPosition(), goal.GetPosition());
				float angle = Quaternion.Angle(root.GetRotation(), goal.GetRotation());
				values.Add(magnitude * phase.x);
				values.Add(magnitude * phase.y);
				values.Add(magnitude * distance * phase.x);
				values.Add(magnitude * distance * phase.y);
				values.Add(magnitude * angle * phase.x);
				values.Add(magnitude * angle * phase.y);
			}
		}
		return values.ToArray();
	}

	public class InputSIGGRAPHAsia {
		public Frame Frame;
		public Matrix4x4 Root;
		public Matrix4x4[] Posture;
		public Vector3[] Velocities;
		public TimeSeries TimeSeries;
		public TimeSeries.Root RootSeries;
		public TimeSeries.Style StyleSeries;
		public TimeSeries.Goal GoalSeries;
		public TimeSeries.Contact ContactSeries;
		public TimeSeries.Phase PhaseSeries;
		public CylinderMap Environment;
		public CuboidMap Interaction;
		public HeightMap EnvironmentMap;

		public InputSIGGRAPHAsia(MotionEditor editor, float timestamp) {
			editor.LoadFrame(timestamp);
			Frame = editor.GetCurrentFrame();

			Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
			Posture = editor.GetActor().GetBoneTransformations();
			Velocities = editor.GetActor().GetBoneVelocities();
			TimeSeries = ((TimeSeriesModule)editor.GetData().GetModule(Module.ID.TimeSeries)).GetTimeSeries(Frame, editor.Mirror, 6, 6, 1f, 1f, 1, 1f/editor.TargetFramerate);
			RootSeries = (TimeSeries.Root)TimeSeries.GetSeries("Root");
			StyleSeries = (TimeSeries.Style)TimeSeries.GetSeries("Style");
			GoalSeries = (TimeSeries.Goal)TimeSeries.GetSeries("Goal");
			ContactSeries = (TimeSeries.Contact)TimeSeries.GetSeries("Contact");
			PhaseSeries = (TimeSeries.Phase)TimeSeries.GetSeries("Phase");

			Environment = ((CylinderMapModule)editor.GetData().GetModule(Module.ID.CylinderMap)).GetCylinderMap(editor, Frame, editor.Mirror);
			EnvironmentMap = ((HeightMapModule)editor.GetData().GetModule(Module.ID.HeightMap)).GetHeightMap(editor.GetActor());
			//Interaction = ((GoalModule)editor.GetData().GetModule(Module.ID.Goal)).Target.GetInteractionGeometry(Frame, editor.Mirror, 1f/editor.TargetFramerate);
			Interaction = ((CuboidMapModule)editor.GetData().GetModule(Module.ID.CuboidMap)).GetCuboidMap(editor, Frame, editor.Mirror);
		}
	}

	public class OutputSIGGRAPHAsia {
		public Frame Frame;
		public Matrix4x4 Root;
		public Matrix4x4[] Posture;
		public Vector3[] Velocities;
		public TimeSeries TimeSeries;
		public TimeSeries.Root RootSeries;
		public TimeSeries.Style StyleSeries;
		public TimeSeries.Goal GoalSeries;
		public TimeSeries.Contact ContactSeries;
		public TimeSeries.Phase PhaseSeries;

		public OutputSIGGRAPHAsia(MotionEditor editor, float timestamp) {
			editor.LoadFrame(timestamp);
			Frame = editor.GetCurrentFrame();
			
			Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
			Posture = editor.GetActor().GetBoneTransformations();
			Velocities = editor.GetActor().GetBoneVelocities();
			TimeSeries = ((TimeSeriesModule)editor.GetData().GetModule(Module.ID.TimeSeries)).GetTimeSeries(Frame, editor.Mirror, 6, 6, 1f, 1f, 1, 1f/editor.TargetFramerate);
			RootSeries = (TimeSeries.Root)TimeSeries.GetSeries("Root");
			StyleSeries = (TimeSeries.Style)TimeSeries.GetSeries("Style");
			GoalSeries = (TimeSeries.Goal)TimeSeries.GetSeries("Goal");
			ContactSeries = (TimeSeries.Contact)TimeSeries.GetSeries("Contact");
			PhaseSeries = (TimeSeries.Phase)TimeSeries.GetSeries("Phase");
		}
	}

	public class InputSteps {
		public Frame Frame;
		public Matrix4x4 Root;
		public Matrix4x4[] Posture;
		public Vector3[] Velocities;
		public TimeSeries TimeSeries;
		public TimeSeries.Root RootSeries;
		public TimeSeries.Feet FeetSeries;
		public TimeSeries.Style StyleSeries;
		public TimeSeries.Goal GoalSeries;
		public TimeSeries.Contact ContactSeries;
		public TimeSeries.Phase PhaseSeries;
		public CylinderMap Environment;
		public CuboidMap Interaction;
		public HeightMap EnvironmentMap;

		public InputSteps(MotionEditor editor, float timestamp) {
			editor.LoadFrame(timestamp);
			Frame = editor.GetCurrentFrame();

			Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
			Posture = editor.GetActor().GetBoneTransformations();
			Velocities = editor.GetActor().GetBoneVelocities();
			TimeSeries = ((TimeSeriesModule)editor.GetData().GetModule(Module.ID.TimeSeries)).GetTimeSeries(Frame, editor.Mirror, 6, 6, 1f, 1f, 1, 1f/editor.TargetFramerate);
			RootSeries = (TimeSeries.Root)TimeSeries.GetSeries("Root");
			FeetSeries = (TimeSeries.Feet)TimeSeries.GetSeries("Feet");
			StyleSeries = (TimeSeries.Style)TimeSeries.GetSeries("Style");
			GoalSeries = (TimeSeries.Goal)TimeSeries.GetSeries("Goal");
			ContactSeries = (TimeSeries.Contact)TimeSeries.GetSeries("Contact");
			PhaseSeries = (TimeSeries.Phase)TimeSeries.GetSeries("Phase");

			Environment = ((CylinderMapModule)editor.GetData().GetModule(Module.ID.CylinderMap)).GetCylinderMap(editor, Frame, editor.Mirror);
			EnvironmentMap = ((HeightMapModule)editor.GetData().GetModule(Module.ID.HeightMap)).GetHeightMap(editor.GetActor());
			//Interaction = ((GoalModule)editor.GetData().GetModule(Module.ID.Goal)).Target.GetInteractionGeometry(Frame, editor.Mirror, 1f/editor.TargetFramerate);
		}
	}

	public class OutputSteps {
		public Frame Frame;
		public Matrix4x4 Root;
		public Matrix4x4[] Posture;
		public Vector3[] Velocities;
		public TimeSeries TimeSeries;
		public TimeSeries.Root RootSeries;
		public TimeSeries.Feet FeetSeries;
		public TimeSeries.Style StyleSeries;
		public TimeSeries.Goal GoalSeries;
		public TimeSeries.Contact ContactSeries;
		public TimeSeries.Phase PhaseSeries;

		public OutputSteps(MotionEditor editor, float timestamp) {
			editor.LoadFrame(timestamp);
			Frame = editor.GetCurrentFrame();
			
			Root = editor.GetActor().GetRoot().GetWorldMatrix(true);
			Posture = editor.GetActor().GetBoneTransformations();
			Velocities = editor.GetActor().GetBoneVelocities();
			TimeSeries = ((TimeSeriesModule)editor.GetData().GetModule(Module.ID.TimeSeries)).GetTimeSeries(Frame, editor.Mirror, 6, 6, 1f, 1f, 1, 1f/editor.TargetFramerate);
			RootSeries = (TimeSeries.Root)TimeSeries.GetSeries("Root");
			FeetSeries = (TimeSeries.Feet)TimeSeries.GetSeries("Feet");
			StyleSeries = (TimeSeries.Style)TimeSeries.GetSeries("Style");
			GoalSeries = (TimeSeries.Goal)TimeSeries.GetSeries("Goal");
			ContactSeries = (TimeSeries.Contact)TimeSeries.GetSeries("Contact");
			PhaseSeries = (TimeSeries.Phase)TimeSeries.GetSeries("Phase");
		}
	}

}
#endif