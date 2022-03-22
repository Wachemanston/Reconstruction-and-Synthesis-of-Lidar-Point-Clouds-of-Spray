<style>
    .tb-title { text-align: center; line-height: 1rem; padding-bottom: 0; margin-bottom: 0; }
</style>
This is the official code release of the paper [Reconstruction and Synthesis of Lidar Point Clouds of Spray](https://ieeexplore.ieee.org/document/9705289) by Yi-Chien Shih, Wei-Hsiang Liao, Wen-Chieh Lin, Sai-Keung Wong and Chieh-Chih Wang.

# License

# Acknowledgments
We would like to thank the authors of [RapidJSON](https://github.com/Tencent/rapidjson/), [KDTree](https://github.com/viliwonka/KDTree), [Pointcloud](https://github.com/MaikelH/Pointcloud), and [VehicleTools](https://github.com/Unity-Technologies/VehicleTools) for their great work.

# Citation
If you find this work useful, please consider citing our paper.
```java
@inproceedings{YiChienShihRA-L22,
  author = {Yi-Chien Shih, Wei-Hsiang Liao, Wen-Chieh Lin, Sai-Keung Wong and Chieh-Chih Wang},
  title = {{Reconstruction and Synthesis of Lidar Point Clouds of Spray}},
  booktitle = {IEEE Robotics and Automation Letters (RA-L)},
  year = {2021},
}
```

# Overview
## Environment
The code has been successfully tested on
* CPU: Intel Core i7-8700
* GPU: NVIDIA GeForce 1080
* RAM: 32GB
* OS: Ubuntu 18.04
* Unity Version: 2021.1.7f1
* Data sequences come from Waymo

## File Organization
There are 3 folders in the zipped file. 
1. DetectionAnalyzer
2. SpraySimulation
3. Tools

Table 1 shows the corresponding implementation for the different steps. Note that the name of the scene used under Unity simulation is SpraySimSimple.

<p class="tb-title">Table 1. The phase and step are referred from Fig 2 in our method.</p>
<table>
    <tr>
        <th>Phase</th>
        <th>Step</th>
        <th>Corresponding Algorithm Location</th>
    </tr>
    <tr>
        <td rowspan="3">Simulation</td>
        <td>Particle Trajectory Simulation</td>
        <td>SpraySimulation/Assets/Script/SprayModel.cs</td>
    </tr>
    <tr>
        <td>Local Selection</td>
        <td>SpraySimulation/Assets/Script/SprayModel.cs
+ the creation of filter_nodup.txt (mentioned below)
</td>
    </tr>
    <tr>
        <td>Lidar Perception</td>
        <td>SpraySimulation/Assets/Script/Lidar.cs</td>
    </tr>
    <tr>
        <td rowspan="2">Data Processing</td>
        <td>Synthesis</td>
        <td>Tools/Synthesis_texture_based<br>Tools/Synthesis_particle_based</td>
    </tr>
    <tr>
        <td>Global Refinement</td>
        <td>Tools/Refinement</td>
    </tr>
</table>

# Getting Started
## Simulation
### Setup
Before running the simulation, we have to set some parameters. Table 2 shows the meaning of global parameters, and Table 3 shows the parameters we use under different situations. Note that our configuration are palced at `SpraySimulation/Config`.

<p class="tb-title">Table 2.</p>
<table>
    <tr>
        <th colspan="2">Global parameters: see SpraySimulation/Assets/.env</th>
    </tr>
    <tr>
        <td>OutputPCDPath</td>
        <td>The folder stores point cloud output.</td>
    </tr>
    <tr>
        <td>OutputFilterPath</td>
        <td>The folder stores filter output.</td>
    </tr>
    <tr>
        <td>SprayDetectionJSONPath</td>
        <td>The folder stores spray detection results (in JSON).</td>
    </tr>
    <tr>
        <td>SimAugmentConfig</td>
        <td>The place stores configuration.</td>
    </tr>
    <tr>
        <td>NotSegmentedDataPath</td>
        <td>The folder stores the original un-segmented point cloud file (with .pcd extension).</td>
    </tr>
    <tr>
        <td>ReconstructRefPath</td>
        <td>The folder stores the original segmented point cloud file (with .pcd extension).</td>
    </tr>
    <tr>
        <td>ManipulationCheckConfig</td>
        <td>The place stored configuration.</td>
    </tr>
    <tr>
        <td>DefaultFilter</td>
        <td>The default folder stores default filter_nodup.txt/filter.txt. </td>
    </tr>
</table>

<p class="tb-title">Table 3. The game object is listed in Hierarchy window and attribute shows in Inspector window.</p>
<table>
    <tr>
        <th rowspan="2">Game Object</th>
        <th rowspan="2">Attribute</th>
        <th colspan="4">Situation</th>
    </tr>
    <tr>
        <td>Simulate with  Random Particles</td>
        <td>Simulate with  Filtered Particles</td>
        <td>Manipulation Check (Filter Particles)</td>
        <td>Data Augmentation</td>
    </tr>
    <tr>
        <td rowspan="3">Vehicles/EgoVehicleSpraySet</td>
        <td>ParticleCountMultiplier </td>
        <td>10,000</td>
        <td>50,000</td>
        <td>10,000</td>
        <td>50,000</td>
    </tr>
    <tr>
        <td>VisibleParticleConfig</td>
        <td>CloseToRealData</td>
        <td>All</td>
        <td>All</td>
        <td>All</td>
    </tr>
    <tr>
        <td>SprayInitialStatusConfig</td>
        <td>Random</td>
        <td>Filtered</td>
        <td>Random</td>
        <td>Filtered</td>
    </tr>
    <tr>
        <td>Lidar</td>
        <td>Is Exporting Filter </td>
        <td>true</td>
        <td>false</td>
        <td>false</td>
        <td>false</td>
    </tr>
    <tr>
        <td colspan="2">Individual Setting</td>
        <td>Set <code>ReconstructRefPath</code>(usually the ground-truth)and remained routing at Lidar.cs, line 169 (if needed).</td>
        <td></td>
        <td>Set Filter File Path at <code>Vehicles/EgoVehicleSpraySet</code>.</td>
        <td>Set Filter File Path at <code>Vehicles/EgoVehicleSpraySet</code>code>. Do the same setting under <code>Assets/Prefab/CarSpraySet</code>.</td>
    </tr>
    <tr>
        <td colspan="2">Common Setting</td>
        <td colspan="4">
            <ol>
                <li>Set velocity of the ego vehicle in different sequences. See Table 4.</li>
                <li>Set wind field data according to the vehicle velocity at <code>_WindFieldLoader</code>. For example, the wind field of each sequence  mentioned in our method shows in Table 4. Note that wind field data is placed at <code>Assets/WindFieldData</code>. See Table 4.</li>
            </ol>
        </td>
    </tr>
</table>

<p class="tb-title">Table 4</p>
<table>
    <tr>
        <th>Sequence Index / Name</th>
        <th>Abbreviation</th>
        <th>Velocity (m/s)</th>
        <th>Correspond Wind Field</th>
    </tr>
    <tr>
        <td>1 / <b>seg</b>ment-<b>29</b>74991090366925955</td>
        <td>seg29</td>
        <td>29</td>
        <td>veh29_Scaled_with_Primitives0.19.csv</td>
    </tr>
    <tr>
        <td>2 / <b>seg</b>ment-<b>31</b>32521568089292927</td>
        <td>seg31</td>
        <td>20</td>
        <td>veh20_0.20.csv</td>
    </tr>
    <tr>
        <td>3 / <b>seg</b>ment-<b>51</b>21298817582693383</td>
        <td>seg51</td>
        <td>29</td>
        <td>veh29_Scaled_with_Primitives0.19.csv</td>
    </tr>
    <tr>
        <td>4 / <b>seg</b>ment-<b>111</b>39647661584646830</td>
        <td>seg111</td>
        <td>21</td>
        <td>veh21_0.20.csv</td>
    </tr>
    <tr>
        <td>5 / <b>seg</b>ment-<b>138</b>30510593707564159</td>
        <td>seg138</td>
        <td>25</td>
        <td>veh25_0.20.csv</td>
    </tr>
</table>

### Usage
1. Open this folder with Unity.
2. Open the scene named "SpraySimple".
3. Check the parameters mentioned above.
4. `Navigation bar` > `Window` > `General` > `Recorder` > `Recorder Window`, open the recorder window.
5. Set parameters in the recorder window. Table 5 shows parameters used in our method.
6. Start recording.

<p class="tb-title">Table 5. Parameters for the recorder.</p>
<table>
    <tr>
        <th>Parameter</th>L
        <th>Value</th>
    </tr>
    <tr>
        <td>Recording Mode</td>
        <td>Frame Interval</td>
    </tr>
    <tr>
        <td>Start</td>
        <td>0</td>
    </tr>
    <tr>
        <td>End</td>
        <td>220</td>
    </tr>
    <tr>
        <td>Target FPS</td>
        <td>Custom</td>
    </tr>
    <tr>
        <td>Value</td>
        <td>10</td>
    </tr>
</table>

## Data Processing - Synthesis
* This part of details could apply to 2 systhesis scripts(i.e. `Tools/Synthesis_texture_based` and `Tools/Synthesis_particle_based`)
* Before synthesizing, generate filter_nodup.txt first. The followings show the steps:
    1. Open `Tools/concat_file.py`
        a. Set input_directory as your `OutputFilterPath` in simulation’s .env above.
        b. Set `output_file` name. Assume the name is “filter_new.txt”
        c. Execute this script.
    2. In command prompt, execute `cat -n <your path to filter_new.txt> | sort -uk2 | sort -nk1 | cut -f2- > filter_nodup.txt`. Done.
*  Set the following parameters in main.cpp before building
    * `filterFileName`: the path to the filter_nodup.txt
* Other details (including in/output) are noted in main.cpp.
* An example of filter_nodup.txt (i.e. output) is provided.

## Data Processing - Global Refinement
* We rename the original .pcd files to ``<segment name>_<4 digit id>``. For example, rename `1518656408.389677047.pcd` to `segment-2974991090366925955_0000.pcd` or `seg29_0000.pcd`.
* We use CloudCompare in this part. Please make sure you’ve downloaded it.
* Set the following parameters in main.cpp before building
    * `groundTruthDirectory`: the folder stores renamed ground-truth .pcd
    * `simulationResultDirectory`: the folder stores simulated .pcd
* Other details (including in/output) are noted in main.cpp.
* An example of filter.txt (i.e. output) is provided.

## Data Augumentation
* Main file location: `Tools/PCD_MergePointCloud`
* Usage: Merge point clouds.
* Other details are noted in main.cpp.
* We use [RapidJSON](https://github.com/Tencent/rapidjson/) as the JSON parser.

# Contributors
* [Yi Chien "Alan" Shih](https://github.com/alan0201tw)
