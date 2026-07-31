# Hierarchical Cooperative Motion Planning for Nonholonomic Multi-UGV Vehicular Systems

## Centralized Spatiotemporal Coordination and Distributed Local Planning

This repository provides supplementary experimental materials for the paper **“Hierarchical Cooperative Motion Planning for Nonholonomic Multi-UGV Vehicular Systems: Centralized Spatiotemporal Coordination and Distributed Local Planning.”** It complements the experimental section of the paper with layer-wise simulation demonstrations, additional parameter settings, an ablation study of MP-EPBS, per-UGV collision-constraint slack statistics, and real-vehicle videos and trajectory results.

Cooperative motion planning for nonholonomic multi-UGV vehicular systems remains challenging because dense inter-vehicle conflicts must be resolved under vehicle kinematic limits and restricted onboard computation. The proposed centralized–distributed hierarchical framework decomposes the coupled planning problem into three connected layers:

1. **Centralized global path coordination:** vehicle-feasible motion primitives are integrated with an enhanced priority-based search framework to resolve spatial conflicts.
2. **Centralized global velocity coordination:** Type-I and Type-II conflict zones are formulated in the spatial domain, and binary-free log-sum-exp approximations are used to coordinate vehicle passing orders and velocity profiles.
3. **Reference-guided distributed local motion planning:** each UGV independently replans over a receding horizon using globally coordinated spatiotemporal references, convex safe spaces, and a constraint-relaxed SQP solver.

The simulation evaluation contains three representative scenarios: **Random-Obstacle**, **Intersection**, and **Narrow-Channel**. Statistical results are averaged over 50 randomized trials per scenario, while the GIFs and plotted profiles show representative runs. Real-vehicle experiments further evaluate the complete framework on Ackermann-steered UGVs.

---

# 1. Experimental Settings and Quantitative Results

## 1.1 Global Path Coordination

For collision checking in the global path layer, static obstacles are inflated outward by 0.08 m. Each UGV footprint is additionally enlarged by 0.05 m in the longitudinal direction and 0.04 m in the lateral direction. These conservative geometric margins provide extra clearance between UGVs and static obstacles.

**Table 1. Parameter settings for global path coordination.**

| Symbol | Description | Value |
|---|---|---:|
| `L_w` | Wheelbase | 2.78 m |
| `L_b` | Vehicle width | 1.85 m |
| `L_f` | Front overhang | 0.915 m |
| `L_r` | Rear overhang | 0.915 m |
| `n_mp` | Number of samples per motion primitive | 7 |
| `k_mp` | Duration of each motion primitive | 2.5 s |
| `K_mp` | Number of candidate motion primitives | 10 |
| `w` | FOCAL suboptimality factor | 1.05 |
| `w_pp` | Obstacle-aware heuristic weight | 1.4 |
| `v_ref` | Reference velocity | 4 m/s |
| `a_max` | Maximum longitudinal acceleration | 3 m/s² |
| `delta_max` | Maximum steering angle | 30° |
| `alpha_max` | Maximum steering rate | 30°/s |

### MP-EPBS Ablation Configurations

Five cumulative configurations are evaluated:

- **MP-PBS:** motion-primitive implementation of the basic priority-based search framework.
- **MP-PBS-F:** adds collision-aware FOCAL initialization to reduce initial inter-UGV conflicts during low-level search.
- **MP-PBS-F-LBP:** further introduces lazy branch pruning, expanding one priority branch first and suspending the symmetric branch until backtracking is required.
- **MP-PBS-F-LBP-IC:** additionally applies implicit-constraint-guided branch selection to prioritize branches that resolve more transitive priority relations.
- **MP-EPBS:** the complete method, further incorporating soft references in the collision-aware heuristic and event-triggered incremental updating.

**Table 2. Ablation results in the Random-Obstacle scenario.**

| Algorithm | Success (%) | Avg. Time (s) | Avg. High-Level Expansions | Avg. Planner Calls | Avg. Path Cost |
|---|---:|---:|---:|---:|---:|
| MP-PBS | 100 | 8.80 | 31.24 | 48.76 | 2239.46 |
| MP-PBS-F | 100 | 6.15 | 20.96 | 32.66 | 2247.40 |
| MP-PBS-F-LBP | 100 | 3.83 | 11.66 | 22.96 | 2281.33 |
| MP-PBS-F-LBP-IC | 100 | 3.78 | 10.68 | 23.82 | 2284.27 |
| **MP-EPBS** | **100** | **2.68** | **6.24** | **17.66** | **2282.38** |

**Table 3. Ablation results in the Intersection scenario.**

| Algorithm | Success (%) | Avg. Time (s) | Avg. High-Level Expansions | Avg. Planner Calls | Avg. Path Cost |
|---|---:|---:|---:|---:|---:|
| MP-PBS | 100 | 0.73 | 7.32 | 12.52 | 561.6188 |
| MP-PBS-F | 100 | 0.77 | 6.48 | 11.56 | 562.7781 |
| MP-PBS-F-LBP | 100 | 0.56 | 4.00 | 9.06 | 570.2307 |
| MP-PBS-F-LBP-IC | 100 | 0.54 | 3.94 | 9.14 | 571.0052 |
| **MP-EPBS** | **100** | **0.44** | **2.66** | **7.66** | **569.4933** |

**Table 4. Ablation results in the Narrow-Channel scenario.**

| Algorithm | Success (%) | Avg. Time (s) | Avg. High-Level Expansions | Avg. Planner Calls | Avg. Path Cost |
|---|---:|---:|---:|---:|---:|
| MP-PBS | 100 | 5.09 | 16.16 | 25.90 | 824.73 |
| MP-PBS-F | 100 | 6.80 | 13.60 | 20.74 | 825.64 |
| MP-PBS-F-LBP | 100 | 3.34 | 7.32 | 12.72 | 839.28 |
| MP-PBS-F-LBP-IC | 100 | 3.21 | 5.90 | 13.44 | 837.59 |
| **MP-EPBS** | **100** | **2.38** | **5.06** | **11.14** | **834.35** |

The cumulative ablation results show that the proposed components progressively reduce redundant high- and low-level searches. Collision-aware FOCAL initialization reduces the initial conflict burden, lazy branch pruning limits unnecessary symmetric expansions, and implicit-constraint-guided branching improves high-level conflict resolution. The complete MP-EPBS achieves the lowest average computation time, high-level expansions, and planner calls in all three scenarios. Relative to MP-PBS, its average path-cost increase remains below 2%, while all configurations retain a 100% success rate.

---

## 1.2 Global Velocity Coordination

The global velocity-coordination layer optimizes vehicle velocities along the globally coordinated paths. The following parameters specify the spatial discretization, vehicle limits, and safety margins used in the conflict-zone-based formulation.

**Table 5. Parameter settings for global velocity coordination.**

| Symbol | Description | Value |
|---|---|---:|
| `L_w` | Wheelbase | 2.78 m |
| `L_b` | Vehicle width | 1.85 m |
| `L_f` | Front overhang | 0.915 m |
| `L_r` | Rear overhang | 0.915 m |
| `Delta s` | Spatial discretization step | 1 m |
| `v_ref` | Reference velocity | 4 m/s |
| `v_min` | Minimum velocity | 0 m/s |
| `v_max` | Maximum velocity | 6 m/s |
| `a_t,max` | Maximum longitudinal acceleration | 3 m/s² |
| `a_n,max` | Maximum lateral acceleration | 3 m/s² |
| `t_safety` | Safety time gap | 0.75 s |
| `d_offset` | Spatial safety distance | 5.0 m |

---

## 1.3 Distributed Local Motion Planning

The local planner uses the globally coordinated spatiotemporal references as nominal guidance and replans independently for each UGV over a receding horizon. Its main parameters are listed below.

**Table 6. Parameter settings for distributed local motion planning.**

| Symbol | Description | Value |
|---|---|---:|
| `L_w` | Wheelbase | 2.78 m |
| `L_b` | Vehicle width | 1.85 m |
| `L_f` | Front overhang | 0.915 m |
| `L_r` | Rear overhang | 0.915 m |
| `v_ref` | Reference velocity | 4 m/s |
| `v_min` | Minimum velocity | 0 m/s |
| `v_max` | Maximum velocity | 6 m/s |
| `a_t,max` | Maximum longitudinal acceleration | 3 m/s² |
| `a_n,max` | Maximum lateral acceleration | 3 m/s² |
| `delta_max` | Maximum steering angle | 30° |
| `alpha_max` | Maximum steering rate | 30°/s |
| `d_safety` | Safety clearance | 0.08 m |
| `n_p` | Prediction horizon length | 40 |
| `Delta t` | Sampling interval | 0.1 s |
| `N_SQP,max` | Maximum SQP iterations | 3 |

---

# 2. Simulation Experiments

The simulation results are organized by scenario. For each scenario, the three GIFs visualize the outputs of the global path-coordination, global velocity-coordination, and distributed local-planning layers. The additional plots then provide the corresponding layer-wise velocity, steering, and safety information.

The global-path and global-velocity plots are presented as complementary outputs of the two centralized layers rather than as a direct before-and-after comparison.

---

## 2.1 Random-Obstacle Scenario

This scenario contains 12 UGVs approaching an obstacle-rich interaction region from multiple directions. It is used to evaluate large-scale spatial conflict resolution, coordinated velocity assignment, and distributed local replanning in the presence of irregular static obstacles.

### Layer-Wise Demonstrations

<table>
<tr>
<td align="center" width="33%">
<img src="Global_path_planning_RandomObstacles.gif" width="300"><br>
<b>Global Path Coordination</b><br>
Vehicle-feasible paths generated by MP-EPBS.
</td>
<td align="center" width="33%">
<img src="Global_velocity_coordination_RandomObstacles.gif" width="300"><br>
<b>Global Velocity Coordination</b><br>
Conflict-zone-based coordinated velocity assignment.
</td>
<td align="center" width="33%">
<img src="Local_motion_planning_RandomObstacles.gif" width="300"><br>
<b>Distributed Local Motion Planning</b><br>
Reference-guided receding-horizon replanning.
</td>
</tr>
</table>

### Centralized-Layer Velocity Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig1.png" width="470"><br>
<b>Velocity profile associated with global path coordination</b>
</td>
<td align="center" width="50%">
<img src="figs/fig2.png" width="470"><br>
<b>Velocity profile generated by global velocity coordination</b>
</td>
</tr>
</table>

The first plot reports the velocity information associated with the globally planned paths, while the second shows the conflict-resolved velocity profile produced by the global velocity-coordination layer. Together, they clarify the outputs passed through the two centralized planning stages.

### Distributed Local-Planning Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig7.png" width="470"><br>
<b>Local-planning velocity profiles</b>
</td>
<td align="center" width="50%">
<img src="figs/fig8.png" width="470"><br>
<b>Local-planning front-wheel steering-angle profiles</b>
</td>
</tr>
</table>

The local velocity and steering-angle profiles show how individual UGVs adjust their motions around the obstacle-rich interaction region while following the globally coordinated references.

**Table 7. Maximum collision-constraint slack for each UGV in the Random-Obstacle scenario.**

| UGV No. | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| Max. slack (m) | 0 | 0.003 | 0 | 0.005 | 0.007 | 0 |
| UGV No. | 7 | 8 | 9 | 10 | 11 | 12 |
| Max. slack (m) | 0.005 | 0.008 | 0.010 | 0 | 0.002 | 0.003 |

All 12 UGVs complete the multi-directional crossing task. The maximum collision-constraint slack is 0.010 m, which remains well below the prescribed safety clearance of 0.08 m.

---

## 2.2 Intersection Scenario

This scenario contains six UGVs entering a structured intersection from different directions. It evaluates passing-order coordination under crossing and merging interactions, together with executable local motion generation.

### Layer-Wise Demonstrations

<table>
<tr>
<td align="center" width="33%">
<img src="Global_path_planning_Intersection.gif" width="300"><br>
<b>Global Path Coordination</b><br>
Vehicle-feasible paths through the intersection.
</td>
<td align="center" width="33%">
<img src="Global_velocity_coordination_Intersection.gif" width="300"><br>
<b>Global Velocity Coordination</b><br>
Conflict-zone-based passing-order and velocity coordination.
</td>
<td align="center" width="33%">
<img src="Local_motion_planning_Intersection.gif" width="300"><br>
<b>Distributed Local Motion Planning</b><br>
Onboard reference tracking and local replanning.
</td>
</tr>
</table>

### Centralized-Layer Velocity Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig3.png" width="470"><br>
<b>Velocity profile associated with global path coordination</b>
</td>
<td align="center" width="50%">
<img src="figs/fig4.png" width="470"><br>
<b>Velocity profile generated by global velocity coordination</b>
</td>
</tr>
</table>

These plots separately present the velocity information from the global path and global velocity layers. The coordinated profiles reflect the temporal ordering required to resolve intersection conflicts without binary decision variables.

### Distributed Local-Planning Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig9.png" width="470"><br>
<b>Local-planning velocity profiles</b>
</td>
<td align="center" width="50%">
<img src="figs/fig10.png" width="470"><br>
<b>Local-planning front-wheel steering-angle profiles</b>
</td>
</tr>
</table>

The local-planning results show continuous velocity evolution and bounded steering responses as the six UGVs traverse the shared intersection.

**Table 8. Maximum collision-constraint slack for each UGV in the Intersection scenario.**

| UGV No. | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| Max. slack (m) | 0 | 0.007 | 0.005 | 0 | 0 | 0.013 |

The maximum collision-constraint slack is 0.013 m, which remains below the prescribed safety clearance of 0.08 m.

---

## 2.3 Narrow-Channel Scenario

This scenario contains six UGVs passing through a bidirectional constrained corridor. It emphasizes conflict resolution in a geometrically restricted environment where priority decisions and velocity coordination strongly affect subsequent replanning.

### Layer-Wise Demonstrations

<table>
<tr>
<td align="center" width="33%">
<img src="Global_path_planning_NarrowChannel.gif" width="300"><br>
<b>Global Path Coordination</b><br>
Vehicle-feasible paths through the constrained passage.
</td>
<td align="center" width="33%">
<img src="Global_velocity_coordination_NarrowChannel.gif" width="300"><br>
<b>Global Velocity Coordination</b><br>
Passing-order and velocity coordination in the shared channel.
</td>
<td align="center" width="33%">
<img src="Local_motion_planning_NarrowChannel.gif" width="300"><br>
<b>Distributed Local Motion Planning</b><br>
Locally executable motion generation in the narrow space.
</td>
</tr>
</table>

### Centralized-Layer Velocity Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig5.png" width="470"><br>
<b>Velocity profile associated with global path coordination</b>
</td>
<td align="center" width="50%">
<img src="figs/fig6.png" width="470"><br>
<b>Velocity profile generated by global velocity coordination</b>
</td>
</tr>
</table>

The two profiles provide complementary views of the centralized outputs. The global path layer supplies vehicle-feasible references, while the velocity layer coordinates entry and passage through the shared channel.

### Distributed Local-Planning Profiles

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig11.png" width="470"><br>
<b>Local-planning velocity profiles</b>
</td>
<td align="center" width="50%">
<img src="figs/fig12.png" width="470"><br>
<b>Local-planning front-wheel steering-angle profiles</b>
</td>
</tr>
</table>

The local profiles illustrate the speed adjustments and bounded steering commands used by the UGVs while negotiating the bidirectional constrained corridor.

**Table 9. Maximum collision-constraint slack for each UGV in the Narrow-Channel scenario.**

| UGV No. | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---:|---:|---:|---:|---:|---:|
| Max. slack (m) | 0.001 | 0.004 | 0.028 | 0 | 0.001 | 0.007 |

All six UGVs pass through the constrained corridor while maintaining bounded velocity and steering responses. The maximum collision-constraint slack is 0.028 m, which remains below the prescribed safety clearance of 0.08 m.

---

# 3. Real-Vehicle Experiments

The real-vehicle experiments evaluate the complete centralized–distributed framework on Ackermann-steered UGVs. For each scenario, the video shows the physical experiment, while the 3-D spatiotemporal trajectory, 2-D top-view trajectory, and measured velocity profiles provide complementary quantitative views of the vehicle motions.

## 3.1 Random-Obstacle Scenario

[▶ Watch the Random-Obstacle real-vehicle experiment](<Real-Vehicle Experiment：Random Obstacles.mp4>)

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig13.png" width="470"><br>
<b>3-D spatiotemporal trajectories</b>
</td>
<td align="center" width="50%">
<img src="figs/fig14.png" width="470"><br>
<b>2-D top-view trajectories</b>
</td>
</tr>
</table>

<p align="center">
<img src="figs/fig19.png" width="650"><br>
<b>Measured velocity profiles</b>
</p>

The 3-D trajectories visualize the temporal evolution of the vehicle motions, while the top-view trajectories show the spatial paths through the obstacle-rich environment. The measured velocity profiles complement the video by showing the executable speed evolution of the physical UGVs.

---

## 3.2 Intersection Scenario

[▶ Watch the Intersection real-vehicle experiment](<Real-Vehicle Experiment：Intersection.mp4>)

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig15.png" width="470"><br>
<b>3-D spatiotemporal trajectories</b>
</td>
<td align="center" width="50%">
<img src="figs/fig16.png" width="470"><br>
<b>2-D top-view trajectories</b>
</td>
</tr>
</table>

<p align="center">
<img src="figs/fig20.png" width="650"><br>
<b>Measured velocity profiles</b>
</p>

The trajectory plots show how the vehicles execute the coordinated passing order through the shared intersection. The measured velocity profiles provide the corresponding temporal motion information during the physical experiment.

---

## 3.3 Narrow-Channel Scenario

[▶ Watch the Narrow-Channel real-vehicle experiment](<Real-Vehicle Experiment：Narrow Channel.mp4>)

<table>
<tr>
<td align="center" width="50%">
<img src="figs/fig17.png" width="470"><br>
<b>3-D spatiotemporal trajectories</b>
</td>
<td align="center" width="50%">
<img src="figs/fig18.png" width="470"><br>
<b>2-D top-view trajectories</b>
</td>
</tr>
</table>

<p align="center">
<img src="figs/fig21.png" width="650"><br>
<b>Measured velocity profiles</b>
</p>

The 3-D and 2-D trajectories show the coordinated bidirectional traversal of the constrained passage. The measured velocity profiles illustrate the speed adjustments required to maintain safe and executable motion in the narrow environment.

---

# 4. File Organization

```text
.
├── figs/
│   ├── fig1.png ... fig6.png       # Centralized-layer velocity profiles
│   ├── fig7.png ... fig12.png      # Local velocity and steering profiles
│   ├── fig13.png ... fig18.png     # Real-vehicle 3-D and 2-D trajectories
│   └── fig19.png ... fig21.png     # Real-vehicle velocity profiles
├── Global_path_planning_*.gif
├── Global_velocity_coordination_*.gif
├── Local_motion_planning_*.gif
├── Real-Vehicle Experiment：Random Obstacles.mp4
├── Real-Vehicle Experiment：Intersection.mp4
└── Real-Vehicle Experiment：Narrow Channel.mp4
```

The repository is intended as an experimental supplement to the paper. The GIFs and videos provide representative visual demonstrations, while Tables 1–9 and the additional plotted results provide the parameter settings, ablation evidence, motion profiles, and safety-related statistics needed to interpret the experiments.
