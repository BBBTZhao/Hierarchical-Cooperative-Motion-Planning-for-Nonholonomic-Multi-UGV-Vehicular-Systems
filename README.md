Cooperative motion planning for nonholonomic multi-UGV vehicular systems remains challenging because dense inter-vehicle conflicts must be resolved under vehicle kinematic limits and restricted onboard computation. This paper proposes a centralized–distributed hierarchical framework that decomposes the coupled problem into vehicle-feasible global path coordination, conflict-zone-based velocity coordination, and reference-guided distributed local motion planning. First, an offline motion primitive library is embedded into a motion-primitive-enhanced priority-based search planner with collision-aware FOCAL initialization, implicit-constraint-guided branching, lazy branch pruning, and event-triggered incremental replanning. Second, Type-I and Type-II conflict zones are formulated in the spatial domain, and their logical passing-order constraints are conservatively reformulated by log-sum-exp (LSE) approximations as differentiable nonlinear constraints without binary variables. Third, globally coordinated references are used to construct decoupled convex safe spaces, while pop-up obstacles are handled by Minkowski-difference-based smooth distance constraints and a constraint-relaxed sequential quadratic programming (SQP) solver. Across 50 randomized trials per scenario, both centralized layers achieved a 100\% success rate. Real-world experiments with Ackermann-steered UGVs further showed average onboard local-planning times of 35.2–48.6 ms on resource-constrained embedded processors, demonstrating robust coordination and real-time deployability.

---

# Simulation Demonstrations

## Global Path Planning

<table>
<tr>
<td align="center">
<img src="Global_path_planning_Intersection.gif" width="300"><br>
<b>Intersection Scenario</b>
</td>

<td align="center">
<img src="Global_path_planning_NarrowChannel.gif" width="300"><br>
<b>Narrow Channel Scenario</b>
</td>

<td align="center">
<img src="Global_path_planning_RandomObstacles.gif" width="300"><br>
<b>Random Obstacles Scenario</b>
</td>
</tr>
</table>

---

## Global Velocity Coordination

<table>
<tr>
<td align="center">
<img src="Global_velocity_coordination_Intersection.gif" width="300"><br>
<b>Intersection Scenario</b>
</td>

<td align="center">
<img src="Global_velocity_coordination_NarrowChannel.gif" width="300"><br>
<b>Narrow Channel Scenario</b>
</td>

<td align="center">
<img src="Global_velocity_coordination_RandomObstacles.gif" width="300"><br>
<b>Random Obstacles Scenario</b>
</td>
</tr>
</table>

---

## Local Motion Planning

<table>
<tr>
<td align="center">
<img src="Local_motion_planning_Intersection.gif" width="300"><br>
<b>Intersection Scenario</b>
</td>

<td align="center">
<img src="Local_motion_planning_NarrowChannel.gif" width="300"><br>
<b>Narrow Channel Scenario</b>
</td>

<td align="center">
<img src="Local_motion_planning_RandomObstacles.gif" width="300"><br>
<b>Random Obstacles Scenario</b>
</td>
</tr>
</table>
