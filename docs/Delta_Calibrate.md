# Delta Calibration

This document describes Klipper's automatic calibration system for
"delta" style printers.

Delta calibration involves finding an accurate geometric model of the printer,
including tower endstop positions, tower angles, the main delta radius,
individual arm lengths, per-tower radius adjustments, and per-tower lean angles.
These settings control printer motion on a delta printer. Each one of these
parameters has a non-obvious and non-linear impact, and it is difficult to
calibrate them manually. In contrast, the software calibration code, especially
when using advanced solvers, can provide excellent results.

Klipper's delta calibration can also account for probe tilt compensation if an
automatic probe with a Z-offset is used, further improving accuracy by
correcting the XY location of probed points.

Ultimately, the delta calibration is dependent on the precision of the
tower endstop switches and the accuracy of the Z probe. If one is using
Trinamic stepper motor drivers then consider enabling [endstop
phase](Endstop_Phase.md) detection to improve the accuracy of those switches.

## Important: SciPy Dependency for Advanced Calibration

The enhanced delta calibration routine, which solves for an extended set of
parameters including tower leans, performs best when the `SciPy` Python library
is available. If `SciPy` is not installed in Klipper's Python environment, the
calibration will fall back to a simpler optimization algorithm which may be less
effective for this complex model.

It is **highly recommended** to install SciPy:
```
~/klippy-env/bin/pip install scipy
```
(Adjust the path to your Klipper virtual environment if necessary.)

## Probing Methods: Automatic vs Manual

Klipper supports calibrating the delta parameters via a manual probing
method (using the "paper test" at the nozzle) or via an automatic Z probe.

*   **Automatic Z Probes:**
    *   A number of delta printer kits come with automatic Z probes that are
        not sufficiently accurate (specifically, small differences in arm
        length can cause effector tilt which can skew an automatic probe).
    *   If using an automatic probe then first
        [calibrate the probe](Probe_Calibrate.md) and then check for a
        [probe location bias](Probe_Calibrate.md#location-bias-check). If the
        automatic probe has a bias of more than 25 microns (.025mm) then use
        manual probing instead for initial geometry calibration.
    *   **Probe Tilt Compensation:** If your probe has a Z-offset (distance
        between nozzle tip and probe trigger point along the Z-axis), the
        effector's natural tilt during XY moves can cause the probe to trigger
        at an XY location slightly different from the nozzle's XY position plus
        the probe's configured XY offsets. Klipper can compensate for this if
        the `delta_effector_radius` is correctly set in the `[printer]`
        section of your config. This compensation is automatically applied
        during `DELTA_CALIBRATE` if an automatic probe with a Z-offset (greater
        than a small threshold, e.g. 0.5mm) is used.
    *   If using a probe that is mounted on the side of the hotend (that is,
        it has an X or Y offset) then note that performing delta calibration
        will invalidate the results of probe calibration. These types of
        probes are rarely suitable for use on a delta. If using the probe
        anyway, then be sure to rerun probe calibration after any delta
        calibration.

*   **Manual Probing (`DELTA_CALIBRATE METHOD=manual`):**
    *   This method uses the nozzle tip and the "paper test" (see [Bed
        Leveling](Bed_Level.md#the-paper-test)).
    *   It eliminates errors from probe inaccuracies or probe XY offsets.
    *   The probe tilt compensation for XY displacement is heuristically
        disabled during manual probing, as the Z-offset is effectively zero.

## Understanding Delta Kinematic Parameters

Klipper's advanced delta calibration adjusts the following parameters:

*   **`[printer]` section:**
    *   `delta_radius`: The main radius of the printer, from the bed center to
        the nominal XY plane of the towers.
    *   `delta_effector_radius`: (New) The distance from the nozzle tip to the
        center of the effector's arm joints (e.g., U-joints or ball bearings),
        measured in the plane of the effector. Crucial for probe tilt
        compensation.

*   **`[stepper_a]`, `[stepper_b]`, `[stepper_c]` sections:**
    *   `position_endstop`: The carriage height along the rail when the endstop
        triggers.
    *   `angle`: The XY angular position of the tower (e.g., A=210, B=330, C=90
        degrees). The calibration typically adjusts two of these relative to the
        third.
    *   `arm_length`: The length of the diagonal rod for that specific tower.
        Individual arm lengths are now calibrated by the standard
        `DELTA_CALIBRATE` command when using the SciPy solver.
    *   `delta_radius_offset`: (New) A per-tower adjustment (in mm) added to
        the global `delta_radius`. Allows for fine-tuning each tower's
        effective radial position.
    *   `radial_lean`: (New) Angle (in degrees) the tower leans along its
        radial line (positive = outwards).
    *   `tangential_lean`: (New) Angle (in degrees) the tower leans
        tangentially (perpendicular to radial, positive = CCW).

These lean parameters help account for towers that are not perfectly vertical.

## Performing Delta Calibration (`DELTA_CALIBRATE`)

The `DELTA_CALIBRATE` command probes the bed and calculates new values for
all the above geometric parameters (endstops, angles, delta_radius, individual
arm_lengths, delta_radius_offsets, and tower leans).

**Initial Configuration:**
*   Your `printer.cfg` must have initial estimates for `delta_radius` and per-stepper `arm_length`, `angle`, and `position_endstop`. These should be accurate to within a few millimeters or degrees. Most delta printer kits provide these.
*   Also, set `delta_effector_radius` in the `[printer]` section. Measure this on your printer: it's the distance from the nozzle tip to the center of where your diagonal rods connect to the effector, in the plane parallel to the effector.
*   The new parameters `delta_radius_offset`, `radial_lean`, and `tangential_lean` can be omitted from the initial config as they will default to 0.

**Probing Strategy for Advanced Calibration:**
*   The default 7-point probing pattern is often insufficient to accurately
    determine the full set of ~20 parameters (including 6 for tower leans).
*   **It is highly recommended to define a custom, more comprehensive probing
    pattern using the `points` option in the `[delta_calibrate]` section.**
    *   Aim for 15-25+ points.
    *   Ensure points are well-distributed across the printable radius.
    *   **Crucially, to help identify tower lean parameters, include probe points
        at different Z heights if your setup allows meaningful Z variation during
        probing (this is advanced and may require temporary adjustments or careful
        setup).** If probing at different Z heights is not feasible, a larger number
        of XY points is even more critical.
    *   Example for more XY points:
        ```
        [delta_calibrate]
        radius: 100 # Max probe radius
        points:
          0,0
          70,0
          35,60.62
          -35,60.62
          -70,0
          -35,-60.62
          35,-60.62
          # Add an inner ring
          35,0
          17.5,30.31
          -17.5,30.31
          -35,0
          -17.5,-30.31
          17.5,-30.31
        ```

**Running the Calibration:**
1.  Ensure your printer is homed (`G28`).
2.  During the delta calibration process it may be necessary for the
    printer to probe below what would otherwise be considered the plane of
    the bed. It is typical to permit this during calibration by updating
    the config so that the printer's `minimum_z_position=-5`. (Once
    calibration completes, one can remove this setting from the config.)
3.  Run the calibration:
    *   For automatic probe: `DELTA_CALIBRATE`
    *   For manual probing: `DELTA_CALIBRATE METHOD=manual`
    The manual method will guide you through the paper test for each point.
4.  After probing, the system will use an advanced optimization algorithm (SciPy's
    L-BFGS-B or Nelder-Mead if SciPy is installed, otherwise a simpler coordinate
    descent) to find the parameters that best fit the probed data. This may take
    some time (tens of seconds to a few minutes depending on the host system and
    number of points).
5.  The calculated parameters will be reported. Save them by running:
    `SAVE_CONFIG`
    This will update your `printer.cfg` and restart Klipper.

The calibration should result in a more accurate printer model. If issues persist,
consider the quality of your probe data and the mechanical soundness of your printer.

## Role of `DELTA_ANALYZE` (Enhanced Calibration with Printed Object)

Previously, `DELTA_ANALYZE CALIBRATE=extended` was the primary way to calibrate
individual arm lengths using measurements from a printed object.

With the new `DELTA_CALIBRATE` being more powerful (calibrating arm lengths,
radius offsets, and tower leans directly from probe data, especially with SciPy),
the role of `DELTA_ANALYZE` might shift:
*   It can still be used to incorporate physical measurements from a printed
    object (`docs/prints/calibrate_size.stl`) to fine-tune *all* parameters
    together, potentially improving dimensional accuracy beyond what Z-probe
    data alone can achieve.
*   If `DELTA_CALIBRATE` (with a good probing strategy) yields satisfactory
    results, `DELTA_ANALYZE` might become an optional step for those seeking
    the highest level of dimensional accuracy validation or who prefer the
    measurement-based workflow.
*   The `DELTA_ANALYZE` command uses the same underlying `calculate_params`
    function, so it also benefits from the SciPy solver if available.

Follow the existing instructions for printing the calibration object and taking
measurements if you choose to use `DELTA_ANALYZE`.

## Using Bed Mesh on a Delta

It is possible to use [bed mesh](Bed_Mesh.md) on a delta. However, it
is important to obtain good delta calibration *prior* to enabling a bed
mesh. Running bed mesh with poor delta calibration will result in
confusing and poor results.

Note that performing delta calibration will invalidate any previously
obtained bed mesh. After performing a new delta calibration be sure to
rerun `BED_MESH_CALIBRATE`.
The `BED_MESH_OUTPUT` command, when used with a circular bed defined by
`mesh_radius`, will now output `Z: nan` for points outside the defined circle,
which should improve rendering in compatible visualizer tools.
