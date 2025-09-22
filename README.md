# Quarter-Car Suspension Modeling and Analysis (MATLAB)

State-space, 2-DOF quarter-car (sprung + unsprung) solved with `ode45`. 

## Files
- `QuarterCarModel_ODE45_FSAE_V1.mlx` – Live Script (original)
- `QuarterCarModel_ODE45_FSAE.mlx` – Live Script (extension)
- `QuarterCarModel_ODE45_FSAE_V1.m` – Script version
- `QuarterCarModel_ODE45_FSAE.m` – Script version
- `FSAE Intern Project Report_Quarter Car Model.pdf` – Full write-up

## Run
1) Open **`QuarterCarModel_ODE45_FSAE.mlx`** in MATLAB  
2) Run all sections (simulates **Sedan**, **Formula**, **Truck** by changing parameters).  
3) Plots generated: body displacement (`x1`), wheel displacement (`x3` presented as `x2`).

## Concepts
- **States**: `[x1, x1_dot, x3, x3_dot]`, positive **up**.  
- **Inputs**: `u1=1000 N` on `m1` and `u2=500 N` on `m2` from *t* = 1.5–2.0 s.  
- **Suspension:** `x_s = x1 - x3` ( `>0` extension, `<0` compression ); tire (road fixed at 0).

## Natural Frequency (conceptual)
The **undamped natural frequency** is the rate a system would oscillate with **no damping**.  
The script also prints a common **1-DOF body-mode approximation** using `kc` and `m1`.

## Report
See **`FSAE Intern Project Report_Quarter Car Model.pdf`** for methodology, figures, and results.
