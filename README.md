# State Space Control Analysis

## Overview
This project implements **state-space control techniques** for an autonomous vehicle model using **MATLAB**. It covers controllability, observability, and state feedback control, including pole placement and canonical forms.

## Features
- **State-space representation** of the system.
- **Controllability and Observability analysis**.
- **State feedback controller design** using pole placement.
- **Closed-loop system simulation**.
- **Transfer function representation**.
- **Plots of system response**.

## MATLAB Files
- **Autonomous_Vehicle.m**: Defines the system, computes controllability, observability, and implements state feedback control.
- **First.m**: Computes eigenvalues, controllability, and observability.
- **Second.m**: Augmented system analysis and step response.

## Key Components
### 1. **State-Space Representation**
   ```matlab
   A = [-3.785 -19.167 0 0;
        0.469  0.976  0 0;
        0      1      0 0;
        1      0     20 0];
   B = [34.409;27.686;0;0];
   C = [0 0 0 1];
   ```
   Defines the state-space matrices.

### 2. **Controllability & Observability**
   ```matlab
   phi_c = ctrb(A,B);
   rank_c = rank(phi_c);
   phi_o = obsv(A,C);
   rank_o = rank(phi_o);
   ```
   Computes the **controllability** and **observability** matrices and their ranks.

### 3. **State Feedback Control**
   ```matlab
   P_SF = [-2, -3, -4, -5];
   K = acker(A, B, P_SF);
   ```
   Implements pole placement for state feedback control.

### 4. **Closed-loop System Simulation**
   ```matlab
   X0 = [1, 2, 3, 4, 0, 0, 0, 0];
   [Y,T,X] = lsim(SYScl, u, t, X0);
   ```
   Simulates the closed-loop system response.

### 5. **System Response Plots**
   ```matlab
   subplot(2,2,1)
   plot(t, X(:,1),'b')
   hold on
   plot(t, X(:,5),'k--')
   ```
   Generates system response plots for different states.

## Running the Code
1. Open MATLAB and navigate to the project folder.
2. Run `Autonomous_Vehicle.m` to analyze the system and visualize results:
   ```matlab
   run('Autonomous_Vehicle.m')
   ```
3. Modify pole locations in `P_SF` and re-run for different control strategies.

## Results
- The system is **controllable** and **observable**.
- Pole placement achieves the desired closed-loop response.
- Step response plots demonstrate system behavior.

## Future Enhancements
- Implement **LQR control** for optimal performance.
- Extend to **nonlinear vehicle dynamics**.
- Develop a **GUI for interactive control analysis**.

## License
This project is licensed under the **MIT License**.

## Author
Developed by **[Your Name]** ([Your GitHub](https://github.com/your-username)).

---
**Feel free to contribute by submitting pull requests or reporting issues!**

